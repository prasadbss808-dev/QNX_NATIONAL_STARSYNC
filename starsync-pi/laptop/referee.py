#!/usr/bin/env python3
"""
referee.py - the scoreboard AND the live dashboard. Runs on the laptop (172.16.6.100).

    python3 laptop/referee.py            then open  http://localhost:8080  in a browser

It listens on UDP 31910 for plain-text lines from the boards:
    EV <node> <C|S> <seq> <corrected_ns> <raw_ns> <U_ns> <validity>  a pulse a node timestamped
    ST <node> <state> <validity> <U_ns> <raw_off> <corr_off> <drift_ppb> <delay> <rec_ms> <det_ms> <restarts>
    AM <node> <from_ns> <to_ns>      a node takes back stamps (it found a clock step)
    TR <pair> <A|B> <gap_us>         SKY: who really fired first in an ordering pair

How it measures the problem sheet's metrics:
    COMMON pulse = one physical edge seen by both nodes at the same instant, so
    offset = ALTAIR stamp - VEGA stamp is the TRUE inter-node offset.
      max inter-node offset   largest |offset|  (trusted stamps; also all)
      timestamp jitter        standard deviation of the offset
      residual drift rate     slope of the offset over the last 30 s (ppb)
      time outside tolerance  0.1 s per pulse with |offset| > tolerance
      honesty                 trusted pulses where |offset| <= U_vega + U_altair
    ORDERING pair = two edges with a known order and gap (from SKY)
      event-ordering errors   timestamps put them in the wrong order (with confidence)
    recovery time             reported by ALTAIR (link back -> trusted again)
    monotonic timestamps      every node's stamps must keep increasing
If SKY's pulses are not arriving, the offset metrics fall back to what ALTAIR itself
measures over PTP (its corrected offset), and the dashboard says so.

Terminal keys (then Enter):  r = report   c = calibrate   z = reset   q = quit
"""
import argparse, json, os, socket, statistics, sys, threading, time
from collections import deque
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

p = argparse.ArgumentParser()
p.add_argument("--tol", type=float, default=100.0, help="tolerance in us")
p.add_argument("--vega", default="172.16.6.76:31903", help="VEGA command address")
p.add_argument("--altair", default="172.16.6.77:31903", help="ALTAIR command address")
p.add_argument("--port", type=int, default=31910, help="UDP port the boards report to")
p.add_argument("--web", type=int, default=8080, help="dashboard web port (0 = off)")
p.add_argument("--csv", default="/tmp/starsync_pairs.csv")
args = p.parse_args()
TOL = args.tol * 1000.0                                  # ns
HERE = os.path.dirname(os.path.abspath(__file__))

lock = threading.Lock()
pending = {"VEGA": [], "ALTAIR": []}      # COMMON pulses waiting for their partner
stag = {"VEGA": [], "ALTAIR": []}         # ordering pulses waiting for their partner
pairs = []                                # one per COMMON pulse seen by both nodes
stag_pairs, truths = [], []               # ordering pairs and SKY's truth records
status = {}                               # node -> last ST fields
status_hist = deque(maxlen=4000)          # ALTAIR's ST lines over time (2 per second)
events = deque(maxlen=300)                # what happened, for the dashboard log
recoveries = []                           # recovery times (ms)
order = {}
mono = {}                                 # node -> [last stamp, checked, violations]
seen = {"VEGA": {"C": 0, "S": 0}, "ALTAIR": {"C": 0, "S": 0}, "TR": 0}
frozen = {}                                  # node -> [last c, last arrival, frozen count]   # raw counts, for wiring checks
t0 = time.time()
csv = open(args.csv, "w")
csv.write("time_s,offset_us,U_vega_us,U_altair_us,valid_vega,valid_altair\n")


def log_event(text):
    """Adds one line to the dashboard's event log and prints it.
    Why: state changes, restarts, recoveries and errors appear live, with their time.
    """
    events.append((round(time.time() - t0, 1), text))
    print("  * " + text)


def clear_scores():
    """Resets all scores (pairs, ordering, recoveries, monotonic checks) and the clock.
    Why: start a clean measurement after calibrating or before a judged run (key z).
    """
    global t0
    pairs.clear(); stag_pairs.clear(); truths.clear(); recoveries.clear(); status_hist.clear()
    order.update({"ok": 0, "wrong": 0, "close": 0, "unreliable": 0, "naive_wrong": 0, "best_gap": None})
    mono.clear()
    t0 = time.time()


clear_scores()


def trusted(pr):
    """True if both nodes said VALID or PREDICTED for this pulse pair and it was not retracted.
    Why: only trusted stamps count for offset, honesty and tolerance.
    """
    return pr["vv"] in ("VALID", "PREDICTED") and pr["va"] in ("VALID", "PREDICTED") and not pr["suspect"]


# ------------------------------------------------------------------ input from the boards
def on_event(node, kind, c, U, v):
    """One EV line (a node's stamp of a SKY pulse): checks that stamps never go backwards,
    finds the other node's stamp of the same physical edge and stores the pair.
    Why: ALTAIR stamp - VEGA stamp of the SAME edge is the true inter-node offset.
    """
    m = mono.setdefault(node, [0, 0, 0])               # monotonic check per node
    if m[0]:
        m[1] += 1
        if c <= m[0]:
            m[2] += 1
            log_event("%s timestamp went BACKWARDS (%d ns)" % (node, c - m[0]))
    m[0] = c
    other = "ALTAIR" if node == "VEGA" else "VEGA"
    now = time.time()
    f = frozen.setdefault(node, [0, 0.0, 0])            # stamps that barely move = frozen guard
    if f[0] and now - f[1] > 0.05:
        f[2] = f[2] + 1 if c - f[0] < 1e6 else 0
    f[0], f[1] = c, now
    box = pending if kind == "C" else stag
    window = 50e6 if kind == "C" else 5e6             # same physical edge
    for e in box[other]:
        if abs(e["c"] - c) < window:
            box[other].remove(e)
            me = {"c": c, "U": U, "v": v}
            vg, al = (me, e) if node == "VEGA" else (e, me)
            if kind == "C":
                pr = {"t": now - t0, "off": al["c"] - vg["c"], "Uv": vg["U"], "Ua": al["U"],
                      "vv": vg["v"], "va": al["v"], "cv": vg["c"], "ca": al["c"], "suspect": False}
                pairs.append(pr)
                csv.write("%.2f,%.3f,%.1f,%.1f,%s,%s\n" % (pr["t"], pr["off"] / 1e3, pr["Uv"] / 1e3,
                                                          pr["Ua"] / 1e3, pr["vv"], pr["va"]))
            else:
                stag_pairs.append({"arr": now, "cv": vg["c"], "ca": al["c"], "Uv": vg["U"],
                                   "Ua": al["U"], "vv": vg["v"], "va": al["v"], "done": False})
                match_order()
            return
    box[node].append({"c": c, "U": U, "v": v, "arr": now})
    box[node][:] = [e for e in box[node] if now - e["arr"] < 2.0]


def match_order():
    """Pairs each measured ordering pair with SKY's truth (TR) that arrived at about the same time
    and scores it: correct, WRONG, too close to call (gap below U), or unreliable.
    Why: the event-ordering metric of the problem sheet.
    """
    for sp in stag_pairs:
        if sp["done"]:
            continue
        cands = [tr for tr in truths if not tr["used"] and abs(tr["arr"] - sp["arr"]) < 0.3]
        if not cands:
            continue
        best = min(cands, key=lambda tr: abs(tr["arr"] - sp["arr"]))
        best["used"] = sp["done"] = True
        d = sp["ca"] - sp["cv"]                          # > 0: VEGA's edge came first
        naive_first = "A" if d > 0 else "B"
        if naive_first != best["first"]:
            order["naive_wrong"] += 1
        if "UNRELIABLE" in (sp["vv"], sp["va"]):
            order["unreliable"] += 1
        elif abs(d) <= sp["Uv"] + sp["Ua"]:
            order["close"] += 1                          # honest: cannot tell
        elif naive_first == best["first"]:
            order["ok"] += 1
            if order["best_gap"] is None or best["gap"] < order["best_gap"]:
                order["best_gap"] = best["gap"]
        else:
            order["wrong"] += 1
            log_event("ordering ERROR: gap %d us judged the wrong way" % best["gap"])


def handle(line):
    """Parses one text line from a board or SKY (EV, ST, AM, TR) and updates the scores.
    Why: the single entry point for everything the referee hears over UDP 31910.
    """
    w = line.split()
    if not w:
        return
    with lock:
        if w[0] == "EV" and len(w) >= 8:
            if w[1] in ("VEGA", "ALTAIR") and w[2] in ("C", "S"):
                seen[w[1]][w[2]] += 1
            on_event(w[1], w[2], int(w[4]), float(w[6]), w[7])
        elif w[0] == "ST" and len(w) >= 12:
            node, old = w[1], status.get(w[1])
            w.append(time.time())                        # when we heard it
            status[node] = w
            if old and (old[2] != w[2] or old[3] != w[3]):
                log_event("%s: %s %s -> %s %s" % (node, old[2], old[3], w[2], w[3]))
            if old and w[11] != old[11]:
                log_event("%s: WATCHDOG restarted the program (restarts %s)" % (node, w[11]))
            if node == "ALTAIR":
                status_hist.append((time.time() - t0, w[2], w[3], float(w[4]), float(w[5]),
                                    float(w[6]), float(w[7]), float(w[8])))
                if old and w[9] != old[9] and float(w[9]) > 0:
                    recoveries.append(float(w[9]))
                    log_event("ALTAIR recovered in %.0f ms (loss detected after %s ms)" % (float(w[9]), w[10]))
        elif w[0] == "AM" and len(w) >= 4:
            key = "cv" if w[1] == "VEGA" else "ca"
            lo, hi = int(w[2]), int(w[3])
            n = 0
            for pr in pairs:
                if lo <= pr[key] <= hi:
                    pr["suspect"] = True
                    n += 1
            log_event("%s found a clock step and retracted %.0f ms of stamps (%d pulses)" % (w[1], (hi - lo) / 1e6, n))
        elif w[0] == "TR" and len(w) >= 4:
            seen["TR"] += 1
            truths.append({"arr": time.time(), "first": w[2], "gap": int(w[3]), "used": False})
            match_order()


# ------------------------------------------------------------------ metrics
def slope_per_s(ts, ys):
    """Least-squares slope of ys over ts (per second); 0 with fewer than 10 points.
    Why: the residual drift rate = slope of the true offset over the last 30 s.
    """
    if len(ts) < 10:
        return 0.0
    mt, my = sum(ts) / len(ts), sum(ys) / len(ys)
    sxx = sum((t - mt) ** 2 for t in ts)
    return sum((t - mt) * (y - my) for t, y in zip(ts, ys)) / sxx if sxx > 0 else 0.0


def metrics():
    """Computes the problem sheet's metrics: max offset, jitter, residual drift, time outside
    tolerance, honesty, ordering, recovery times, monotonic stamps. Uses SKY's pulses (true
    offset) and falls back to ALTAIR's own PTP view when no pulses arrive.
    Why: one place that turns raw stamps into the numbers the jury asks for.
    """
    m = {"tol_us": args.tol, "pulses": len(pairs)}
    tr = [x for x in pairs if trusted(x)]
    if pairs:                                            # the real thing: SKY's pulses
        m["source"] = "pulses"
        offs = [x["off"] for x in tr]
        m["max_offset_us"] = max((abs(o) for o in offs), default=0) / 1e3
        m["max_offset_all_us"] = max(abs(x["off"]) for x in pairs) / 1e3
        m["jitter_us"] = statistics.pstdev(offs) / 1e3 if len(offs) > 1 else 0.0
        recent = [x for x in tr if x["t"] > pairs[-1]["t"] - 30]
        m["residual_drift_ppb"] = slope_per_s([x["t"] for x in recent], [x["off"] for x in recent])
        m["outside_s"] = 0.1 * sum(1 for x in pairs if abs(x["off"]) > TOL)
        m["outside_trusted_s"] = 0.1 * sum(1 for x in tr if abs(x["off"]) > TOL)
        m["honesty"] = 100.0 * sum(1 for x in tr if abs(x["off"]) <= x["Uv"] + x["Ua"]) / len(tr) if tr else 100.0
        m["trusted_pct"] = 100.0 * len(tr) / len(pairs)
        m["offset_now_us"] = pairs[-1]["off"] / 1e3
    else:                                                # fallback: ALTAIR's own PTP view
        m["source"] = "ptp"
        h = list(status_hist)
        good = [x for x in h if x[2] in ("VALID", "PREDICTED")]
        corr = [x[5] for x in good]
        m["max_offset_us"] = max((abs(c) for c in corr), default=0) / 1e3
        m["max_offset_all_us"] = max((abs(x[5]) for x in h), default=0) / 1e3
        m["jitter_us"] = statistics.pstdev(corr) / 1e3 if len(corr) > 1 else 0.0
        recent = [x for x in good if h and x[0] > h[-1][0] - 30]
        m["residual_drift_ppb"] = slope_per_s([x[0] for x in recent], [x[5] for x in recent])
        m["outside_s"] = 0.5 * sum(1 for x in h if x[2] == "UNRELIABLE" or abs(x[5]) > TOL)
        m["outside_trusted_s"] = 0.5 * sum(1 for x in good if abs(x[5]) > TOL)
        m["honesty"] = 100.0 * sum(1 for x in good if abs(x[5]) <= x[3]) / len(good) if good else 100.0
        m["trusted_pct"] = 100.0 * len(good) / len(h) if h else 0.0
        m["offset_now_us"] = h[-1][5] / 1e3 if h else 0.0
    m["recovery_last_ms"] = recoveries[-1] if recoveries else None
    m["recovery_max_ms"] = max(recoveries) if recoveries else None
    m["recoveries"] = recoveries[-10:]
    m["order"] = dict(order)
    m["monotonic"] = {n: {"checked": v[1], "violations": v[2]} for n, v in mono.items()}
    return m


def wiring_hint():
    """Explains in one sentence why there are no pulse pairs yet, from what has arrived.
    Why: on the day, a loose wire must be found in seconds.
    """
    v, a, tr = seen["VEGA"]["C"], seen["ALTAIR"]["C"], seen["TR"]
    if pairs:
        return ""
    stuck = [n for n, f in frozen.items() if f[2] >= 5]
    if stuck:
        return ("%s's timestamps are FROZEN (an old stamp from an earlier run is held in /dev/shmem). On %s: "
                "Ctrl-C, then  rm -f /dev/shmem/starsync_keep_*  and start again." % (stuck[0], stuck[0]))
    if v and a:
        return ("Both boards see pulses but no pair matches within 50 ms: on both run  cat /dev/starsync/time  "
                "- the seconds must match and keep moving.")
    if not tr and not v and not a:
        return "Nothing from SKY yet: start run-sky.sh on SKY."
    if not v and not a:
        return ("SKY is running (%d ordering pairs sent) but neither VEGA nor ALTAIR sees its pulses: check the wires "
                "SKY pin 11 -> pin 13 on both, and GND to GND; test with ./skygen test on SKY." % tr)
    who = "VEGA" if not v else "ALTAIR"
    return "%s sees no SKY pulses (the other one does): check %s's wire to pin 13 and its GND." % (who, who)


def dashboard_data():
    """Everything the web dashboard shows, as one JSON-ready dict: metrics, node status,
    the offset and PTP series of the last 3 minutes, and the event log.
    Why: the browser polls /data once a second and draws it.
    """
    with lock:
        m = metrics()
        now = time.time()
        nodes = {}
        for n, w in status.items():
            nodes[n] = {"state": w[2], "validity": w[3], "U_us": float(w[4]) / 1e3,
                        "raw_offset_us": float(w[5]) / 1e3, "corr_offset_us": float(w[6]) / 1e3,
                        "drift_ppm": float(w[7]) / 1e3, "delay_us": float(w[8]) / 1e3,
                        "recovery_ms": float(w[9]), "detect_ms": float(w[10]), "restarts": int(w[11]),
                        "lat_avg_us": float(w[12]) if len(w) > 14 else None,      # QNX timer wake-up
                        "lat_max_us": float(w[13]) if len(w) > 14 else None,
                        "age_s": round(now - w[-1], 1)}
        t_end = now - t0
        off = [[round(x["t"], 2), round(x["off"] / 1e3, 2), round((x["Uv"] + x["Ua"]) / 1e3, 2), trusted(x)]
               for x in pairs if x["t"] > t_end - 180]
        ptp = [[round(x[0], 2), x[1], x[2], round(x[3] / 1e3, 2), round(x[5] / 1e3, 2), round(x[6] / 1e3, 3),
                round(x[7] / 1e3, 2)] for x in status_hist if x[0] > t_end - 180]
        return {"t": round(t_end, 1), "metrics": m, "nodes": nodes, "offset_series": off,
                "seen": seen, "hint": wiring_hint(),
                "ptp_series": ptp, "events": list(events)[-40:]}


# ------------------------------------------------------------------ commands to the boards
def send_command(target, cmd):
    """Sends a text command (e.g. 'drift 20') over UDP to VEGA, ALTAIR or both; returns the replies.
    Why: the dashboard's fault buttons and the command box use this.
    """
    addrs = {"vega": args.vega, "altair": args.altair}
    out = []
    for name in (["vega", "altair"] if target == "all" else [target]):
        host, port = addrs[name].split(":")
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.settimeout(1.0)
        s.sendto(cmd.encode(), (host, int(port)))
        try:
            out.append("%s: %s" % (name.upper(), s.recv(4096).decode().strip()))
        except socket.timeout:
            out.append("%s: no answer (is starsync running?)" % name.upper())
        s.close()
    log_event("command to %s: %s" % (target.upper(), cmd))
    return "\n".join(out)


def calibrate():
    """Median of the last 100 trusted true offsets; sends 'calib' so ALTAIR removes it.
    Why: removes the fixed cable / network asymmetry that PTP alone cannot see.
    """
    with lock:
        tr = [x["off"] for x in pairs[-100:] if trusted(x)]
    if len(tr) < 20:
        return "calibrate: need at least 20 trusted pulses (SKY running, ALTAIR LOCKED)"
    med = statistics.median(tr) / 1e3
    return "ALTAIR was %+.1f us off on average -> %s" % (med, send_command("altair", "calib %.2f" % med))


def report():
    """Prints the final referee report with every metric.
    Why: the summary to show the judges (key r, and automatically on quit).
    """
    with lock:
        m = metrics()
    o = m["order"]
    print("\n=================== StarSync referee report ===================")
    print("source                  %s" % ("SKY pulses (true offset)" if m["source"] == "pulses" else "ALTAIR's own PTP view (SKY pulses not arriving)"))
    print("max inter-node offset   %.1f us trusted, %.1f us all   (tolerance %.0f us)" % (m["max_offset_us"], m["max_offset_all_us"], args.tol))
    print("timestamp jitter        %.2f us" % m["jitter_us"])
    print("residual drift          %+.1f ppb  (last 30 s)" % m["residual_drift_ppb"])
    print("time outside tolerance  %.1f s  (trusted but outside: %.1f s  <- must be 0)" % (m["outside_s"], m["outside_trusted_s"]))
    print("honesty                 %.1f %%" % m["honesty"])
    print("event ordering          correct %d, WRONG %d, too close %d, unreliable %d, resolution %s"
          % (o["ok"], o["wrong"], o["close"], o["unreliable"], "%d us" % o["best_gap"] if o["best_gap"] else "-"))
    print("recovery times          %s ms" % (", ".join("%.0f" % r for r in m["recoveries"]) or "-"))
    print("monotonic               %s" % (", ".join("%s %d checked, %d backwards" % (n, v["checked"], v["violations"])
                                                    for n, v in m["monotonic"].items()) or "-"))
    print("================================================================\n")


# ------------------------------------------------------------------ the web dashboard
class Web(BaseHTTPRequestHandler):
    """The dashboard web server: the page, its live data and its command button."""
    def log_message(self, *a):                           # keep the terminal quiet
        """Silences the HTTP server's per-request log lines.
        Why: keeps the referee's terminal readable.
        """
        pass

    def reply(self, code, body, ctype):
        """Sends one HTTP response with the given status code, body and content type.
        Why: shared by every page and API answer.
        """
        data = body if isinstance(body, bytes) else body.encode()
        self.send_response(code)
        self.send_header("Content-Type", ctype)
        self.send_header("Cache-Control", "no-store")
        self.send_header("Content-Length", str(len(data)))
        self.end_headers()
        self.wfile.write(data)

    def do_GET(self):
        """Serves the dashboard page (/) and the live data (/data, JSON).
        Why: the dashboard is one static page that polls /data.
        """
        if self.path.startswith("/data"):
            self.reply(200, json.dumps(dashboard_data()), "application/json")
        elif self.path in ("/", "/index.html"):
            with open(os.path.join(HERE, "dashboard.html"), "rb") as f:
                self.reply(200, f.read(), "text/html; charset=utf-8")
        else:
            self.reply(404, "not found", "text/plain")

    def do_POST(self):
        """Runs a dashboard action (/cmd): calibrate, reset, or a command for a board.
        Why: lets the judges inject faults from the browser.
        """
        n = int(self.headers.get("Content-Length", 0))
        req = json.loads(self.rfile.read(n) or b"{}")
        cmd, target = req.get("cmd", "").strip(), req.get("target", "altair")
        if cmd == "calibrate":
            out = calibrate()
        elif cmd == "reset":
            with lock:
                clear_scores()
            out = "scores reset"
        elif cmd and target in ("vega", "altair", "all"):
            out = send_command(target, cmd)
        else:
            out = "unknown command"
        self.reply(200, json.dumps({"reply": out}), "application/json")


def keyboard():
    """Reads the terminal keys: r = report, c = calibrate, z = reset scores, q = quit.
    Why: control the referee without the browser.
    """
    for cmd in sys.stdin:
        c = cmd.strip().lower()
        if c.startswith("r"): report()
        elif c.startswith("c"): print(calibrate())
        elif c.startswith("z"):
            with lock:
                clear_scores()
            print("scores reset")
        elif c.startswith("q"):
            report(); csv.close(); os._exit(0)


# ------------------------------------------------------------------ main
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(("0.0.0.0", args.port))
sock.settimeout(0.2)
if args.web:
    web = ThreadingHTTPServer(("0.0.0.0", args.web), Web)
    threading.Thread(target=web.serve_forever, daemon=True).start()
    print("dashboard: open  http://localhost:%d  in a browser" % args.web)
print("referee: listening on UDP %d, tolerance +-%.0f us. Type r=report c=calibrate z=reset q=quit" % (args.port, args.tol))
threading.Thread(target=keyboard, daemon=True).start()
next_print = time.time() + 2
try:
    while True:
        try:
            data, _ = sock.recvfrom(4096)
            for ln in data.decode(errors="ignore").splitlines():
                handle(ln)
        except socket.timeout:
            pass
        if time.time() >= next_print:
            next_print = time.time() + 2
            with lock:
                m = metrics()
                a = status.get("ALTAIR")
            print("[%5.0fs] %s | offset %+7.1f us | max %.1f us | jitter %.1f us | drift %+.1f ppb | outside %.1f s (trusted %.1f) | %s"
                  % (time.time() - t0, "pulses %d" % m["pulses"] if m["source"] == "pulses" else
                     "PTP view (pulse edges: VEGA %d, ALTAIR %d; SKY truths %d)" % (seen["VEGA"]["C"], seen["ALTAIR"]["C"], seen["TR"]),
                     m["offset_now_us"], m["max_offset_us"], m["jitter_us"], m["residual_drift_ppb"],
                     m["outside_s"], m["outside_trusted_s"],
                     "ALTAIR %s %s U=%.1fus" % (a[2], a[3], float(a[4]) / 1e3) if a else "ALTAIR: no status"))
            if m["source"] != "pulses" and int(time.time() - t0) % 10 < 2:
                with lock:
                    hint = wiring_hint()
                if hint:
                    print("   HINT: " + hint)
            csv.flush()
except KeyboardInterrupt:
    report()
