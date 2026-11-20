#!/bin/sh
# deploy.sh - on the laptop: copy the QNX programs to the three Pis
# (asks the qnxuser password twice per board).
#     source ~/qnx800/qnxsdp-env.sh ; make qnx ; sh scripts/deploy.sh
cd "$(dirname "$0")/.." || exit 1
[ -f build/qnx/starsync ] || { echo "build first:  source ~/qnx800/qnxsdp-env.sh ; make qnx"; exit 1; }
for ip in 172.16.6.76 172.16.6.77 172.16.6.78; do
    echo "---- $ip"
    ssh qnxuser@$ip "mkdir -p starsync" &&
    scp build/qnx/starsync build/qnx/skygen build/qnx/hog $(ls build/qnx/starsync-prof 2>/dev/null) \
        scripts/run-*.sh scripts/trace.sh qnxuser@$ip:starsync/
done
echo "done. On each Pi:  cd starsync ; su ; sh run-vega.sh | run-altair.sh | run-sky.sh"
