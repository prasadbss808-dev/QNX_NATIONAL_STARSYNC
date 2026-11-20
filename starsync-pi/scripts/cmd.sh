#!/bin/sh
# cmd.sh - laptop: send a command to VEGA / ALTAIR and print the reply.
#   sh scripts/cmd.sh altair status          sh scripts/cmd.sh all blackout 15
#   sh scripts/cmd.sh 127.0.0.1:31913 drift 20          (any IP:PORT)
# On a Pi itself:  cat /dev/starsync/status   and   echo "drift 20" > /dev/starsync/ctl
[ $# -ge 2 ] || { echo "usage: cmd.sh vega|altair|all|IP:PORT COMMAND..."; exit 1; }
t=$1; shift
case $t in
  vega)   targets="172.16.6.76:31903" ;;
  altair) targets="172.16.6.77:31903" ;;
  all)    targets="172.16.6.76:31903 172.16.6.77:31903" ;;
  *)      targets="$t" ;;
esac
for a in $targets; do echo "$*" | nc -u -w1 "${a%:*}" "${a#*:}"; done
