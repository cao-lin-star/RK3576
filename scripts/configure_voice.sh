#!/usr/bin/env bash
# Configure only the dedicated voice port. Never probe or send to other UARTs.
set -euo pipefail
if [ "$EUID" -ne 0 ]; then echo 'Run with sudo.'; exit 1; fi
port=${1:-}
control=${2:-0}
case "$port" in /dev/serial/by-id/*|/dev/serial/by-path/*) ;; *) echo 'Use the dedicated USB-TTL /dev/serial/by-id/ or by-path link.'; exit 1;; esac
case "$control" in 0|1) ;; *) echo 'Second argument: 0 playback only, 1 voice returns enabled.'; exit 1;; esac
test -c "$port" || { echo 'Serial device missing'; exit 1; }
config=/etc/footbath/mobile.env
test -f "$config"
cp -a "$config" "${config}.before_voice_$(date +%Y%m%d_%H%M%S)"
python3 - "$config" "$port" "$control" <<'PY'
import sys
from pathlib import Path
p=Path(sys.argv[1]); keys={'FOOTBATH_VOICE_ENABLED':'1','FOOTBATH_VOICE_CONTROL':sys.argv[3],'FOOTBATH_VOICE_PORT':sys.argv[2]}
lines=[line for line in p.read_text().splitlines() if line.split('=',1)[0].strip() not in keys]
p.write_text('\n'.join(lines+[k+'='+v for k,v in keys.items()])+'\n')
PY
printf 'Configured voice port: %s; voice movement control: %s\n' "$port" "$control"
echo 'End the current robot task before restarting footbath-mobile. This script does not restart it.'
