#!/usr/bin/env python3
"""Start/reuse the CM5 HMMD services and open the browser view through SSH."""

import argparse
import errno
import shlex
import shutil
import socket
import subprocess
import sys
import time
import webbrowser


REMOTE_STARTUP = r'''#!/usr/bin/env bash
set -euo pipefail
HOST_WORKSPACE=$1
DEVICE=$2
BAUD_RATE=$3
STARTUP_TIMEOUT=$4
WEB_ROOT="$HOST_WORKSPACE/src/cm5/ros2/src/hmmd_radar/web"
SENSOR_LOG=/tmp/navbot-hmmd-sensor.log
BRIDGE_LOG=/tmp/navbot-hmmd-rosbridge.log
WEB_LOG=/tmp/navbot-hmmd-web.log
STATUS_FILE=$(mktemp /tmp/navbot-hmmd-status.XXXXXX)
trap 'rm -f "$STATUS_FILE"' EXIT

for tool in flock fuser ss python3 curl timeout; do
  command -v "$tool" >/dev/null || { echo "Fehlt auf dem CM5: $tool" >&2; exit 1; }
done
[[ -r /opt/ros/jazzy/setup.bash ]] || { echo "ROS-Setup fehlt: /opt/ros/jazzy/setup.bash" >&2; exit 1; }
[[ -r "$HOST_WORKSPACE/src/cm5/ros2/install/setup.bash" ]] || { echo "Workspace-Setup fehlt: $HOST_WORKSPACE/src/cm5/ros2/install/setup.bash" >&2; exit 1; }
[[ -d "$WEB_ROOT" ]] || { echo "HMMD-Webverzeichnis fehlt: $WEB_ROOT" >&2; exit 1; }
set +u
source /opt/ros/jazzy/setup.bash
source "$HOST_WORKSPACE/src/cm5/ros2/install/setup.bash"
set -u
command -v ros2 >/dev/null || { echo "Fehlt auf dem CM5 nach ROS-Setup: ros2" >&2; exit 1; }

exec 9>/tmp/navbot-hmmd-startup.lock
flock -w "$STARTUP_TIMEOUT" 9 || { echo "Ein anderer HMMD-Start hält die Startsperre" >&2; exit 1; }

sensor_processes() {
  python3 - "$DEVICE" "$BAUD_RATE" <<'__SENSOR_PY__'
import os
import sys
from pathlib import Path

device, baud_rate = sys.argv[1:]
for process in Path("/proc").iterdir():
    if not process.name.isdigit():
        continue
    try:
        args = [os.fsdecode(arg) for arg in (process / "cmdline").read_bytes().split(b"\0") if arg]
    except (FileNotFoundError, PermissionError, ProcessLookupError):
        continue
    executable = args[:2]
    if not any(os.path.basename(arg) == "hmmd_sensor" for arg in executable):
        continue
    if f"port:={device}" not in args or f"baud_rate:={baud_rate}" not in args:
        print(f"HMMD-Sensor PID {process.name} läuft mit anderer UART-/Baud-Konfiguration", file=sys.stderr)
        raise SystemExit(1)
    print(process.name)
__SENSOR_PY__
}
sensor_pids() { fuser "$DEVICE" 2>/dev/null || true; }
verify_sensor_owners() {
  local pid
  for pid in $(sensor_pids); do
    [[ "$pid" == "$sensor_pid" ]] || { echo "UART $DEVICE ist durch anderen Prozess $pid belegt" >&2; return 1; }
  done
}

sensor_pid=$(sensor_processes)
[[ $(wc -w <<<"$sensor_pid") -le 1 ]] || { echo "Mehrere HMMD-Sensorprozesse laufen: $sensor_pid" >&2; exit 1; }
verify_sensor_owners
sensor_state=reused
if [[ -z "$sensor_pid" ]]; then
  [[ -c "$DEVICE" && -r "$DEVICE" && -w "$DEVICE" ]] || { echo "HMMD-Gerät ist nicht lesbar/schreibbar: $DEVICE" >&2; exit 1; }
  sensor_state=started
  nohup ros2 run hmmd_radar hmmd_sensor --ros-args -p "port:=$DEVICE" -p "baud_rate:=$BAUD_RATE" >"$SENSOR_LOG" 2>&1 </dev/null 9>&- &
  deadline=$((SECONDS + STARTUP_TIMEOUT))
  while [[ -z "$sensor_pid" ]]; do
    sensor_pid=$(sensor_processes)
    if (( SECONDS >= deadline )); then
      echo "HMMD-Sensor startete nicht; Log: $SENSOR_LOG" >&2
      tail -n 20 "$SENSOR_LOG" >&2 || true
      exit 1
    fi
    sleep 1
  done
fi
[[ $(wc -w <<<"$sensor_pid") -eq 1 ]] || { echo "Mehrere HMMD-Sensorprozesse laufen: $sensor_pid" >&2; exit 1; }
verify_sensor_owners

status_ready() {
  if ! timeout 5 ros2 topic echo --once /hmmd/status diagnostic_msgs/msg/DiagnosticArray >"$STATUS_FILE" 2>&1; then return 1; fi
  python3 - "$STATUS_FILE" "$DEVICE" <<'__STATUS_PY__'
import sys
import yaml

status_path, device = sys.argv[1:]
try:
    with open(status_path, encoding="utf-8") as status:
        message = next(yaml.safe_load_all(status), None)
except yaml.YAMLError:
    raise SystemExit(1)
entries = message.get("status", []) if isinstance(message, dict) else []
ready = any(isinstance(entry, dict) and entry.get("name") == "hmmd_sensor"
            and entry.get("hardware_id") == device for entry in entries)
raise SystemExit(0 if ready else 1)
__STATUS_PY__
}
deadline=$((SECONDS + STARTUP_TIMEOUT))
while ! status_ready; do
  kill -0 "$sensor_pid" 2>/dev/null || { echo "HMMD-Sensor wurde beendet; Log: $SENSOR_LOG" >&2; tail -n 20 "$SENSOR_LOG" >&2 || true; exit 1; }
  verify_sensor_owners || { echo "Statusprüfung fehlgeschlagen; Sensorlog: $SENSOR_LOG" >&2; tail -n 20 "$SENSOR_LOG" >&2 2>/dev/null || true; exit 1; }
  if (( SECONDS >= deadline )); then
    echo "Kein frischer /hmmd/status für hmmd_sensor auf $DEVICE; letzter Status:" >&2
    cat "$STATUS_FILE" >&2
    echo "Sensorlog: $SENSOR_LOG" >&2
    tail -n 20 "$SENSOR_LOG" >&2 2>/dev/null || true
    exit 1
  fi
  sleep 1
done
kill -0 "$sensor_pid" 2>/dev/null || { echo "HMMD-Sensor wurde beendet" >&2; exit 1; }
verify_sensor_owners

listener_rows() { ss -H -ltnp "sport = :$1"; }
listener_pids() { sed -n 's/.*pid=\([0-9][0-9]*\).*/\1/p' <<<"$1" | sort -u; }
proc_args() { tr '\0' ' ' <"/proc/$1/cmdline" 2>/dev/null || true; }

bridge_state=reused
bridge_rows=$(listener_rows 9090)
if [[ -z "$bridge_rows" ]]; then
  bridge_state=started
  nohup ros2 launch rosbridge_server rosbridge_websocket_launch.xml \
    address:=127.0.0.1 port:=9090 \
    'topics_glob:=""' 'topics_pub_glob:="[]"' \
    'topics_sub_glob:="[/hmmd/rdmap,/hmmd/status]"' \
    'services_glob:="[]"' 'params_glob:="[]"' >"$BRIDGE_LOG" 2>&1 </dev/null 9>&- &
  deadline=$((SECONDS + STARTUP_TIMEOUT))
  while [[ -z $(listener_rows 9090) ]]; do
    if (( SECONDS >= deadline )); then echo "rosbridge startete nicht; Log: $BRIDGE_LOG" >&2; tail -n 20 "$BRIDGE_LOG" >&2 || true; exit 1; fi
    sleep 1
  done
  bridge_rows=$(listener_rows 9090)
fi
if [[ $(wc -l <<<"$bridge_rows") -ne 1 || "$bridge_rows" != *"127.0.0.1:9090"* ]]; then
  echo "Port 9090 hat einen unerwarteten Listener: $bridge_rows" >&2; exit 1
fi
bridge_pids=$(listener_pids "$bridge_rows")
[[ $(wc -w <<<"$bridge_pids") -eq 1 ]] || { echo "rosbridge-Prozess auf Port 9090 nicht eindeutig prüfbar: $bridge_rows" >&2; exit 1; }
bridge_args=$(proc_args "$bridge_pids")
[[ "$bridge_args" == *rosbridge_websocket* ]] || { echo "Port 9090 gehört keinem rosbridge-WebSocket-Prozess (PID $bridge_pids): $bridge_args" >&2; exit 1; }
bridge_launch_args=
bridge_parent=$bridge_pids
for _ in 1 2 3 4 5; do
  bridge_parent=$(awk '/^PPid:/ {print $2}' "/proc/$bridge_parent/status" 2>/dev/null || true)
  [[ -n "$bridge_parent" && "$bridge_parent" -gt 1 ]] || break
  bridge_launch_args=$(proc_args "$bridge_parent")
  if [[ "$bridge_launch_args" == *rosbridge_websocket_launch.xml* ]]; then break; fi
done
[[ "$bridge_launch_args" == *rosbridge_websocket_launch.xml* ]] || { echo "rosbridge-Launch-Prozess in der Elternkette von PID $bridge_pids nicht gefunden" >&2; exit 1; }
for expected in address:=127.0.0.1 \
  'topics_glob:=""' 'topics_pub_glob:="[]"' \
  'topics_sub_glob:="[/hmmd/rdmap,/hmmd/status]"' \
  'services_glob:="[]"' 'params_glob:="[]"'; do
  [[ "$bridge_launch_args" == *"$expected"* ]] || { echo "rosbridge-Elternprozess hat eine unerwartete HMMD-Konfiguration (PID $bridge_parent): $bridge_launch_args" >&2; exit 1; }
done
timeout 3 bash -c '</dev/tcp/127.0.0.1/9090' || { echo "rosbridge auf 127.0.0.1:9090 antwortet nicht; Log: $BRIDGE_LOG" >&2; exit 1; }

web_state=reused
web_rows=$(listener_rows 8080)
if [[ -z "$web_rows" ]]; then
  web_state=started
  nohup python3 -m http.server 8080 --bind 0.0.0.0 --directory "$WEB_ROOT" >"$WEB_LOG" 2>&1 </dev/null 9>&- &
  deadline=$((SECONDS + STARTUP_TIMEOUT))
  while [[ -z $(listener_rows 8080) ]]; do
    if (( SECONDS >= deadline )); then echo "HTTP-Server startete nicht; Log: $WEB_LOG" >&2; tail -n 20 "$WEB_LOG" >&2 || true; exit 1; fi
    sleep 1
  done
  web_rows=$(listener_rows 8080)
fi
if [[ $(wc -l <<<"$web_rows") -ne 1 || "$web_rows" != *"0.0.0.0:8080"* ]]; then
  echo "Port 8080 hat einen unerwarteten Listener: $web_rows" >&2; exit 1
fi
web_pids=$(listener_pids "$web_rows")
[[ $(wc -w <<<"$web_pids") -eq 1 ]] || { echo "HTTP-Prozess auf Port 8080 nicht eindeutig prüfbar: $web_rows" >&2; exit 1; }
python3 - "$web_pids" "$WEB_ROOT" <<'__WEB_CHECK_PY__' || { echo "Port 8080 gehört nicht zum erwarteten HMMD-Webserver (PID $web_pids)" >&2; exit 1; }
import os
import sys
pid, expected_root = sys.argv[1:]
with open(f"/proc/{pid}/cmdline", "rb") as cmdline:
    args = [part.decode() for part in cmdline.read().split(b"\0") if part]
cwd = os.path.realpath(os.readlink(f"/proc/{pid}/cwd"))
try:
    module = args.index("-m")
    expected_args = ["-m", "http.server", "8080", "--bind", "0.0.0.0", "--directory"]
    if args[module:module + len(expected_args)] != expected_args:
        raise SystemExit(1)
    served_root = os.path.realpath(os.path.join(cwd, args[module + len(expected_args)]))
except (ValueError, IndexError):
    raise SystemExit(1)
raise SystemExit(0 if served_root == os.path.realpath(expected_root) else 1)
__WEB_CHECK_PY__
page=$(curl --fail --silent --show-error --max-time 3 http://127.0.0.1:8080/ 2>/dev/null) || { echo "HMMD-Webseite antwortet nicht; Log: $WEB_LOG" >&2; exit 1; }
[[ "$page" == *"<title>HMMD radar data</title>"* ]] || { echo "Port 8080 liefert nicht die HMMD-Webseite" >&2; exit 1; }
printf 'HMMD bereit: Sensor %s, rosbridge %s, Web %s\nLogs für neu gestartete Dienste: %s %s %s\n' "$sensor_state" "$bridge_state" "$web_state" "$SENSOR_LOG" "$BRIDGE_LOG" "$WEB_LOG"
'''


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.178.28", help="CM5-Adresse")
    parser.add_argument("--user", default="niko", help="SSH-Benutzer auf dem CM5")
    parser.add_argument("--workspace", default="/home/niko/Navbot-ES02-cm5-hmmd", help="Projektpfad auf dem CM5")
    parser.add_argument("--device", default="/dev/ttyAMA0", help="HMMD-UART auf dem CM5")
    parser.add_argument("--baud-rate", type=int, default=115200, help="UART-Baudrate")
    parser.add_argument("--startup-timeout", type=int, default=30, help="Wartezeit pro Dienst in Sekunden")
    args = parser.parse_args()
    if not shutil.which("ssh"):
        parser.error("OpenSSH (ssh) muss auf dem PC installiert sein.")
    if args.baud_rate <= 0 or args.startup_timeout <= 0:
        parser.error("Baudrate und Start-Zeitlimit müssen positiv sein.")
    url = f"http://{args.host}:8080/"
    try:
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 9090))
    except OSError as error:
        if error.errno != errno.EADDRINUSE:
            print(f"Lokaler Port 9090 kann nicht geöffnet werden: {error}", file=sys.stderr)
            return 1
        print("Port 9090 ist belegt. Beende zuerst den vorhandenen Tunnel.", file=sys.stderr)
        print(f"Falls der CM5-Tunnel bereits läuft, öffne direkt {url}")
        return 1

    remote_args = [args.workspace, args.device, str(args.baud_rate), str(args.startup_timeout)]
    remote_command = shlex.join(["bash", "-s", "--", *remote_args])
    bootstrap = [
        "ssh", "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
        f"{args.user}@{args.host}", remote_command,
    ]
    print("CM5-Dienste starten oder laufende Instanzen prüfen …", flush=True)
    try:
        result = subprocess.run(bootstrap, input=REMOTE_STARTUP, text=True,
                                timeout=args.startup_timeout * 5 + 30)
    except KeyboardInterrupt:
        print("\nCM5-Start abgebrochen. Bereits gestartete Dienste bleiben aktiv.")
        return 0
    except subprocess.TimeoutExpired:
        print("Zeitüberschreitung beim Einrichten der CM5-Dienste.", file=sys.stderr)
        return 1
    if result.returncode:
        print("CM5-Start fehlgeschlagen. Laufende Prozesse wurden nicht beendet.", file=sys.stderr)
        return result.returncode

    command = [
        "ssh", "-N", "-o", "ConnectTimeout=10", "-o", "ExitOnForwardFailure=yes",
        "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
        "-L", "127.0.0.1:9090:127.0.0.1:9090",
        f"{args.user}@{args.host}",
    ]
    print("SSH-Tunnel starten; bei Bedarf SSH-Passwort im Terminal eingeben.", flush=True)
    tunnel = subprocess.Popen(command)
    try:
        deadline = time.monotonic() + 120
        while True:
            if tunnel.poll() is not None:
                print("SSH-Tunnel konnte nicht gestartet werden.", file=sys.stderr)
                return tunnel.returncode or 1
            try:
                with socket.create_connection(("127.0.0.1", 9090), timeout=0.2):
                    break
            except OSError:
                if time.monotonic() >= deadline:
                    print("Zeitüberschreitung beim Start des SSH-Tunnels.", file=sys.stderr)
                    return 1
                time.sleep(0.2)
        print(f"Browser: {url}\nTerminal offen lassen. Strg+C beendet den Tunnel; CM5-Dienste bleiben aktiv.", flush=True)
        if not webbrowser.open(url):
            print("Bitte die URL manuell im Browser öffnen.", flush=True)
        return tunnel.wait()
    except KeyboardInterrupt:
        print("\nTunnel wird beendet.")
        return 0
    finally:
        if tunnel.poll() is None:
            tunnel.terminate()
            try:
                tunnel.wait(timeout=5)
            except subprocess.TimeoutExpired:
                tunnel.kill()
                tunnel.wait()


if __name__ == "__main__":
    sys.exit(main())
