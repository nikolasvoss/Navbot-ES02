#!/usr/bin/env python3
"""Start or reuse the HMMD sensor, rosbridge, and web server."""

import argparse
import errno
import re
import shlex
import shutil
import socket
import subprocess
import sys
import time
import webbrowser
from pathlib import Path


STARTUP_BASH = r'''#!/usr/bin/env bash
set -euo pipefail
HOST_WORKSPACE=${1:-"$HOME/Navbot-ES02-cm5-hmmd"}
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

SSH_OPTIONS = [
    "-o", "ConnectTimeout=10",
    "-o", "ServerAliveInterval=15",
    "-o", "ServerAliveCountMax=3",
]
SSH_DESTINATION = re.compile(r"(?:[A-Za-z0-9_][A-Za-z0-9_.-]*@)?[A-Za-z0-9_][A-Za-z0-9_.-]*\Z")


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--ssh", metavar="USER@HOST", help="SSH-Ziel; ohne diese Option lokal auf dem CM5 starten")
    parser.add_argument("--workspace", help="Projektpfad; lokal standardmäßig der Checkout, remote ~/Navbot-ES02-cm5-hmmd")
    parser.add_argument("--device", default="/dev/ttyAMA0", help="HMMD-UART auf dem Zielsystem")
    parser.add_argument("--baud-rate", type=int, default=115200, help="UART-Baudrate")
    parser.add_argument("--startup-timeout", type=int, default=30, help="Wartezeit pro Dienst in Sekunden")
    parser.add_argument("--no-browser", action="store_true", help="keinen Browser öffnen")
    args = parser.parse_args(argv)
    if args.ssh is not None and not SSH_DESTINATION.fullmatch(args.ssh):
        parser.error("SSH-Ziel muss ein Hostalias oder user@host ohne Shell-Sonderzeichen sein.")
    if args.baud_rate <= 0 or args.startup_timeout <= 0:
        parser.error("Baudrate und Start-Zeitlimit müssen positiv sein.")
    if "\0" in args.device or not args.device:
        parser.error("Gerätepfad darf nicht leer sein.")
    if args.workspace is not None and (not args.workspace or "\0" in args.workspace):
        parser.error("Workspace-Pfad ist ungültig.")
    return args


def workspace_for(args):
    if args.workspace is not None:
        return args.workspace
    if args.ssh:
        return ""
    return str(Path(__file__).resolve().parent.parent)


def startup_command(args):
    parameters = [workspace_for(args), args.device, str(args.baud_rate), str(args.startup_timeout)]
    if args.ssh:
        return ["ssh", *SSH_OPTIONS, args.ssh, shlex.join(["bash", "-s", "--", *parameters])]
    return ["bash", "-s", "--", *parameters]


def check_local_ports(ports=(8080, 9090)):
    for port in ports:
        try:
            with socket.socket() as probe:
                probe.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                probe.bind(("127.0.0.1", port))
        except OSError as error:
            if error.errno == errno.EADDRINUSE:
                print(f"Lokaler Port {port} ist belegt. Beende zuerst den vorhandenen Tunnel oder Dienst.", file=sys.stderr)
            else:
                print(f"Lokaler Port {port} kann nicht geöffnet werden: {error}", file=sys.stderr)
            return False
    return True


def run_startup(args):
    command = startup_command(args)
    if args.ssh:
        if not shutil.which("ssh"):
            print("OpenSSH (ssh) muss für --ssh installiert sein.", file=sys.stderr)
            return 1
        print("CM5-Dienste starten oder laufende Instanzen prüfen …", flush=True)
    else:
        print("HMMD-Dienste lokal starten oder laufende Instanzen prüfen …", flush=True)
    try:
        result = subprocess.run(
            command,
            input=STARTUP_BASH,
            text=True,
            timeout=args.startup_timeout * 5 + 30,
        )
    except KeyboardInterrupt:
        print("\nStart abgebrochen. Bereits gestartete Dienste bleiben aktiv.")
        return 130
    except subprocess.TimeoutExpired:
        print("Zeitüberschreitung beim Einrichten der HMMD-Dienste.", file=sys.stderr)
        return 1
    except OSError as error:
        print(f"Startprogramm kann nicht ausgeführt werden: {error}", file=sys.stderr)
        return 1
    if result.returncode:
        print("HMMD-Start fehlgeschlagen. Laufende Prozesse wurden nicht beendet.", file=sys.stderr)
        return result.returncode
    return 0


def tunnel_command(args):
    return [
        "ssh", "-N", "-o", "ConnectTimeout=10", "-o", "ExitOnForwardFailure=yes",
        "-o", "ServerAliveInterval=15", "-o", "ServerAliveCountMax=3",
        "-L", "127.0.0.1:8080:127.0.0.1:8080",
        "-L", "127.0.0.1:9090:127.0.0.1:9090",
        args.ssh,
    ]


def wait_for_tunnel(tunnel, ports=(8080, 9090), timeout=120):
    deadline = time.monotonic() + timeout
    ready = set()
    while len(ready) < len(ports):
        if tunnel.poll() is not None:
            return False
        for port in ports:
            if port in ready:
                continue
            try:
                with socket.create_connection(("127.0.0.1", port), timeout=0.2):
                    ready.add(port)
            except OSError:
                pass
        if len(ready) == len(ports):
            return True
        if time.monotonic() >= deadline:
            return False
        time.sleep(0.2)
    return True


def stop_tunnel(tunnel):
    if tunnel.poll() is None:
        tunnel.terminate()
        try:
            tunnel.wait(timeout=5)
        except subprocess.TimeoutExpired:
            tunnel.kill()
            tunnel.wait()


def run_tunnel(args):
    print("SSH-Tunnel starten; bei Bedarf SSH-Passwort im Terminal eingeben.", flush=True)
    tunnel = subprocess.Popen(tunnel_command(args))
    try:
        if not wait_for_tunnel(tunnel):
            print("SSH-Tunnel konnte nicht gestartet werden oder die Wartezeit ist abgelaufen.", file=sys.stderr)
            return tunnel.poll() or 1
        url = "http://127.0.0.1:8080/"
        print(f"Browser: {url}\nTerminal offen lassen. Strg+C beendet den Tunnel; CM5-Dienste bleiben aktiv.", flush=True)
        if not args.no_browser and not webbrowser.open(url):
            print("Bitte die URL manuell im Browser öffnen.", flush=True)
        return tunnel.wait()
    except KeyboardInterrupt:
        print("\nTunnel wird beendet.")
        return 0
    finally:
        stop_tunnel(tunnel)


def main(argv=None):
    args = parse_args(argv)
    if args.ssh and not check_local_ports():
        return 1
    result = run_startup(args)
    if result:
        return result
    url = "http://127.0.0.1:8080/"
    if not args.ssh:
        print(f"HMMD-Seite: {url}")
        if not args.no_browser and not webbrowser.open(url):
            print("Bitte die URL manuell im Browser öffnen.")
        return 0
    return run_tunnel(args)


if __name__ == "__main__":
    sys.exit(main())
