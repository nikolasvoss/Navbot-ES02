#!/usr/bin/env python3
"""Open the CM5 HMMD browser view from a PC using an SSH tunnel."""

import argparse
import shutil
import socket
import subprocess
import sys
import time
import webbrowser


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--host", default="192.168.178.28", help="CM5 address")
    parser.add_argument("--user", default="niko", help="CM5 SSH user")
    args = parser.parse_args()
    if not shutil.which("ssh"):
        parser.error("OpenSSH (ssh) muss auf dem PC installiert sein.")
    url = f"http://{args.host}:8080/"
    try:
        with socket.socket() as probe:
            probe.bind(("127.0.0.1", 9090))
    except OSError:
        print("Port 9090 ist belegt. Beende zuerst den vorhandenen Tunnel.", file=sys.stderr)
        print(f"Falls der CM5-Tunnel bereits läuft, öffne direkt {url}")
        return 1

    command = [
        "ssh", "-N", "-o", "ExitOnForwardFailure=yes",
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
        print(f"Browser: {url}\nTerminal offen lassen. Strg+C beendet den Tunnel.", flush=True)
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
