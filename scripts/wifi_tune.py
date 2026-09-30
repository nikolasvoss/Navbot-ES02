#!/usr/bin/env python3
"""Local WLAN parameter terminal for Navbot-ES02."""
from __future__ import annotations

import argparse
import http.client
import ipaddress
import json
import os
from pathlib import Path
import re
import secrets
import sys
import tempfile
from typing import Any

API = "/api/v1"
NAMES = {"PP", "PI", "PD", "PL", "SP", "SI", "SD", "SL", "YP", "YI", "YD", "YL", "RP", "RI", "RD", "RL", "U", "V"}
GAIN_NAMES = NAMES - {"U", "V"}
RANGES = {
    "PP": (0, 20), "PI": (0, 500), "PD": (0, 2), "PL": (0, 1),
    "SP": (0, 2), "SI": (0, 5), "SD": (0, 2), "SL": (0, 100),
    "YP": (0, 200), "YI": (0, 100), "YD": (0, 5), "YL": (0, 100),
    "RP": (0, 10), "RI": (0, 100), "RD": (0, 5), "RL": (0, 100),
    "U": (0, 1), "V": (0, 0.4),
}
LINE_RE = re.compile(r"^\s*([A-Z]{1,2})\s*([+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?)?\s*$")

class LocalError(Exception):
    pass
class TransportError(Exception):
    pass
class DeviceError(Exception):
    pass

class Client:
    def __init__(self, host: str, timeout: float = 2.0):
        try:
            parsed = ipaddress.ip_address(host)
        except ValueError as exc:
            raise LocalError("--host muss eine IPv4-Adresse im Heimnetz sein") from exc
        if not isinstance(parsed, ipaddress.IPv4Address):
            raise LocalError("Nur lokale IPv4-Verbindungen werden unterstützt")
        self.host, self.timeout = str(parsed), timeout
        self.boot_id: str | None = None

    def request(self, method: str, endpoint: str, payload: dict[str, Any] | None = None) -> dict[str, Any]:
        body = None if payload is None else json.dumps(payload, allow_nan=False, separators=(",", ":"))
        headers = {"Accept": "application/json"}
        if body is not None:
            headers["Content-Type"] = "application/json"
        conn = http.client.HTTPConnection(self.host, 80, timeout=self.timeout)
        try:
            conn.request(method, API + endpoint, body=body, headers=headers)
            response = conn.getresponse()
            raw = response.read(4096)
            try:
                data = json.loads(raw)
            except (UnicodeDecodeError, json.JSONDecodeError) as exc:
                raise TransportError("Ungültige JSON-Antwort vom Gerät") from exc
            if not isinstance(data, dict):
                raise TransportError("Ungültige Antwortstruktur vom Gerät")
            if data.get("error") == "OUTCOME_UNKNOWN":
                raise TransportError("HTTP 504: Ausgang des Schreibens unbekannt; bitte show/status lesen, nicht automatisch wiederholen")
            if response.status >= 400 or data.get("ok") is False:
                raise DeviceError(f"HTTP {response.status}: {data.get('error', 'Gerät hat Anfrage abgelehnt')}")
            if response.status != 200:
                raise TransportError(f"Unerwarteter HTTP-Status {response.status}")
            boot_id = data.get("boot_id")
            if isinstance(boot_id, str):
                self.boot_id = boot_id
            return data
        except (OSError, TimeoutError, http.client.HTTPException) as exc:
            if isinstance(exc, ConnectionRefusedError):
                message = (
                    f"Verbindung zu {self.host}:80 abgelehnt. Prüfe die Roboter-IP "
                    "und ob die WLAN-Tuning-API in der Firmware aktiviert ist."
                )
            elif isinstance(exc, TimeoutError):
                if method == "POST":
                    message = (
                        f"Keine Antwort von {self.host}:80 innerhalb von {self.timeout:g} s. "
                        "Ausgang des Schreibzugriffs unklar; zuerst status/show lesen und "
                        "nicht automatisch wiederholen."
                    )
                else:
                    message = f"Keine Antwort von {self.host}:80 innerhalb von {self.timeout:g} s"
            else:
                message = f"Transportfehler zu {self.host}:80: {exc}"
            raise TransportError(message) from exc
        finally:
            conn.close()

    def status(self) -> dict[str, Any]:
        return self.request("GET", "/status")
    def parameters(self) -> dict[str, Any]:
        return self.request("GET", "/parameters")
    def write(self, values: dict[str, float]) -> dict[str, Any]:
        if self.boot_id is None:
            status = self.status()
            self.boot_id = status.get("boot_id")
        if not isinstance(self.boot_id, str):
            raise TransportError("Gerät liefert keine Boot-ID")
        return self.request("POST", "/parameters", {
            "request_id": secrets.token_hex(16),
            "expected_boot_id": self.boot_id,
            "values": values,
        })


def json_no_duplicates(pairs: list[tuple[str, Any]]) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Doppelter JSON-Schlüssel: {key}")
        result[key] = value
    return result

def load_config(path: Path, host_arg: str | None) -> str:
    try:
        stat = path.stat()
        if stat.st_mode & 0o077:
            raise LocalError(f"Konfigurationsdatei muss Rechte 0600 haben: {path}")
        if stat.st_size > 8192:
            raise LocalError("Konfigurationsdatei ist zu groß")
        data = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=json_no_duplicates)
    except FileNotFoundError as exc:
        raise LocalError(f"Konfiguration fehlt: {path}; starte einmal mit --host ROBOTER-IP") from exc
    except (OSError, ValueError) as exc:
        raise LocalError(f"Konfiguration kann nicht gelesen werden: {exc}") from exc
    if not isinstance(data, dict) or not isinstance(data.get("host"), str):
        raise LocalError("Konfiguration benötigt das Feld host")
    return host_arg or data["host"]


def save_config(path: Path, host: str) -> None:
    parent = path.parent
    fd = -1
    temp_name: str | None = None
    try:
        parent.mkdir(mode=0o700, parents=True, exist_ok=True)
        encoded = json.dumps({"host": host}, indent=2) + "\n"
        fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=parent)
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            fd = -1
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temp_name, path)
    except OSError as exc:
        raise LocalError(f"Verbindungskonfiguration konnte nicht gespeichert werden: {exc}") from exc
    finally:
        if fd >= 0:
            os.close(fd)
        if temp_name and os.path.exists(temp_name):
            try:
                os.unlink(temp_name)
            except OSError:
                pass


def parse_terminal_line(line: str) -> tuple[str, float | None] | None:
    if len(line) > 128:
        raise LocalError("Eingabe ist zu lang")
    match = LINE_RE.fullmatch(line)
    if not match:
        raise LocalError("Erwartet wird ein erlaubter Parameter, optional mit Zahl (z. B. PP5 oder PD0.12)")
    name, number = match.groups()
    if name not in NAMES:
        raise LocalError(f"Parameter {name!r} ist nicht freigegeben")
    if number is None:
        return name, None
    try:
        value = float(number)
    except ValueError as exc:
        raise LocalError("Ungültige Zahl") from exc
    if not (float("-inf") < value < float("inf")):
        raise LocalError("Zahl muss endlich sein")
    low, high = RANGES[name]
    if not low <= value <= high:
        raise LocalError(f"{name} liegt außerhalb des Eingabebereichs {low}..{high}")
    if name == "U" and value not in (0.0, 1.0):
        raise LocalError("U akzeptiert nur 0 oder 1")
    return name, value


def validate_profile(data: Any) -> dict[str, float]:
    if not isinstance(data, dict) or set(data) - {"version", "comment", "values"}:
        raise LocalError("Profil enthält unbekannte Felder")
    if isinstance(data.get("version"), bool) or data.get("version") != 1 or not isinstance(data.get("values"), dict):
        raise LocalError("Ungültige Profilversion oder fehlende values")
    values = data["values"]
    if not values or len(values) > len(NAMES) or set(values) - NAMES:
        raise LocalError("Profil enthält keine Werte, zu viele Werte oder unbekannte Parameter")
    if "comment" in data and not isinstance(data["comment"], str):
        raise LocalError("comment muss Text sein")
    for name, value in values.items():
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not (-float("inf") < float(value) < float("inf")):
            raise LocalError(f"Ungültiger Zahlenwert für {name}")
        lo, hi = RANGES[name]
        if not lo <= float(value) <= hi:
            raise LocalError(f"{name} liegt außerhalb des Eingabebereichs {lo}..{hi}")
        if name == "U" and value not in (0, 1):
            raise LocalError("U akzeptiert nur 0 oder 1")
    if GAIN_NAMES.intersection(values) and values.get("U") != 1:
        raise LocalError("Ein Profil mit manuellen Gains muss U=1 enthalten")
    return {name: float(value) for name, value in values.items()}


def save_profile(path: Path, client: Client, force: bool, comment: str = "") -> dict[str, Any]:
    if path.exists() and not force:
        raise LocalError(f"Datei existiert bereits; zum Überschreiben save --force {path}")
    parent = path.parent
    if not parent.is_dir():
        raise LocalError(f"Verzeichnis existiert nicht: {parent}")
    snapshot = client.parameters()
    values = snapshot.get("values")
    if not isinstance(values, dict):
        raise TransportError("Snapshot enthält keine Werte")
    if values.get("U") == 1:
        selected = {name: values[name] for name in sorted(NAMES) if name in values}
    else:
        selected = {name: values[name] for name in ("U", "V") if name in values}
    profile = {"version": 1, "comment": comment, "values": selected}
    encoded = json.dumps(profile, ensure_ascii=False, indent=2, allow_nan=False) + "\n"
    fd = None
    temp_name = None
    try:
        fd, temp_name = tempfile.mkstemp(prefix=f".{path.name}.", dir=parent)
        os.fchmod(fd, 0o600)
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            fd = None
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        if path.exists() and not force:
            raise LocalError(f"Datei existiert bereits; zum Überschreiben save --force {path}")
        os.replace(temp_name, path)
        temp_name = None
    except OSError as exc:
        raise LocalError(f"Profil konnte nicht gespeichert werden: {exc}") from exc
    finally:
        if fd is not None:
            os.close(fd)
        if temp_name:
            try: os.unlink(temp_name)
            except OSError: pass
    return profile


def load_profile(path: Path, client: Client) -> dict[str, Any]:
    try:
        if path.stat().st_size > 16384:
            raise LocalError("Profil ist zu groß (maximal 16 KiB)")
        data = json.loads(path.read_text(encoding="utf-8"), object_pairs_hook=json_no_duplicates)
    except (OSError, ValueError) as exc:
        raise LocalError(f"Profil kann nicht gelesen werden: {exc}") from exc
    values = validate_profile(data)
    return client.write(values)


def format_data(data: dict[str, Any], json_output: bool = False) -> str:
    if json_output:
        return json.dumps(data, ensure_ascii=False, separators=(",", ":"))
    if "values" in data:
        return "\n".join(f"{k}={v}" for k, v in data["values"].items()) + "\nWerte gelten bis zum Neustart."
    fields = ("api_version", "boot_id", "build_id", "supported_mode", "rc_valid", "rc_age_ms", "ch5_off", "tuning_mode")
    return " ".join(f"{key}={data.get(key)}" for key in fields)


def execute(text: str, client: Client, json_output: bool = False) -> str | None:
    stripped = text.strip()
    if not stripped:
        return None
    lower = stripped.lower()
    if lower in ("quit", "exit"):
        raise EOFError
    if lower == "help":
        return "PP/PI/PD/PL, SP/SI/SD/SL, YP/YI/YD/YL, RP/RI/RD/RL, U, V; Name liest, Name+Zahl schreibt; status, show, save [--force] DATEI, load DATEI, quit"
    if lower == "status":
        return format_data(client.status(), json_output)
    if lower == "show":
        return format_data(client.parameters(), json_output)
    parts = stripped.split()
    if parts[0].lower() == "save":
        force = "--force" in parts[1:]
        args = [part for part in parts[1:] if part != "--force"]
        if len(args) != 1:
            raise LocalError("Verwendung: save [--force] DATEI")
        profile = save_profile(Path(args[0]), client, force)
        return f"Profil gespeichert: {args[0]} ({len(profile['values'])} Werte)"
    if parts[0].lower() == "load":
        if len(parts) != 2:
            raise LocalError("Verwendung: load DATEI")
        return format_data(load_profile(Path(parts[1]), client), json_output)
    parsed = parse_terminal_line(stripped)
    if parsed is None:
        return None
    name, value = parsed
    if value is None:
        data = client.parameters()
        values = data.get("values", {})
        if name not in values:
            raise TransportError("Parameter fehlt im Gerätesnapshot")
        return json.dumps({name: values[name]}, separators=(",", ":")) if json_output else f"{name}={values[name]}"
    return format_data(client.write({name: value}), json_output)


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(
        description="WLAN-Terminal für Reglerparameter des Navbot-ES02.",
        epilog=(
            "Terminalbeispiele: PP liest den aktuellen Wert, PP5 schreibt PP=5; "
            "show liest alle Parameter, status zeigt Verbindungs- und Freigabestatus. "
            "Gain-Schreibzugriffe brauchen U=1, frische SBUS-Daten und CH5 OFF. "
            "Parameteränderungen gelten bis zum Neustart."
        ),
    )
    parser.add_argument("--host", help="Roboter-IPv4; überschreibt host aus --config")
    parser.add_argument("--config", type=Path, default=Path.home() / ".config/navbot/wifi-tuning.json")
    parser.add_argument("--command", help="Einzelnen Terminalbefehl ausführen")
    parser.add_argument("--json", action="store_true", help="Antwort maschinenlesbar als JSON ausgeben")
    parser.add_argument("--timeout", type=float, default=2.0)
    args = parser.parse_args(argv)
    try:
        if args.config.exists():
            host = load_config(args.config, args.host)
        elif args.host:
            host = args.host
        else:
            host = input("IPv4-Adresse des Roboters: ").strip()
        client = Client(host, args.timeout)
        if not args.config.exists():
            save_config(args.config, client.host)
            print(f"Roboteradresse lokal gespeichert: {args.config} (Verbindung noch nicht geprüft)")
        if args.command is not None:
            try:
                result = execute(args.command, client, args.json)
            except EOFError:
                return 0
            print(result or "")
            return 0
        while True:
            try:
                line = input("navbot> ")
            except EOFError:
                print()
                return 0
            try:
                result = execute(line, client, args.json)
                if result is not None:
                    print(result)
            except EOFError:
                return 0
            except LocalError as exc:
                print(f"Eingabefehler: {exc}", file=sys.stderr)
            except DeviceError as exc:
                print(f"Geräteablehnung: {exc}", file=sys.stderr)
            except TransportError as exc:
                print(f"Transportfehler: {exc}", file=sys.stderr)
    except LocalError as exc:
        print(f"Eingabefehler: {exc}", file=sys.stderr)
        return 2
    except DeviceError as exc:
        print(f"Geräteablehnung: {exc}", file=sys.stderr)
        return 4
    except TransportError as exc:
        print(f"Transportfehler: {exc}", file=sys.stderr)
        return 3
    except KeyboardInterrupt:
        print()
        return 0

if __name__ == "__main__":
    raise SystemExit(main())
