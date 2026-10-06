#!/usr/bin/env python3
import argparse
import json
import re
from pathlib import Path

ROS_NAME = re.compile(r"/[A-Za-z][A-Za-z0-9_]*(?:/[A-Za-z][A-Za-z0-9_]*)*\Z")
ROS_TYPE = re.compile(r"[A-Za-z][A-Za-z0-9_]*/(?:msg|srv)/[A-Za-z][A-Za-z0-9_]*\Z")


def validate_manifest(manifest):
    if not isinstance(manifest, dict) or not isinstance(manifest.get("topics"), list) or not isinstance(manifest.get("services"), list):
        raise ValueError("manifest must contain topic and service arrays")
    names = set()
    subscriptions, publications, services = [], [], []
    for kind, entries in (("topic", manifest["topics"]), ("service", manifest["services"])):
        for entry in entries:
            if not isinstance(entry, dict):
                raise ValueError(f"invalid {kind} declaration")
            name, endpoint_type = entry.get("name"), entry.get("type")
            if not isinstance(name, str) or not ROS_NAME.fullmatch(name) or not isinstance(endpoint_type, str) or not ROS_TYPE.fullmatch(endpoint_type):
                raise ValueError(f"invalid ROS name or type in {kind} declaration")
            if (kind == "topic" and "/msg/" not in endpoint_type) or (kind == "service" and "/srv/" not in endpoint_type):
                raise ValueError(f"wrong ROS type kind for {name}")
            label = entry.get("label")
            if not isinstance(label, str) or not label.strip():
                raise ValueError(f"invalid label for {name}")
            if "defaultPayload" in entry and (not isinstance(entry["defaultPayload"], dict)):
                raise ValueError(f"invalid default payload for {name}")
            if name in names:
                raise ValueError(f"duplicate endpoint name: {name}")
            names.add(name)
            if kind == "service":
                timeout = entry.get("timeoutMs")
                if not isinstance(timeout, int) or isinstance(timeout, bool) or timeout < 1:
                    raise ValueError(f"invalid service timeout for {name}")
                services.append(name)
                continue
            direction = entry.get("direction")
            if direction not in ("subscribe", "publish", "both"):
                raise ValueError(f"invalid topic direction for {name}")
            if entry.get("reliability") not in ("reliable", "best_effort"):
                raise ValueError(f"invalid topic reliability for {name}")
            for key, minimum in (("throttleMs", 0), ("staleAfterMs", 1)):
                value = entry.get(key)
                if not isinstance(value, int) or isinstance(value, bool) or value < minimum:
                    raise ValueError(f"invalid {key} for {name}")
            if not isinstance(entry.get("pinned"), bool) or (direction == "publish" and entry["pinned"]):
                raise ValueError(f"invalid pinned setting for {name}")
            if direction in ("subscribe", "both"):
                subscriptions.append(name)
            if direction in ("publish", "both"):
                publications.append(name)
    return {
        "topics_glob": "[]",
        "topics_pub_glob": "[" + ",".join(publications) + "]",
        "topics_sub_glob": "[" + ",".join(subscriptions) + "]",
        "services_glob": "[" + ",".join(services) + "]",
        "params_glob": "[]",
    }


def bridge_arguments(manifest):
    filters = validate_manifest(manifest)
    return [f'{key}:="{value}"' for key, value in filters.items()]


def main(argv=None):
    parser = argparse.ArgumentParser(description="Validate endpoint-manifest.json and print bounded rosbridge filter arguments.")
    parser.add_argument("manifest", type=Path)
    parser.add_argument("--format", choices=("json", "shell"), default="json")
    args = parser.parse_args(argv)
    try:
        manifest = json.loads(args.manifest.read_text(encoding="utf-8"))
        values = bridge_arguments(manifest)
    except (OSError, json.JSONDecodeError, ValueError) as error:
        parser.error(str(error))
    if args.format == "shell":
        print("\n".join(values))
    else:
        print(json.dumps(values))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
