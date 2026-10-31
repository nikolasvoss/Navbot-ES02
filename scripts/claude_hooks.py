#!/usr/bin/env python3
"""claude_hooks: Claude Code hook entry points for the agent workflow.

Each hook reads its JSON payload on stdin. Exit 2 blocks the action and sends the message on stderr.
SessionStart stdout is added to the agent's context. The hook table and settings: docs/integration/plans/agent-workflow.md, "Shared names".
"""

from __future__ import annotations

import argparse
import json
import re
import sys
from collections.abc import Callable
from pathlib import Path

import devtools
import task

STATE_CHARS = 8000
FLASH_PATTERN = re.compile(r"(^|[;&|]\s*)(\S*python3?\s+\S*upload_firmware\.py|\S*esptool\S*(\s|$)|arduino-cli\s+upload)")
WORKTREE_PATTERN = re.compile(r"(^|[;&|]\s*)git\s+worktree\s+add\b")
FLASH_MESSAGE = "Flash only with python3 scripts/deploy.py; it checks that the robot is safe first."
WORKTREE_MESSAGE = "Create worktrees with python3 scripts/task.py start <branch>."
STOP_MESSAGE = "Run python3 scripts/verify.py, or say why the task is blocked."
BLOCKED_COMMANDS = ((FLASH_PATTERN, FLASH_MESSAGE), (WORKTREE_PATTERN, WORKTREE_MESSAGE))


def _cwd(payload: dict) -> Path:
    return Path(payload.get("cwd") or Path.cwd())


def session_start(payload: dict) -> int:
    cwd = _cwd(payload)
    if payload.get("source") == "startup":
        try:
            lines = task.sweep(devtools.main_checkout(cwd))
        except task.Refused as error:
            print(error, file=sys.stderr)
            return 0
        # Active tasks are skipped at every start; only what was removed is news.
        removed = [line for line in lines if line.startswith("removed")]
        if removed:
            print("\n".join(removed))
        return 0
    branch = devtools.current_branch(cwd)
    if branch is None:
        return 0
    state = devtools.task_dir(branch, devtools.main_checkout(cwd)) / "state.md"
    if state.is_file():
        print(f"Task state (work/{devtools.slug(branch)}/state.md):")
        sys.stdout.write(state.read_text(encoding="utf-8")[:STATE_CHARS])
    return 0


def worktree_create(payload: dict) -> int:
    cwd = _cwd(payload)
    name = payload["name"]
    main = devtools.main_checkout(cwd)
    if name.startswith("agent-"):
        path = main / task.WORKTREES / name
        devtools.git(["worktree", "add", "-b", f"worktree-{name}", str(path), "HEAD"], cwd)
    else:
        path = Path(task.start(main, name)["worktree"])
    print(path)
    return 0


def pre_bash(payload: dict) -> int:
    command = payload.get("tool_input", {}).get("command", "")
    for pattern, message in BLOCKED_COMMANDS:
        if pattern.search(command):
            print(message, file=sys.stderr)
            return 2
    return 0


def stop(payload: dict) -> int:
    if payload.get("stop_hook_active"):
        return 0
    cwd = _cwd(payload)
    branch = devtools.current_branch(cwd)
    if branch is None:
        return 0
    task_dir = devtools.task_dir(branch, devtools.main_checkout(cwd))
    if not (task_dir / "state.md").is_file():
        return 0
    state = devtools.read_state(task_dir / "state.md")
    if state.status != "implementing":
        return 0
    head = devtools.git(["rev-parse", "HEAD"], cwd)
    if head == state.baseline:
        return 0
    if devtools.git(["status", "--porcelain"], cwd):
        return 0
    if (devtools.read_evidence(task_dir, head) or {}).get("verify"):
        return 0
    print(STOP_MESSAGE, file=sys.stderr)
    return 2


HOOKS: dict[str, Callable[[dict], int]] = {
    "session-start": session_start,
    "worktree-create": worktree_create,
    "pre-bash": pre_bash,
    "stop": stop,
}


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description="Claude Code hooks for the agent workflow; the JSON payload is read from stdin.")
    parser.add_argument("hook", choices=sorted(HOOKS))
    args = parser.parse_args(argv)
    text = sys.stdin.read()
    return HOOKS[args.hook](json.loads(text) if text.strip() else {})


if __name__ == "__main__":
    raise SystemExit(main())
