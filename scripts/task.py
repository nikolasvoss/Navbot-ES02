#!/usr/bin/env python3
"""task: start, sweep, drop and status of agent task worktrees.

A task is a branch with a worktree in MAIN/.claude/worktrees/<slug>/ and a task folder MAIN/work/<slug>/.
Paths, state and evidence come from devtools. Plan: docs/integration/plans/agent-workflow.md
Exit codes: 0 done, 1 refused (reason on stderr), 2 usage error.
"""

from __future__ import annotations

import argparse
import json
import os
import shutil
import subprocess
import sys
from collections.abc import Callable
from pathlib import Path

import devtools

SYSTEMD_DIR = Path("/etc/systemd/system")
WORKTREES = Path(".claude") / "worktrees"
TASKS = Path("work")
STATE_BODY = "\n## Contract\n\n## Plan\n\n## Decisions\n\n## Progress\n\n## Next step\n"


class Refused(Exception):
    """A command that cannot run now. The message goes to stderr and the exit code is 1."""


def _git(cwd: Path, *args: str) -> subprocess.CompletedProcess[str]:
    return subprocess.run(["git", *args], cwd=cwd, capture_output=True, text=True)


def _git_out(cwd: Path, *args: str) -> str:
    proc = _git(cwd, *args)
    if proc.returncode:
        raise Refused(proc.stderr.strip() or f"git {' '.join(args)} failed")
    return proc.stdout.strip()


def _has_branch(main: Path, branch: str) -> bool:
    return _git(main, "show-ref", "--verify", "--quiet", f"refs/heads/{branch}").returncode == 0


def _dirty(worktree: Path) -> bool:
    return bool(_git_out(worktree, "status", "--porcelain"))


def _worktrees(main: Path) -> list[tuple[Path, str | None]]:
    """Every worktree of the repository as (path, branch); the branch is None when HEAD is detached."""
    found: list[tuple[Path, str | None]] = []
    path: Path | None = None
    branch: str | None = None
    for line in [*_git_out(main, "worktree", "list", "--porcelain").splitlines(), ""]:
        if line.startswith("worktree "):
            path = Path(line.removeprefix("worktree "))
        elif line.startswith("branch refs/heads/"):
            branch = line.removeprefix("branch refs/heads/")
        elif not line and path is not None:
            found.append((path, branch))
            path, branch = None, None
    return found


def _merged(main: Path, branch: str) -> bool:
    return _git(main, "merge-base", "--is-ancestor", f"refs/heads/{branch}", "origin/main").returncode == 0


def _other_branches_containing(main: Path, branch: str) -> list[str]:
    names = _git_out(main, "for-each-ref", "--contains", f"refs/heads/{branch}", "--format=%(refname:short)", "refs/heads/")
    return [name for name in names.splitlines() if name != branch]


def process_inside(worktree: Path) -> bool:
    """True when a process has its working directory inside the worktree."""
    root = worktree.resolve()
    for cwd_link in devtools.PROC.glob("[0-9]*/cwd"):
        try:
            target = Path(os.readlink(cwd_link))
        except OSError:
            continue
        if target.is_relative_to(root):
            return True
    return False


def _unit_naming(worktree: Path, systemd_dir: Path) -> str | None:
    """The first navbot service unit whose file mentions the worktree path."""
    for unit in sorted(systemd_dir.glob("navbot-*.service")):
        if str(worktree) in unit.read_text(encoding="utf-8", errors="replace"):
            return unit.name
    return None


def _task_blocker(main: Path, worktree: Path, branch: str | None, systemd_dir: Path, busy: Callable[[Path], bool]) -> str | None:
    """The reason a task worktree must stay, or None when it may be removed."""
    if branch is None:
        return "detached HEAD"
    if not _merged(main, branch):
        return "not merged into origin/main"
    state = devtools.task_dir(branch, main) / "state.md"
    if not state.is_file():
        return "no task state"
    if _git_out(main, "rev-parse", f"refs/heads/{branch}") == devtools.read_state(state).baseline:
        return "no commits since the baseline"
    if _dirty(worktree):
        return "uncommitted changes"
    if busy(worktree):
        return "a process runs in the worktree"
    unit = _unit_naming(worktree, systemd_dir)
    if unit:
        return f"a systemd unit names the worktree: {unit}"
    return None


def _agent_blocker(main: Path, worktree: Path, branch: str | None, busy: Callable[[Path], bool]) -> str | None:
    """The reason a subagent worktree must stay, or None when it may be removed."""
    if branch is None:
        return "detached HEAD"
    # The branch's reflog has one entry until its first commit; Claude Code removes unchanged worktrees itself.
    if len(_git_out(main, "reflog", "show", "--format=%H", f"refs/heads/{branch}").splitlines()) <= 1:
        return "no commits yet"
    if busy(worktree):
        return "a process runs in the worktree"
    if _dirty(worktree):
        return "uncommitted changes"
    if not _other_branches_containing(main, branch):
        return "HEAD is on no other branch"
    return None


def _remove_tree(path: Path) -> None:
    if path.exists():
        shutil.rmtree(path)


def sweep(
    main: Path,
    *,
    dry_run: bool = False,
    systemd_dir: Path = SYSTEMD_DIR,
    busy: Callable[[Path], bool] = process_inside,
) -> list[str]:
    """Remove finished worktrees and task folders of deleted branches. Returns one line per item."""
    _git_out(main, "fetch", "origin", "main")
    verb = "would remove" if dry_run else "removed"
    lines: list[str] = []
    worktrees_root = (main / WORKTREES).resolve()
    for path, branch in _worktrees(main):
        if path.resolve().parent != worktrees_root:
            continue
        agent = path.name.startswith("agent-")
        if agent:
            reason = _agent_blocker(main, path, branch, busy)
        else:
            reason = _task_blocker(main, path, branch, systemd_dir, busy)
        if reason:
            lines.append(f"skipped {branch or path.name}: {reason}")
            continue
        lines.append(f"{verb} {branch}")
        if dry_run:
            continue
        _git_out(main, "worktree", "remove", str(path))
        _git_out(main, "branch", "-D", branch)
        if not agent:
            _remove_tree(devtools.task_dir(branch, main))
    tasks_root = main / TASKS
    for task in sorted(tasks_root.iterdir()) if tasks_root.is_dir() else []:
        if not (task / "state.md").is_file():
            continue
        branch = devtools.read_state(task / "state.md").branch
        if _has_branch(main, branch):
            continue
        lines.append(f"{verb} task folder {task.name}: branch {branch} no longer exists")
        if not dry_run:
            shutil.rmtree(task)
    return lines


def _started(branch: str, worktree: Path, task: Path) -> dict:
    return {"branch": branch, "worktree": str(worktree), "task_dir": str(task), "state": str(task / "state.md")}


def start(main: Path, branch: str, from_ref: str | None = None) -> dict:
    """Create or resume the worktree, task folder and state.md of a branch."""
    worktree = devtools.worktree_dir(branch, main)
    task = devtools.task_dir(branch, main)
    state_path = task / "state.md"
    if worktree.exists() and state_path.exists():
        return _started(branch, worktree, task)
    if _has_branch(main, branch):
        if from_ref is not None:
            raise Refused(f"branch {branch} exists; --from applies only to a new branch")
        baseline = _git_out(main, "merge-base", branch, "main")
        if not worktree.exists():
            _git_out(main, "worktree", "add", str(worktree), branch)
    else:
        baseline = _git_out(main, "rev-parse", "--verify", f"{from_ref or 'main'}^{{commit}}")
        _git_out(main, "worktree", "add", "-b", branch, str(worktree), baseline)
    if not state_path.exists():
        task.mkdir(parents=True, exist_ok=True)
        devtools.write_state(
            state_path,
            devtools.State(
                branch=branch,
                worktree=str(worktree),
                baseline=baseline,
                status="implementing",
                acceptance=[],
                body=STATE_BODY,
            ),
        )
    return _started(branch, worktree, task)


def drop(main: Path, branch: str) -> bool:
    """Remove the worktree (refused with uncommitted changes) and the task folder; keep the branch.

    Returns False when the branch had neither a worktree nor a task folder.
    """
    worktree = devtools.worktree_dir(branch, main)
    task = devtools.task_dir(branch, main)
    if not worktree.exists() and not task.exists():
        return False
    if worktree.exists():
        if _dirty(worktree):
            raise Refused(f"uncommitted changes in {worktree}; commit or discard them first")
        _git_out(main, "worktree", "remove", str(worktree))
    _remove_tree(task)
    return True


def status(cwd: Path, main: Path) -> dict:
    """The status of the task on the current branch, and the verify result recorded for HEAD."""
    branch = devtools.current_branch(cwd)
    if branch is None:
        raise Refused("HEAD is detached; check out a task branch")
    task = devtools.task_dir(branch, main)
    if not (task / "state.md").is_file():
        raise Refused(f"no task folder for {branch}; run task.py start {branch}")
    head = _git_out(cwd, "rev-parse", "HEAD")
    verify = (devtools.read_evidence(task, head) or {}).get("verify") or {}
    return {
        "branch": branch,
        "task_dir": str(task),
        "status": devtools.read_state(task / "state.md").status,
        "evidence": verify.get("result", "missing"),
    }


def main(
    argv: list[str] | None = None,
    *,
    cwd: Path | None = None,
    systemd_dir: Path = SYSTEMD_DIR,
    busy: Callable[[Path], bool] = process_inside,
) -> int:
    parser = argparse.ArgumentParser(description="Start, sweep, drop and inspect agent task worktrees.")
    sub = parser.add_subparsers(dest="command", required=True)
    start_cmd = sub.add_parser("start", help="create or resume a task's worktree and task folder")
    start_cmd.add_argument("branch")
    start_cmd.add_argument("--from", dest="from_ref", metavar="REF", help="base of a new branch (default: main)")
    sweep_cmd = sub.add_parser("sweep", help="remove finished worktrees and task folders of deleted branches")
    sweep_cmd.add_argument("--dry-run", action="store_true", help="print what would be removed")
    drop_cmd = sub.add_parser("drop", help="remove a task's worktree and task folder; keep the branch")
    drop_cmd.add_argument("branch")
    sub.add_parser("status", help="print the current task's status and its evidence for HEAD")
    args = parser.parse_args(argv)

    here = cwd or Path.cwd()
    main_dir = devtools.main_checkout(here)
    try:
        if args.command == "start":
            print(json.dumps(start(main_dir, args.branch, args.from_ref)))
        elif args.command == "sweep":
            for line in sweep(main_dir, dry_run=args.dry_run, systemd_dir=systemd_dir, busy=busy):
                print(line)
        elif args.command == "drop":
            removed = drop(main_dir, args.branch)
            print(f"dropped {args.branch}" if removed else f"nothing to drop for {args.branch}")
        else:
            print(json.dumps(status(here, main_dir)))
    except Refused as error:
        print(error, file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
