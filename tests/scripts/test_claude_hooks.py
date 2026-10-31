"""claude_hooks: the four Claude Code hooks, run as subprocesses in temporary repositories with a bare origin."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
HOOK = ROOT / "scripts" / "claude_hooks.py"
TASK = ROOT / "scripts" / "task.py"
SETTINGS = ROOT / ".claude" / "settings.json"
FLASH_MESSAGE = "Flash only with python3 scripts/deploy.py; it checks that the robot is safe first.\n"
WORKTREE_MESSAGE = "Create worktrees with python3 scripts/task.py start <branch>.\n"
STOP_MESSAGE = "Run python3 scripts/verify.py, or say why the task is blocked.\n"
ORPHAN_STATE = """---
branch: feat/gone
worktree: /nowhere
baseline: 0000000000000000000000000000000000000000
status: implementing
acceptance: []
---

## Contract
"""


def git(cwd: Path, *args: str) -> str:
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


def hook(name: str, payload: dict, cwd: Path) -> subprocess.CompletedProcess[str]:
    return subprocess.run(
        [sys.executable, str(HOOK), name], input=json.dumps(payload), cwd=cwd, capture_output=True, text=True
    )


def task_start(repo: Path, branch: str) -> None:
    subprocess.run([sys.executable, str(TASK), "start", branch], cwd=repo, check=True, capture_output=True)


def commit(cwd: Path, name: str) -> None:
    (cwd / name).write_text(name + "\n", encoding="utf-8")
    git(cwd, "add", name)
    git(cwd, "commit", "-q", "-m", f"add {name}")


@pytest.fixture
def repo(tmp_path):
    origin = tmp_path / "origin.git"
    main = tmp_path / "Navbot-ES02"
    git(tmp_path, "init", "-q", "--bare", "-b", "main", str(origin))
    git(tmp_path, "init", "-q", "-b", "main", str(main))
    git(main, "config", "user.name", "Test")
    git(main, "config", "user.email", "test@example.invalid")
    git(main, "config", "commit.gpgsign", "false")
    (main / ".gitignore").write_text("/work/\n/.claude/worktrees/\n", encoding="utf-8")
    git(main, "add", ".gitignore")
    git(main, "commit", "-q", "-m", "init")
    git(main, "remote", "add", "origin", str(origin))
    git(main, "push", "-q", "-u", "origin", "main")
    return main.resolve()


@pytest.fixture
def task_worktree(repo):
    task_start(repo, "feat/x")
    return repo / ".claude" / "worktrees" / "feat-x"


def test_settings_register_the_four_hooks():
    hooks = json.loads(SETTINGS.read_text(encoding="utf-8"))["hooks"]
    commands = {event: [h["command"] for entry in entries for h in entry["hooks"]] for event, entries in hooks.items()}
    assert commands == {
        "SessionStart": ['python3 "$CLAUDE_PROJECT_DIR"/scripts/claude_hooks.py session-start'],
        "WorktreeCreate": ['python3 "$CLAUDE_PROJECT_DIR"/scripts/claude_hooks.py worktree-create'],
        "PreToolUse": ['python3 "$CLAUDE_PROJECT_DIR"/scripts/claude_hooks.py pre-bash'],
        "Stop": ['python3 "$CLAUDE_PROJECT_DIR"/scripts/claude_hooks.py stop'],
    }


@pytest.mark.parametrize(
    "command",
    [
        "python3 scripts/upload_firmware.py /abs/build/flash --port /dev/ttyUSB0",
        "cd x && python3 scripts/upload_firmware.py build/flash",
        "esptool.py --port /dev/ttyUSB0 write_flash 0x0 firmware.bin",
        "arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32",
    ],
)
def test_pre_bash_blocks_direct_flashing(tmp_path, command):
    result = hook("pre-bash", {"tool_input": {"command": command}}, tmp_path)
    assert (result.returncode, result.stdout, result.stderr) == (2, "", FLASH_MESSAGE)


def test_pre_bash_blocks_worktree_add_after_a_separator(tmp_path):
    result = hook("pre-bash", {"tool_input": {"command": "git add x && git worktree add ../x -b y"}}, tmp_path)
    assert (result.returncode, result.stdout, result.stderr) == (2, "", WORKTREE_MESSAGE)


@pytest.mark.parametrize(
    "command",
    [
        "python3 scripts/deploy.py --json",
        "grep esptool docs/integration/plans/agent-workflow.md",
        "python3 -m pytest tests/scripts/test_upload_firmware.py -q",
        "git worktree list",
    ],
)
def test_pre_bash_allows_other_commands(tmp_path, command):
    result = hook("pre-bash", {"tool_input": {"command": command}}, tmp_path)
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_worktree_create_for_a_task_starts_it(repo):
    result = hook("worktree-create", {"name": "feat/x", "cwd": str(repo)}, repo)
    assert (result.returncode, result.stdout, result.stderr) == (0, f"{repo}/.claude/worktrees/feat-x\n", "")
    assert git(repo, "show-ref", "refs/heads/feat/x") != ""
    assert (repo / "work" / "feat-x" / "state.md").is_file()


def test_worktree_create_for_an_agent_makes_a_branch_without_a_task_folder(repo):
    result = hook("worktree-create", {"name": "agent-7", "cwd": str(repo)}, repo)
    assert (result.returncode, result.stdout, result.stderr) == (0, f"{repo}/.claude/worktrees/agent-7\n", "")
    assert git(repo, "show-ref", "refs/heads/worktree-agent-7") != ""
    assert not (repo / "work").exists()


def test_session_start_on_startup_prints_nothing_when_nothing_is_swept(repo):
    result = hook("session-start", {"source": "startup", "cwd": str(repo)}, repo)
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_session_start_on_startup_prints_only_removed_items(repo):
    task_start(repo, "feat/x")
    orphan = repo / "work" / "orphan"
    orphan.mkdir(parents=True)
    (orphan / "state.md").write_text(ORPHAN_STATE, encoding="utf-8")
    result = hook("session-start", {"source": "startup", "cwd": str(repo)}, repo)
    assert (result.returncode, result.stdout, result.stderr) == (
        0,
        "removed task folder orphan: branch feat/gone no longer exists\n",
        "",
    )
    assert not orphan.exists()


@pytest.mark.parametrize("source", ["compact", "resume"])
def test_session_start_after_compact_or_resume_prints_the_state(repo, task_worktree, source):
    state = (repo / "work" / "feat-x" / "state.md").read_text(encoding="utf-8")
    result = hook("session-start", {"source": source, "cwd": str(task_worktree)}, task_worktree)
    assert (result.returncode, result.stderr) == (0, "")
    assert result.stdout == f"Task state (work/feat-x/state.md):\n{state}"


def test_session_start_truncates_the_state_to_8000_characters(repo, task_worktree):
    state_path = repo / "work" / "feat-x" / "state.md"
    state_path.write_text(state_path.read_text(encoding="utf-8") + "x" * 9000, encoding="utf-8")
    text = state_path.read_text(encoding="utf-8")
    result = hook("session-start", {"source": "resume", "cwd": str(task_worktree)}, task_worktree)
    assert result.stdout == f"Task state (work/feat-x/state.md):\n{text[:8000]}"


def test_session_start_after_resume_prints_nothing_without_a_task(repo):
    result = hook("session-start", {"source": "resume", "cwd": str(repo)}, repo)
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_stop_blocks_an_implementing_task_with_commits_and_no_verify_result(task_worktree):
    commit(task_worktree, "a.txt")
    result = hook("stop", {"cwd": str(task_worktree), "stop_hook_active": False}, task_worktree)
    assert (result.returncode, result.stdout, result.stderr) == (2, "", STOP_MESSAGE)


def test_stop_never_blocks_when_stop_hook_is_already_active(task_worktree):
    commit(task_worktree, "a.txt")
    result = hook("stop", {"cwd": str(task_worktree), "stop_hook_active": True}, task_worktree)
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_stop_allows_a_task_whose_head_has_a_verify_result(task_worktree, repo):
    commit(task_worktree, "a.txt")
    head = git(task_worktree, "rev-parse", "HEAD")
    evidence = repo / "work" / "feat-x" / "evidence"
    evidence.mkdir(parents=True)
    (evidence / f"{head}.json").write_text(json.dumps({"verify": {"result": "done"}}), encoding="utf-8")
    result = hook("stop", {"cwd": str(task_worktree), "stop_hook_active": False}, task_worktree)
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_stop_allows_a_task_with_uncommitted_changes(task_worktree):
    commit(task_worktree, "a.txt")
    (task_worktree / "b.txt").write_text("draft\n", encoding="utf-8")
    result = hook("stop", {"cwd": str(task_worktree), "stop_hook_active": False}, task_worktree)
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_stop_allows_a_task_whose_head_is_still_the_baseline(task_worktree):
    result = hook("stop", {"cwd": str(task_worktree), "stop_hook_active": False}, task_worktree)
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_stop_allows_a_task_whose_status_is_not_implementing(task_worktree, repo):
    commit(task_worktree, "a.txt")
    state = repo / "work" / "feat-x" / "state.md"
    state.write_text(state.read_text(encoding="utf-8").replace("status: implementing", "status: done"), encoding="utf-8")
    result = hook("stop", {"cwd": str(task_worktree), "stop_hook_active": False}, task_worktree)
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")


def test_stop_ignores_the_main_checkout_without_a_task(repo):
    result = hook("stop", {"cwd": str(repo), "stop_hook_active": False}, repo)
    assert (result.returncode, result.stdout, result.stderr) == (0, "", "")
