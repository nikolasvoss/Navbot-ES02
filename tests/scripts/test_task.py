"""task: start, sweep, drop and status in temporary repositories with a bare origin."""

import json
import subprocess
import sys
from pathlib import Path

import pytest

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / "scripts"))
import devtools  # noqa: E402
import task  # noqa: E402


def git(cwd: Path, *args: str) -> str:
    return subprocess.run(["git", *args], cwd=cwd, check=True, capture_output=True, text=True).stdout.strip()


def branch_exists(repo: Path, branch: str) -> bool:
    return subprocess.run(["git", "show-ref", "--verify", "--quiet", f"refs/heads/{branch}"], cwd=repo).returncode == 0


def commit(cwd: Path, name: str) -> str:
    (cwd / name).write_text(name + "\n", encoding="utf-8")
    git(cwd, "add", name)
    git(cwd, "commit", "-q", "-m", f"add {name}")
    return git(cwd, "rev-parse", "HEAD")


def nobody(path: Path) -> bool:
    return False


def run_task(capsys, args, cwd, systemd, busy=nobody):
    code = task.main(args, cwd=cwd, systemd_dir=systemd, busy=busy)
    captured = capsys.readouterr()
    return code, captured.out, captured.err


def begin(capsys, repo, systemd, branch):
    code, out, err = run_task(capsys, ["start", branch], repo, systemd)
    assert (code, err) == (0, "")
    return json.loads(out)


def merge_to_main(repo, branch):
    git(repo, "merge", "-q", "--no-edit", branch)
    git(repo, "push", "-q", "origin", "main")


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
    return main


@pytest.fixture
def systemd(tmp_path):
    path = tmp_path / "systemd"
    path.mkdir()
    return path


def test_start_creates_the_worktree_the_task_folder_and_the_state(repo, systemd, capsys):
    base = git(repo, "rev-parse", "main")
    code, out, err = run_task(capsys, ["start", "feat/one-task"], repo, systemd)
    worktree = repo / ".claude/worktrees/feat-one-task"
    task_dir = repo / "work/feat-one-task"
    assert (code, err) == (0, "")
    assert json.loads(out) == {
        "branch": "feat/one-task",
        "worktree": str(worktree),
        "task_dir": str(task_dir),
        "state": str(task_dir / "state.md"),
    }
    assert git(worktree, "rev-parse", "--abbrev-ref", "HEAD") == "feat/one-task"
    state = devtools.read_state(task_dir / "state.md")
    assert (state.branch, state.worktree, state.baseline, state.status, state.acceptance) == (
        "feat/one-task",
        str(worktree),
        base,
        "implementing",
        [],
    )


def test_start_reuses_an_existing_branch_with_its_merge_base_as_baseline(repo, systemd, capsys):
    base = git(repo, "rev-parse", "main")
    git(repo, "branch", "feat/old")
    commit(repo, "later.txt")
    code, _, err = run_task(capsys, ["start", "feat/old"], repo, systemd)
    assert (code, err) == (0, "")
    assert devtools.read_state(repo / "work/feat-old/state.md").baseline == base
    assert git(repo / ".claude/worktrees/feat-old", "rev-parse", "--abbrev-ref", "HEAD") == "feat/old"


def test_start_from_a_ref_sets_the_baseline_of_a_new_branch(repo, systemd, capsys):
    first = commit(repo, "first.txt")
    commit(repo, "second.txt")
    code, _, err = run_task(capsys, ["start", "feat/from", "--from", first], repo, systemd)
    assert (code, err) == (0, "")
    assert devtools.read_state(repo / "work/feat-from/state.md").baseline == first


def test_start_refuses_from_for_an_existing_branch(repo, systemd, capsys):
    git(repo, "branch", "feat/taken")
    code, out, err = run_task(capsys, ["start", "feat/taken", "--from", "main"], repo, systemd)
    assert (code, out, err) == (1, "", "branch feat/taken exists; --from applies only to a new branch\n")


def test_start_twice_prints_the_same_json_and_changes_nothing(repo, systemd, capsys):
    code1, out1, _ = run_task(capsys, ["start", "feat/twice"], repo, systemd)
    state = repo / "work/feat-twice/state.md"
    before = state.read_bytes()
    code2, out2, _ = run_task(capsys, ["start", "feat/twice"], repo, systemd)
    assert (code1, code2) == (0, 0)
    assert out2 == out1
    assert state.read_bytes() == before


def test_bad_usage_exits_with_2(repo, systemd):
    with pytest.raises(SystemExit) as info:
        task.main(["start"], cwd=repo, systemd_dir=systemd)
    assert info.value.code == 2


def test_sweep_removes_a_merged_clean_task(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/done")
    commit(repo / ".claude/worktrees/feat-done", "done.txt")
    merge_to_main(repo, "feat/done")
    code, out, err = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out, err) == (0, "removed feat/done\n", "")
    assert not (repo / ".claude/worktrees/feat-done").exists()
    assert not (repo / "work/feat-done").exists()
    assert not branch_exists(repo, "feat/done")


def test_sweep_dry_run_removes_nothing(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/done")
    commit(repo / ".claude/worktrees/feat-done", "done.txt")
    merge_to_main(repo, "feat/done")
    code, out, _ = run_task(capsys, ["sweep", "--dry-run"], repo, systemd)
    assert (code, out) == (0, "would remove feat/done\n")
    assert (repo / ".claude/worktrees/feat-done").exists()
    assert (repo / "work/feat-done").exists()
    assert branch_exists(repo, "feat/done")


def test_sweep_skips_an_unmerged_branch(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/open")
    commit(repo / ".claude/worktrees/feat-open", "open.txt")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "skipped feat/open: not merged into origin/main\n")
    assert (repo / ".claude/worktrees/feat-open").exists()


def test_sweep_skips_uncommitted_changes(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/dirty")
    worktree = repo / ".claude/worktrees/feat-dirty"
    commit(worktree, "dirty.txt")
    merge_to_main(repo, "feat/dirty")
    (worktree / "scratch.txt").write_text("not committed\n", encoding="utf-8")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "skipped feat/dirty: uncommitted changes\n")
    assert worktree.exists()


def test_sweep_skips_a_worktree_with_a_process_inside(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/busy")
    commit(repo / ".claude/worktrees/feat-busy", "busy.txt")
    merge_to_main(repo, "feat/busy")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd, busy=lambda path: path.name == "feat-busy")
    assert (code, out) == (0, "skipped feat/busy: a process runs in the worktree\n")
    assert (repo / ".claude/worktrees/feat-busy").exists()


def test_sweep_skips_a_worktree_named_by_a_systemd_unit(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/unit")
    worktree = repo / ".claude/worktrees/feat-unit"
    commit(worktree, "unit.txt")
    merge_to_main(repo, "feat/unit")
    (systemd / "navbot-bridge.service").write_text(f"[Service]\nExecStart=python3 {worktree}/scripts/bridge.py\n")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "skipped feat/unit: a systemd unit names the worktree: navbot-bridge.service\n")
    assert worktree.exists()


def test_sweep_ignores_a_unit_that_does_not_name_the_worktree(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/other")
    commit(repo / ".claude/worktrees/feat-other", "other.txt")
    merge_to_main(repo, "feat/other")
    (systemd / "navbot-bridge.service").write_text("[Service]\nExecStart=python3 /elsewhere/bridge.py\n")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "removed feat/other\n")


def test_sweep_removes_an_agent_worktree_whose_head_is_on_another_branch(repo, systemd, capsys):
    agent = repo / ".claude/worktrees/agent-review"
    git(repo, "worktree", "add", "-q", "-b", "worktree-agent-review", str(agent), "HEAD")
    commit(agent, "review.txt")
    git(repo, "branch", "lead", "worktree-agent-review")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "removed worktree-agent-review\n")
    assert not agent.exists()
    assert not branch_exists(repo, "worktree-agent-review")


def test_sweep_keeps_an_agent_worktree_whose_head_is_only_on_it(repo, systemd, capsys):
    agent = repo / ".claude/worktrees/agent-solo"
    git(repo, "worktree", "add", "-q", "-b", "worktree-agent-solo", str(agent), "HEAD")
    commit(agent, "solo.txt")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "skipped worktree-agent-solo: HEAD is on no other branch\n")
    assert agent.exists()


def test_sweep_keeps_a_dirty_agent_worktree(repo, systemd, capsys):
    agent = repo / ".claude/worktrees/agent-dirty"
    git(repo, "worktree", "add", "-q", "-b", "worktree-agent-dirty", str(agent), "HEAD")
    commit(agent, "dirty.txt")
    git(repo, "branch", "lead", "worktree-agent-dirty")
    (agent / "scratch.txt").write_text("not committed\n", encoding="utf-8")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "skipped worktree-agent-dirty: uncommitted changes\n")
    assert agent.exists()


def test_sweep_keeps_a_new_agent_worktree_without_commits(repo, systemd, capsys):
    agent = repo / ".claude/worktrees/agent-fresh"
    git(repo, "worktree", "add", "-q", "-b", "worktree-agent-fresh", str(agent), "HEAD")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "skipped worktree-agent-fresh: no commits yet\n")
    assert agent.exists()


def test_sweep_keeps_a_new_task_without_commits(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/fresh")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "skipped feat/fresh: no commits since the baseline\n")
    assert (repo / ".claude/worktrees/feat-fresh").exists()
    assert (repo / "work/feat-fresh/state.md").is_file()


def test_sweep_skips_a_task_folder_with_an_unreadable_state_and_keeps_sweeping(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/done")
    commit(repo / ".claude/worktrees/feat-done", "done.txt")
    merge_to_main(repo, "feat/done")
    broken = repo / "work/feat-broken"
    broken.mkdir(parents=True)
    (broken / "state.md").write_text("not a state file\n", encoding="utf-8")
    code, out, err = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out, err) == (0, "removed feat/done\nskipped feat-broken: unreadable state.md\n", "")
    assert broken.exists()
    assert not (repo / "work/feat-done").exists()


def test_start_refuses_a_branch_whose_slug_belongs_to_another_branch(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/x")
    code, out, err = run_task(capsys, ["start", "feat-x"], repo, systemd)
    assert (code, out, err) == (
        1,
        "",
        "work/feat-x belongs to feat/x; feat-x has the same slug\n",
    )
    assert not branch_exists(repo, "feat-x")


def test_drop_refuses_a_branch_whose_slug_belongs_to_another_branch(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/x")
    worktree = repo / ".claude/worktrees/feat-x"
    code, out, err = run_task(capsys, ["drop", "feat-x"], repo, systemd)
    assert (code, out, err) == (1, "", "work/feat-x belongs to feat/x; feat-x has the same slug\n")
    assert worktree.exists()
    assert (repo / "work/feat-x").exists()


def test_drop_refuses_a_worktree_with_a_process_inside(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/busy-drop")
    worktree = repo / ".claude/worktrees/feat-busy-drop"
    code, out, err = run_task(
        capsys, ["drop", "feat/busy-drop"], repo, systemd, busy=lambda path: path.name == "feat-busy-drop"
    )
    assert (code, out, err) == (1, "", "cannot drop feat/busy-drop: a process runs in the worktree\n")
    assert worktree.exists()
    assert (repo / "work/feat-busy-drop").exists()


def test_drop_refuses_a_worktree_named_by_a_systemd_unit(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/unit-drop")
    worktree = repo / ".claude/worktrees/feat-unit-drop"
    (systemd / "navbot-bridge.service").write_text(f"[Service]\nExecStart=python3 {worktree}/scripts/bridge.py\n")
    code, out, err = run_task(capsys, ["drop", "feat/unit-drop"], repo, systemd)
    assert (code, out, err) == (
        1,
        "",
        "cannot drop feat/unit-drop: a systemd unit names the worktree: navbot-bridge.service\n",
    )
    assert worktree.exists()


def test_sweep_removes_task_folders_whose_branch_is_gone_and_keeps_other_folders(repo, systemd, capsys):
    orphan = repo / "work/feat-gone"
    orphan.mkdir(parents=True)
    devtools.write_state(
        orphan / "state.md",
        devtools.State(
            branch="feat/gone",
            worktree=str(repo / ".claude/worktrees/feat-gone"),
            baseline=git(repo, "rev-parse", "main"),
            status="implementing",
            acceptance=[],
            body="",
        ),
    )
    notes = repo / "work/notes"
    notes.mkdir()
    (notes / "capture.txt").write_text("kept\n", encoding="utf-8")
    code, out, _ = run_task(capsys, ["sweep"], repo, systemd)
    assert (code, out) == (0, "removed task folder feat-gone: branch feat/gone no longer exists\n")
    assert not orphan.exists()
    assert (notes / "capture.txt").exists()


def test_process_inside_sees_the_working_directory_of_this_process(tmp_path, monkeypatch):
    worktree = tmp_path / "worktree"
    worktree.mkdir()
    elsewhere = tmp_path / "elsewhere"
    elsewhere.mkdir()
    monkeypatch.chdir(worktree)
    assert task.process_inside(worktree) is True
    monkeypatch.chdir(elsewhere)
    assert task.process_inside(worktree) is False


def test_drop_refuses_uncommitted_changes_then_removes_the_worktree_and_the_folder(repo, systemd, capsys):
    begin(capsys, repo, systemd, "feat/drop")
    worktree = repo / ".claude/worktrees/feat-drop"
    task_dir = repo / "work/feat-drop"
    (worktree / "scratch.txt").write_text("not committed\n", encoding="utf-8")
    code, out, err = run_task(capsys, ["drop", "feat/drop"], repo, systemd)
    assert (code, out, err) == (1, "", f"uncommitted changes in {worktree}; commit or discard them first\n")
    assert worktree.exists() and task_dir.exists()

    (worktree / "scratch.txt").unlink()
    code, out, err = run_task(capsys, ["drop", "feat/drop"], repo, systemd)
    assert (code, out, err) == (0, "dropped feat/drop\n", "")
    assert not worktree.exists()
    assert not task_dir.exists()
    assert branch_exists(repo, "feat/drop")

    code, out, err = run_task(capsys, ["drop", "feat/drop"], repo, systemd)
    assert (code, out, err) == (0, "nothing to drop for feat/drop\n", "")


@pytest.mark.parametrize(
    "recorded, expected",
    [("done", "done"), ("failed", "failed"), ("blocked", "blocked"), (None, "missing")],
)
def test_status_reports_the_verify_result_recorded_for_head(repo, systemd, capsys, recorded, expected):
    begin(capsys, repo, systemd, "feat/status")
    worktree = repo / ".claude/worktrees/feat-status"
    task_dir = repo / "work/feat-status"
    if recorded is not None:
        devtools.update_evidence(
            task_dir,
            git(worktree, "rev-parse", "HEAD"),
            "feat/status",
            verify={"result": recorded, "at": "2026-11-02T10:22:00+01:00"},
        )
    code, out, err = run_task(capsys, ["status"], worktree, systemd)
    assert (code, err) == (0, "")
    assert json.loads(out) == {
        "branch": "feat/status",
        "task_dir": str(task_dir),
        "status": "implementing",
        "evidence": expected,
    }
