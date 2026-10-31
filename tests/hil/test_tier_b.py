"""Tier B: checks that need the user at the robot. Plan: docs/integration/plans/agent-workflow.md, T11.

Run by the user only, through `python3 scripts/verify.py --guided`. Each test prints what the user must do with CH5 or
the robot, then polls the robot until it shows the result. The tests run in file order and each one starts from the
state the previous one left; the robot ends with CH5 at 0.
"""

import queue
import time

import pytest

from navbot_link.link import CommandRefused
from navbot_link.protocol import LinkError

pytestmark = [pytest.mark.guided, pytest.mark.robot]

GUIDED_TIMEOUT_S = 60
POLL_S = 0.2
BALANCE_HOLD_S = 2.0
COMMAND_SPEED_MPS = 0.1
COMMAND_PERIOD_S = 0.1
COMMAND_SECONDS = 2.0
# The demand must be zero this soon after the last Command: the robot's command timeout is 250 ms, and the rest is polling.
DEMAND_ZERO_S = 1.0


def instruct(text):
    """Print the instruction for the user at the robot."""
    print(f"\nGUIDED: {text}", flush=True)


def wait_for(link, name, expected, timeout_s=GUIDED_TIMEOUT_S):
    """Poll `name` until it equals `expected`; fail with the last value when the time runs out."""
    deadline = time.monotonic() + timeout_s
    while True:
        value = link.get(name)
        if value == expected:
            return
        if time.monotonic() > deadline:
            pytest.fail(f"{name} is {value} after {timeout_s} s, expected {expected}")
        time.sleep(POLL_S)


def take_events(link):
    """Every event queued so far, without waiting."""
    events = []
    while True:
        try:
            events.append(link.events.get_nowait())
        except queue.Empty:
            return events


# Position 2 starts balancing, so the robot must stand on the floor before CH5 reaches it.
FLOOR = "Put the robot on the floor and hold it upright: position 2 starts balancing. "


def move_ch5(position):
    instruct(f"{FLOOR if position == 2 else ''}Move CH5 to position {position}.")


def test_ch5_position_follows_switch(link):
    for position in (0, 1, 2):
        move_ch5(position)
        wait_for(link, "ch5_position", position)


def test_reboot_refused_while_ch5_requests(link):
    for position in (1, 2):
        move_ch5(position)
        wait_for(link, "ch5_position", position)
        with pytest.raises(LinkError, match=r"^call refused: CH5 must be 0 and balance off$"):
            link.call("reboot")


def test_balance_runs_at_ch5_2(link):
    instruct("Robot on the floor, released by hand. CH5 must be at position 2; move it there if it is not.")
    wait_for(link, "ch5_position", 2)
    wait_for(link, "balance_state", 2)


def test_balance_keeps_running_at_ch5_1(link):
    instruct("Move CH5 from 2 to 1. The robot must keep balancing.")
    wait_for(link, "ch5_position", 1)
    deadline = time.monotonic() + BALANCE_HOLD_S
    while time.monotonic() < deadline:
        state = link.get("balance_state")
        assert state == 2, f"balance_state is {state} after CH5 moved to 1, expected 2"
        time.sleep(POLL_S)


def test_link_balance_off_and_on(link):
    instruct("CH5 stays at 1. The link switches balance off, then on. Hold the robot: while balance is off the wheels are free.")
    wait_for(link, "ch5_position", 1)
    link.call("balance", on=0)
    wait_for(link, "balance_state", 0)
    link.call("balance", on=1)
    wait_for(link, "balance_state", 2)


def test_command_stops_after_timeout(link):
    instruct("CH5 at 1 and balance running. The robot drives at 0.1 m/s for 2 s; keep clear of the wheels.")
    wait_for(link, "ch5_position", 1)
    wait_for(link, "balance_state", 2)
    take_events(link)
    deadline = time.monotonic() + COMMAND_SECONDS
    while time.monotonic() < deadline:
        link.command(COMMAND_SPEED_MPS, 0.0)
        time.sleep(COMMAND_PERIOD_S)
    assert link.get("cmd_speed_mps") == pytest.approx(COMMAND_SPEED_MPS)
    instruct("Command stopped. The demand must return to zero by itself.")
    wait_for(link, "cmd_speed_mps", 0.0, timeout_s=DEMAND_ZERO_S)
    assert [e for e in take_events(link) if isinstance(e, CommandRefused)] == []


def test_ch5_moves_start_and_end_balance(link):
    instruct("Hold the robot. Move CH5 to position 0: balance ends.")
    wait_for(link, "ch5_position", 0)
    wait_for(link, "balance_state", 0)
    instruct("Move CH5 from 0 straight to 2 in one move, without stopping at 1: balance starts.")
    wait_for(link, "ch5_position", 2)
    wait_for(link, "balance_state", 2)
    instruct("Move CH5 from 2 straight to 0 in one move. Hold the robot: balance ends.")
    wait_for(link, "ch5_position", 0)
    wait_for(link, "balance_state", 0)
