"""Closed-loop scenarios of the host simulation: the balance path on the modelled robot (plan, T6).

Each test writes a scenario, runs it in batch mode and asserts on the CSV rows with the limits of the plan.
The robot runs with placeholder physical parameters; a failing limit is reported, not loosened.
"""

from conftest import run_sim

ARM = "0 rc 5 192\n0.5 rc 5 1792\n"  # CH5 off, then on: the balance arms
STATE_RUN = 2.0
STATE_FAULT = 4.0
FAULT_FALL = 6.0
FAULT_INVALID_ENVELOPE = 8.0  # bus voltage outside busMinV..busMaxV: the low-battery path


def seconds(row):
    return row["t_us"] / 1e6


def from_time(rows, start_s):
    return [row for row in rows if seconds(row) >= start_s]


def first_time(rows, predicate, start_s=0.0):
    """Time in seconds of the first row at or after start_s where predicate holds, or None."""
    for row in from_time(rows, start_s):
        if predicate(row):
            return seconds(row)
    return None


def test_stand(sim_binary):
    # The IMU reads 1.1 deg forward lean at balance, so the robot drifts back while the speed integral learns that lean
    # (about 0.17 m in the first 10 s); after that it holds its place.
    rows, _ = run_sim(sim_binary, ARM + "20.5 end\n")
    assert all(row["balance_state"] == STATE_RUN for row in from_time(rows, 1.0))
    assert all(abs(row["pitch_rad"]) < 0.035 for row in from_time(rows, 2.0))
    held = from_time(rows, 12.0)
    assert max(row["x_m"] for row in held) - min(row["x_m"] for row in held) < 0.05
    assert all(row["fallen"] == 0.0 for row in rows)


# The robot's drive run of 2026-11-01 (lab/robot/captures/2026-11-01-sim-validation-drive.csv): 0.2 m/s for 3 s, then stop.
# Measured: cruise 0.239 m/s; after the stop it travels 0.104 m, rolls back 0.075 m at up to 0.067 m/s, and is below
# 0.02 m/s 3.2 s after the stop. The model matches the cruise speed; the stop transient within a factor of two
# (docs/robot/software/simulation.md, Validation).
ROBOT_CRUISE_MPS = 0.239
ROBOT_STOP_TRAVEL_M = 0.104
ROBOT_ROLL_BACK_M = 0.075
ROBOT_BACK_SPEED_MPS = 0.067


def test_drive_step_rolls_back_like_the_robot(sim_binary):
    # 20 s standing first, so the speed integral has learned the standing lean, as on the robot.
    rows, _ = run_sim(sim_binary, ARM + "20 rc 3 1525\n23 rc 3 992\n30 end\n")
    cruise = [row["v_mps"] for row in rows if 21.25 <= seconds(row) < 23.0]
    assert abs(sum(cruise) / len(cruise) - ROBOT_CRUISE_MPS) <= 0.1 * ROBOT_CRUISE_MPS
    after = [row for row in rows if seconds(row) >= 23.0]
    x_stop = after[0]["x_m"]
    x_peak = max(row["x_m"] for row in after)
    assert 0.5 * ROBOT_STOP_TRAVEL_M <= x_peak - x_stop <= 2.0 * ROBOT_STOP_TRAVEL_M
    assert 0.5 * ROBOT_ROLL_BACK_M <= x_peak - after[-1]["x_m"] <= 2.0 * ROBOT_ROLL_BACK_M
    assert 0.5 * ROBOT_BACK_SPEED_MPS <= -min(row["v_mps"] for row in after) <= 2.0 * ROBOT_BACK_SPEED_MPS
    assert all(abs(row["v_mps"]) < 0.02 for row in after if seconds(row) >= 28.0)
    assert all(row["balance_state"] == STATE_RUN for row in from_time(rows, 1.0))


def test_turn(sim_binary):
    rows, _ = run_sim(sim_binary, ARM + "2 rc 4 192\n5 rc 4 992\n6 end\n")
    window = [row["yaw_rate_rad_s"] for row in rows if 3.5 <= seconds(row) <= 5.0]
    mean_yaw_rate = sum(window) / len(window)
    # The robot turned at 79 % of the commanded rate on 2026-11-01; here CH4 commands the 1 rad/s cap (floor scrub); within 15 % of that.
    assert abs(mean_yaw_rate - 0.79) <= 0.15 * 0.79
    assert all(abs(row["pitch_rad"]) < 0.09 for row in rows)


def test_push_recovers(sim_binary):
    rows, _ = run_sim(sim_binary, ARM + "3 push 0.05\n6 end\n")
    assert all(row["balance_state"] == STATE_RUN for row in from_time(rows, 1.0))
    assert all(abs(row["pitch_rad"]) < 0.035 for row in from_time(rows, 5.0))


def test_fall_faults(sim_binary):
    rows, _ = run_sim(sim_binary, ARM + "3 push 1.0\n4 end\n")
    t_exceeded = first_time(rows, lambda row: abs(row["pitch_rad"]) > 0.611)
    assert t_exceeded is not None
    # The Fall fault or the overspeed guard, whichever acts first, must switch both drivers off. In the simulation the
    # spinning wheels trip the guard 11 ms after the crossing, which leaves the controller Disabled (balance-controller.md).
    t_faulted = first_time(
        rows,
        lambda row: (row["balance_fault"] == FAULT_FALL or row["overspeed_tripped"] == 1.0) and row["motors_enabled"] == 0.0,
        t_exceeded,
    )
    assert t_faulted is not None
    assert t_faulted - t_exceeded <= 0.1


def test_rc_loss_keeps_balancing(sim_binary):
    rows, _ = run_sim(sim_binary, ARM + "3 rc_frames off\n8 end\n")
    assert all(row["balance_state"] == STATE_RUN for row in from_time(rows, 1.0))
    assert all(abs(row["pitch_rad"]) < 0.035 for row in from_time(rows, 4.0))


def test_ch5_off_disables(sim_binary):
    rows, _ = run_sim(sim_binary, ARM + "3 rc 5 192\n3.5 end\n")
    t_disabled = first_time(
        rows, lambda row: row["balance_state"] == 0.0 and row["motors_enabled"] == 0.0, 3.0
    )
    assert t_disabled is not None
    assert t_disabled - 3.0 <= 0.020


def test_overspeed_trips(sim_binary):
    rows, _ = run_sim(sim_binary, "0 lift on\n0 rc 5 192\n0.5 rc 5 1792\n2 rc 3 1792\n6 end\n")
    assert rows[-1]["overspeed_tripped"] == 1.0
    assert rows[-1]["motors_enabled"] == 0.0


def test_low_battery(sim_binary):
    rows, _ = run_sim(sim_binary, ARM + "3 battery 5.5\n4 end\n")
    assert rows[-1]["balance_state"] == STATE_FAULT
    assert rows[-1]["balance_fault"] == FAULT_INVALID_ENVELOPE


def test_flipped_motor_sign_falls(sim_binary):
    rows, _ = run_sim(sim_binary, ARM + "4 end\n", "flipMotorSign=1")
    last = rows[-1]
    assert last["fallen"] == 1.0 or last["balance_fault"] == FAULT_FALL or last["overspeed_tripped"] == 1.0


def test_flipped_gyro_sign_falls(sim_binary):
    rows, _ = run_sim(sim_binary, ARM + "4 end\n", "sensors.flipGyroX=1")
    last = rows[-1]
    assert last["fallen"] == 1.0 or last["balance_fault"] == FAULT_FALL or last["overspeed_tripped"] == 1.0
