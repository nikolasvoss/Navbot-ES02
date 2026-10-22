#!/usr/bin/env python3
"""Capture K55/K56/K57/K58 traces with CH340 DTR/RTS inactive."""

import argparse
import re
import select
import shlex
import sys
import time
from pathlib import Path


TUNE_COMMAND = re.compile(r"(?:Y[PIDL]([-+]?(?:\d+(?:\.\d*)?|\.\d+))?|K(?:0|9|55|56|57|58))\Z")

# Mirror the firmware caps in MotorDiagnostic.h; the firmware refuses anything larger too.
# Wheel-clear runs only (default caps, lowered 2026-10-22): 2.0 V and 60 rad/s, 5000 ms lease, 0.35 A.
MAX_CURRENT_A = 0.35
MAX_VOLTAGE_V = 2.0
MAX_LEASE_MS = 5000
NUMBER = r"[-+]?(?:\d+(?:\.\d*)?|\.\d+)"
CURRENT_COMMAND = re.compile(rf"T ({NUMBER}) ({NUMBER}) (\d+)")
VOLTAGE_COMMAND = re.compile(rf"M ([12]) ({NUMBER}) (\d+)")
MOTOR_STOP_OR_RESET = re.compile(r"[TM] (?:STOP|RESET)")

# Field order of the firmware DRIVE row (LogFormat.cpp) after the tag. The
# analysis scripts read this header; they reject rows whose field count differs.
DRIVE_COLUMNS = (
    "time_ms", "ch5_mode", "voltage_min_raw_v", "voltage_filtered_v",
    "imu_sequence", "imu_read_time_us", "imu_dt_s", "imu_accepted_samples",
    "imu_long_gap_count", "imu_max_interval_us",
    "control_start_us", "control_execution_us", "max_loop_entry_interval_us",
    "ch3_target", "motor1_velocity_f", "motor2_velocity_f", "tick_dt_s",
    "body_pitching_f",
    "roll_ok",
    "motor1_target", "motor2_target",
    "motor1_current_sp", "motor2_current_sp", "motor1_voltage_q", "motor2_voltage_q",
    "motor1_shaft_velocity", "motor2_shaft_velocity",
    "battery_raw_instant_v", "battery_filtered_instant_v",
    "top_ball_x", "body_pitching", "max_servo_range_deg",
    "sequence", "logger_dropped", "uart_write_failures",
)
TRACE_HEADERS = {
    55: "TRACE,time_ms,ch5_mode,ch6_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,pitch_deg,servo1_cmd_deg,servo2_cmd_deg,servo3_cmd_deg,servo4_cmd_deg,servo1_range_deg,servo2_range_deg,servo3_range_deg,servo4_range_deg,motor1_target,motor2_target,sequence,logger_dropped,uart_write_failures",
    56: "CTRL,time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,gyro_z_rad_s,body_turn_command,yaw_error,yaw_output,motor1_target,motor2_target,max_servo_range_deg,sequence,logger_dropped,uart_write_failures",
    57: "BAL,time_ms,ch5_mode,voltage_min_raw_v,roll_deg,motor1_target,motor2_target,max_servo_range_deg,sequence,logger_dropped,uart_write_failures",
    58: "DRIVE," + ",".join(DRIVE_COLUMNS),
}


def motion_command_allowed(command):
    """Accept only the bounded T/M grammar of the firmware diagnostic commands."""
    if MOTOR_STOP_OR_RESET.fullmatch(command):
        return True
    match = CURRENT_COMMAND.fullmatch(command)
    if match:
        left, right, lease = float(match[1]), float(match[2]), int(match[3])
        return abs(left) <= MAX_CURRENT_A and abs(right) <= MAX_CURRENT_A and 1 <= lease <= MAX_LEASE_MS
    match = VOLTAGE_COMMAND.fullmatch(command)
    if match:
        voltage, lease = float(match[2]), int(match[3])
        return abs(voltage) <= MAX_VOLTAGE_V and 1 <= lease <= MAX_LEASE_MS
    return False


def interactive_command(text):
    """Return the command to send for one line of stdin, or None if it is not whitelisted."""
    command = text.strip().upper()
    if TUNE_COMMAND.fullmatch(command) or motion_command_allowed(command):
        return command
    return None


def drain(connection, log, seconds):
    """Log firmware replies for a short time after K0; result text is printed only in Idle."""
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        line = connection.readline().decode("ascii", errors="replace").rstrip()
        if line:
            log.write(line + "\n")
            print(line, flush=True)


def main():
    parser = argparse.ArgumentParser(
        description=__doc__,
        epilog=("After capture, summarize the log for an agent with:\n"
                "  python3 scripts/summarize_balance_trace.py balance-trace.log\n"
                "See scripts/summarize_balance_trace.py for the repeatable run summary."),
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--port", default="/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0")
    parser.add_argument("--baud-rate", type=int, default=576_000,
                        help="serial baud rate (default: 576000)")
    parser.add_argument("--seconds", type=float, default=20, help="capture time after trace starts (default: 20)")
    parser.add_argument("--output", type=Path, default=Path("balance-trace.log"))
    parser.add_argument("--trace-mode", type=int, choices=(55, 56, 57, 58), default=55,
                        help="55: actuator trace (default); 56: angle/yaw; 57: balance P/I/D; 58: drive-stop trace")
    parser.add_argument("--interactive", action="store_true",
                        help="send Y/K tuning commands and bounded T/M motor commands from stdin "
                             "while recording; prints every tenth trace row")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    import serial  # Deferred so --help and the command whitelist work without pyserial.

    connection = serial.Serial(baudrate=args.baud_rate, timeout=0.2)
    connection.port = args.port
    connection.dtr = False
    connection.rts = False

    try:
        with connection, args.output.open("w", encoding="utf-8") as log:
            log.write("# " + TRACE_HEADERS[args.trace_mode] + "\n")
            print(f"Waiting for firmware startup on {args.port}...", flush=True)
            startup_deadline = time.monotonic() + 12
            while time.monotonic() < startup_deadline:
                line = connection.readline().decode("ascii", errors="replace").rstrip()
                if line:
                    log.write(line + "\n")
                    if "Motor ready." in line:
                        break

            connection.write(f"K{args.trace_mode}\n".encode("ascii"))
            if args.trace_mode in (56, 57):
                print(f"Recording for {args.seconds:g} s. Keep CH5 off for the first baseline capture.", flush=True)
            else:
                print(f"Recording for {args.seconds:g} s. Switch CH5 once while the robot is supported.", flush=True)
            if args.interactive:
                print("Type tuning commands and press Enter (for example YP10, K0). "
                      "Motor commands (T/M) need balance off.", flush=True)
            deadline = time.monotonic() + args.seconds
            trace_count = 0
            input_open = True
            while time.monotonic() < deadline:
                if args.interactive and input_open and select.select([sys.stdin], [], [], 0)[0]:
                    raw = sys.stdin.readline()
                    if not raw:
                        input_open = False
                    else:
                        command = interactive_command(raw)
                        if command:
                            connection.write((command + "\n").encode("ascii"))
                            log.write("# CMD " + command + "\n")
                            print("Sent " + command, flush=True)
                        elif raw.strip():
                            print("Ignored unsupported command: " + raw.strip().upper(), flush=True)
                line = connection.readline().decode("ascii", errors="replace").rstrip()
                if line:
                    log.write(line + "\n")
                    if line.startswith(("TRACE,", "CTRL,", "BAL,", "DRIVE,")):
                        trace_count += 1
                        if args.interactive and trace_count % 10:
                            continue
                    if args.interactive or line.startswith(("TRACE,", "CTRL,", "BAL,", "DRIVE,")) or "Voltage:" in line or "Brownout" in line or "system run." in line:
                        print(line, flush=True)
            connection.write(b"K0\n")
            drain(connection, log, 2.0)
            print(f"Saved {args.output.resolve()}", flush=True)
            print("Summarize this run with: python3 scripts/summarize_balance_trace.py "
                  + shlex.quote(str(args.output)), flush=True)
    except (OSError, serial.SerialException) as error:
        print(f"capture_balance_trace: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
