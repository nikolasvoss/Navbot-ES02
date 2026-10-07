#!/usr/bin/env python3
"""Capture K55/K56/K57 traces with CH340 DTR/RTS inactive."""

import argparse
import re
import select
import shlex
import sys
import time
from pathlib import Path

import serial


TUNE_COMMAND = re.compile(r"(?:U[01]|[PSYR][PIDL](?:[-+]?(?:\d+(?:\.\d*)?|\.\d+))?|[VW](?:[-+]?(?:\d+(?:\.\d*)?|\.\d+))?|K(?:0|9|55|56|57|58))\Z")


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
                        help="send U/P/S/Y/R/V/W/K tuning commands from stdin while recording; prints every tenth trace row")
    args = parser.parse_args()
    if args.seconds <= 0:
        parser.error("--seconds must be positive")

    connection = serial.Serial(baudrate=args.baud_rate, timeout=0.2)
    connection.port = args.port
    connection.dtr = False
    connection.rts = False

    try:
        with connection, args.output.open("w", encoding="utf-8") as log:
            if args.trace_mode == 55:
                log.write("# TRACE,time_ms,ch5_mode,ch6_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,pitch_deg,servo1_cmd_deg,servo2_cmd_deg,servo3_cmd_deg,servo4_cmd_deg,servo1_range_deg,servo2_range_deg,servo3_range_deg,servo4_range_deg,motor1_target,motor2_target,sequence,logger_dropped,uart_write_failures\n")
            elif args.trace_mode == 56:
                log.write("# CTRL,time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,roll_deg,gyro_z_rad_s,body_turn_command,angle_error_deg,angle_output,yaw_error,yaw_output,motor1_target,motor2_target,max_servo_range_deg,sequence,logger_dropped,uart_write_failures\n")
            elif args.trace_mode == 57:
                log.write("# BAL,time_ms,ch5_mode,voltage_min_raw_v,roll_deg,angle_error_deg,angle_out_p,angle_out_i,angle_out_d,body_x,motor1_target,motor2_target,max_servo_range_deg,sequence,logger_dropped,uart_write_failures\n")
            else:
                log.write("# DRIVE,time_ms,ch5_mode,voltage_min_raw_v,voltage_filtered_v,ch3_target,effective_speed_target,motor1_velocity_f,motor2_velocity_f,tick_dt_s,speed_error,speed_out_p,speed_out_i,speed_out_d,speed_output,speed_body_x_raw,body_x,body_pitching_f,roll_ok,angle_output,angle_out_p,angle_out_i,angle_out_d,wheel_speed_feedback,motor1_target,motor2_target,top_ball_x,touch_x,body_pitching,max_servo_range_deg,sequence,logger_dropped,uart_write_failures\n")
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
                print("Type tuning commands and press Enter (for example U1, YP10, V0.2, V, K0).", flush=True)
            deadline = time.monotonic() + args.seconds
            trace_count = 0
            input_open = True
            while time.monotonic() < deadline:
                if args.interactive and input_open and select.select([sys.stdin], [], [], 0)[0]:
                    command = sys.stdin.readline()
                    if not command:
                        input_open = False
                    else:
                        command = command.strip().upper()
                        if TUNE_COMMAND.fullmatch(command):
                            connection.write((command + "\n").encode("ascii"))
                            log.write("# CMD " + command + "\n")
                            print("Sent " + command, flush=True)
                        elif command:
                            print("Ignored unsupported command: " + command, flush=True)
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
            print(f"Saved {args.output.resolve()}", flush=True)
            print("Summarize this run with: python3 scripts/summarize_balance_trace.py "
                  + shlex.quote(str(args.output)), flush=True)
    except (OSError, serial.SerialException) as error:
        print(f"capture_balance_trace: {error}", file=sys.stderr)
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
