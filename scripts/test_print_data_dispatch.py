#!/usr/bin/env python3
"""Compile the print_data selector and trace dispatch from the sketch source."""

from pathlib import Path
import subprocess
import tempfile


ROOT = Path(__file__).resolve().parents[1]
SKETCH = ROOT / "src/ES-02/OllieFOCdrive/OllieFOCdrive.ino"


def extract_function(source: str) -> str:
    start = source.index("void print_data(void) {")
    opening = source.index("{", start)
    depth = 0
    for index in range(opening, len(source)):
        if source[index] == "{":
            depth += 1
        elif source[index] == "}":
            depth -= 1
            if depth == 0:
                return source[start : index + 1]
    raise AssertionError("print_data has no matching closing brace")


def make_trace_harness(function: str) -> str:
    debug_selector = function.rindex("  const Logging::DebugSelector selector =")
    sample_start = function.index("  Logging::DebugSample sample{};", debug_selector)
    trace_switch = function.index("  switch ((int)Select)", sample_start)
    debug_prefix = function[:function.index("  if (selection >= 60")]
    debug_prefix += function[debug_selector:sample_start]
    # The payload assembly needs the full Arduino sketch state. Keep the real
    # selector guard and trace switch while omitting only the unrelated payload.
    if debug_prefix.rstrip().endswith("{"):
        debug_prefix += "\n  }\n"
    function = debug_prefix + function[trace_switch:]

    return r'''#include "LogSamples.h"
#include <algorithm>
#include <cassert>
#include <cstdint>

struct PidState { float error = 0, output = 0, outP = 0, outI = 0, outD = 0; };
struct TouchState { float XPdatF = 0; };
TouchState Touch;
PidState Angle_Pid, Yaw_Pid, Speed_Pid;
namespace Logging {
int traceSubmits = 0, controlSubmits = 0, balanceSubmits = 0, driveSubmits = 0;
void submit(const TraceSample&) { ++traceSubmits; }
void submit(const ControlSample&) { ++controlSubmits; }
void submit(const BalanceSample&) { ++balanceSubmits; }
void submit(const DriveSample&) { ++driveSubmits; }
}
namespace RobotLogCapture {
bool selectedDebugDue(Logging::DebugSelector, uint32_t) { return true; }
void copyServoRangesAndBeginNextWindow(int32_t ranges[4]) {
  for (int i = 0; i < 4; ++i) ranges[i] = 0;
}
}
float VoltageADCMin = 0, VoltageADC = 0, Voltage = 0, rollBiasCorrected = 0, pitchBiasCorrected = 0;
float BodyTurn = 0, MovementSpeed = 0, m1FilteredVelocityRadPerSec = 0, m2FilteredVelocityRadPerSec = 0;
float driveEffectiveSpeed = 0, driveSpeedBodyXRaw = 0, BodyX = 0, BodyPitching_f = 0;
float wheelVelocityFeedbackCorrection = 0, top_ball_x = 0, BodyPitching = 0, controlTimestepSec = 0;
int pid_gains_mode = 0, posture_or_mark_mode = 0, RobotTumble = 0;
struct Attitude { struct { float z = 0; } gyro; } attitude;
constexpr int ROBOT_NOT_TUMBLING = 0;
float motor1Target = 0, motor2Target = 0;
struct Motor { float target = 0; } motor1, motor2;
int servoTraceAngle[4] = {};
float Select = 0;
uint32_t controlGateSequence = 0, activeTraceSequence = 0;
uint32_t clockMs = 1;
uint32_t millis() { return clockMs; }
bool pid_gains_mode_is_enabled(int) { return false; }
int max(int a, int b) { return std::max(a, b); }
void print_data(void);
''' + function + r'''

int submitCount() {
  return Logging::traceSubmits + Logging::controlSubmits +
         Logging::balanceSubmits + Logging::driveSubmits;
}

int main() {
  const int selectors[] = {55, 56, 57, 58};
  for (int i = 0; i < 4; ++i) {
    Select = selectors[i];
    controlGateSequence = 1;
    const int before = submitCount();
    print_data();
    assert(submitCount() == before);
    controlGateSequence = 7;
    print_data();
    assert(submitCount() == before + 1);
  }
  assert(Logging::traceSubmits == 1);
  assert(Logging::controlSubmits == 1);
  assert(Logging::balanceSubmits == 1);
  assert(Logging::driveSubmits == 1);
}
'''


def main() -> None:
    source = SKETCH.read_text()
    function = extract_function(source)
    harness = make_trace_harness(function)
    with tempfile.TemporaryDirectory(prefix="print-data-dispatch-") as temp:
        temp_path = Path(temp)
        cpp_path = temp_path / "print_data_dispatch.cpp"
        binary_path = temp_path / "print_data_dispatch"
        cpp_path.write_text(harness)
        subprocess.run(
            ["g++", "-std=c++11", "-Wall", "-Wextra", "-Werror", "-I",
             str(ROOT / "src/ES-02/OllieFOCdrive"), str(cpp_path), "-o", str(binary_path)],
            check=True,
        )
        subprocess.run([str(binary_path)], check=True)
    print("print_data trace dispatch and 7-gate cadence passed")


if __name__ == "__main__":
    main()
