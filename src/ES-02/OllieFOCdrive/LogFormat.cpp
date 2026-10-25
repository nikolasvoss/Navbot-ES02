#include <stdarg.h>
#include <stdio.h>

#include "LoggingInternal.h"

namespace LoggingInternal {
namespace {

int appendf(char *buffer, size_t capacity, size_t &length, const char *format, ...) {
  if (length >= capacity) return -1;
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(buffer + length, capacity - length, format, args);
  va_end(args);
  if (written < 0 || static_cast<size_t>(written) >= capacity - length) return -1;
  length += static_cast<size_t>(written);
  return static_cast<int>(length);
}

int formatDebug(const Logging::DebugSample &sample, char *buffer, size_t capacity) {
  size_t length = 0;
  const Logging::DebugPayload &p = sample.payload;
  switch (sample.selector) {
    case Logging::DebugSelector::K1:
      return appendf(buffer, capacity, length, "dt:%.6f Roll:%.2f Pitch:%.2f Yaw:%.2f\n", p.timedVector3.dtSec,
                     p.timedVector3.x, p.timedVector3.y, p.timedVector3.z);
    case Logging::DebugSelector::K2:
      return appendf(buffer, capacity, length, "dt:%.6f accx:%.2f accy:%.2f accz:%.2f\n", p.timedVector3.dtSec,
                     p.timedVector3.x, p.timedVector3.y, p.timedVector3.z);
    case Logging::DebugSelector::K3:
      return appendf(buffer, capacity, length, "dt:%.6f gyrox:%.4f gyroy:%.4f gyroz:%.4f\n", p.timedVector3.dtSec,
                     p.timedVector3.x, p.timedVector3.y, p.timedVector3.z);
    case Logging::DebugSelector::K4:
      return appendf(buffer, capacity, length, "dt:%.6f Roll:%.2f Pitch:%.2f Yaw:%.2f\n", p.timedVector3.dtSec,
                     p.timedVector3.x, p.timedVector3.y, p.timedVector3.z);
    case Logging::DebugSelector::K5:
      return appendf(buffer, capacity, length, "dt:%.6f eRoll:%.2f ePitch:%.2f eYaw:%.2f\n", p.timedVector3.dtSec,
                     p.timedVector3.x, p.timedVector3.y, p.timedVector3.z);
    case Logging::DebugSelector::K6:
      return appendf(buffer, capacity, length, " v1:%.2f v2:%.2f\n", p.motorVelocities.left, p.motorVelocities.right);
    case Logging::DebugSelector::K7:
      return appendf(buffer, capacity, length, " v1:%.2f v1f:%.2f\n", p.pair.first, p.pair.second);
    case Logging::DebugSelector::K8:
      for (size_t i = 0; i < 10; ++i)
        if (appendf(buffer, capacity, length, " ch:%d", static_cast<int>(p.receiverChannels.channels[i])) < 0)
          return -1;
      return appendf(buffer, capacity, length, " sbus_dt_ms:%.2f \n", p.receiverChannels.frameDtMs);
    case Logging::DebugSelector::K9:
      return appendf(
          buffer, capacity, length, " PP:%.2f PI:%.2f PD:%.2f SP:%.2f SI:%.2f SD:%.2f YP:%.2f YI:%.2f YD:%.2fdt:%.6f\n",
          p.controllerGains.angle.kp, p.controllerGains.angle.ki, p.controllerGains.angle.kd,
          p.controllerGains.speed.kp, p.controllerGains.speed.ki, p.controllerGains.speed.kd, p.controllerGains.yaw.kp,
          p.controllerGains.yaw.ki, p.controllerGains.yaw.kd, p.controllerGains.dtSec);
    case Logging::DebugSelector::K10:
      return appendf(buffer, capacity, length, " twoKp:%.2f twoKi:%.2f Roll:%.2f Pitch:%.2fIMUdt:%.6f\n",
                     p.mahony.twoKp, p.mahony.twoKi, p.mahony.rollDeg, p.mahony.pitchDeg, p.mahony.imuDtSec);
    case Logging::DebugSelector::K11:
      return appendf(buffer, capacity, length, " x:%.2f y:%.2f z:%.2f gx:%.2f gy:%.2f gz:%.2f IMUdt:%.6f\n",
                     p.imuAxes.angleX, p.imuAxes.angleY, p.imuAxes.angleZ, p.imuAxes.gyroX, p.imuAxes.gyroY,
                     p.imuAxes.gyroZ, p.imuAxes.dtSec);
    case Logging::DebugSelector::K12:
      return appendf(buffer, capacity, length, " x:%.2f Roll:%.2f\n", p.pair.first, p.pair.second);
    case Logging::DebugSelector::K13:
      return appendf(buffer, capacity, length, "dt:%.6f gyroxf:%.2f gyroyf:%.2f gyrozf:%.2f\n", p.timedVector3.dtSec,
                     p.timedVector3.x, p.timedVector3.y, p.timedVector3.z);
    case Logging::DebugSelector::K14:
      return appendf(buffer, capacity, length, "dt:%.6f accx:%.2f accy:%.2f accz:%.2f\n", p.timedVector3.dtSec,
                     p.timedVector3.x, p.timedVector3.y, p.timedVector3.z);
    case Logging::DebugSelector::K15:
      return appendf(buffer, capacity, length, " accy:%.2f accyf:%.2f\n", p.pair.first, p.pair.second);
    case Logging::DebugSelector::K16:
      return appendf(buffer, capacity, length, " gyro:%.2f gyrof:%.2f\n", p.pair.first, p.pair.second);
    case Logging::DebugSelector::K17:
      return appendf(buffer, capacity, length, " current_sp:%.6f\n", p.currentSetpoint.amperes);
    case Logging::DebugSelector::K18:
      return appendf(buffer, capacity, length, " t:%.6f a1:%.6f a11:%.6f a2:%.6f a22:%.6f\n", p.sensorAngles.target,
                     p.sensorAngles.leftAbsolute, p.sensorAngles.leftMechanical, p.sensorAngles.rightAbsolute,
                     p.sensorAngles.rightMechanical);
    case Logging::DebugSelector::K19:
      return appendf(buffer, capacity, length, " servo1:%.2f servo2:%.2f servo3:%.2f servo4:%.2f\n",
                     p.servoOffsets.servo[0], p.servoOffsets.servo[1], p.servoOffsets.servo[2],
                     p.servoOffsets.servo[3]);
    case Logging::DebugSelector::K20:
      return appendf(buffer, capacity, length, " vra:%.6f BodyRoll:%.6f BodyHeightM:%.6f\n",
                     p.ballBalanceGeometry.ballX, p.ballBalanceGeometry.bodyRoll, p.ballBalanceGeometry.bodyHeightM);
    case Logging::DebugSelector::K21:
      return appendf(buffer, capacity, length, " roll_ok:%.6f BodyPitching:%.6f\n", p.balanceState.rollOk,
                     p.balanceState.bodyPitch);
    case Logging::DebugSelector::K22:
      return appendf(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f out:%.5f\n", p.pidIntegralState.integralLimit,
                     p.pidIntegralState.integral, p.pidIntegralState.integralOutput, p.pidIntegralState.output);
    case Logging::DebugSelector::K23:
      return appendf(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f A:%.5f out:%.5f\n",
                     p.pidIntegralOutputState.integralLimit, p.pidIntegralOutputState.integral,
                     p.pidIntegralOutputState.integralOutput, p.pidIntegralOutputState.auxiliary,
                     p.pidIntegralOutputState.output);
    case Logging::DebugSelector::K24:
      return appendf(buffer, capacity, length, " EN:%.2f HZ:%.5f\n", p.pair.first, p.pair.second);
    case Logging::DebugSelector::K25:
      return appendf(buffer, capacity, length, " LpfOut:%.6f BodyPitching:%.6f\n", p.bodyPitchState.filteredDeg,
                     p.bodyPitchState.rawDeg);
    case Logging::DebugSelector::K28:
      return appendf(buffer, capacity, length, "  P:%.2f  R:%.5f  H:%.5f  S:%.2f  vra:%.2f  vra:%.2f\n",
                     p.filteredGeometry.bodyPitch, p.filteredGeometry.bodyRoll, p.filteredGeometry.bodyHeightM,
                     p.filteredGeometry.slideStep, p.filteredGeometry.ballX, p.filteredGeometry.ballY);
    case Logging::DebugSelector::K31:
      return appendf(buffer, capacity, length, " E:%.6f it:%.5f il:%.5f oI:%.5f out:%.5f\n", p.pidState.error,
                     p.pidState.integralLimit, p.pidState.integral, p.pidState.integralOutput, p.pidState.output);
    case Logging::DebugSelector::K32:
      return appendf(buffer, capacity, length, " RP:%.6f RI:%.6f RD:%.6f\n", p.pidCoefficients.kp, p.pidCoefficients.ki,
                     p.pidCoefficients.kd);
    case Logging::DebugSelector::K33:
      return appendf(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f A:%.5f out:%.5f\n",
                     p.pidIntegralOutputState.integralLimit, p.pidIntegralOutputState.integral,
                     p.pidIntegralOutputState.integralOutput, p.pidIntegralOutputState.auxiliary,
                     p.pidIntegralOutputState.output);
    case Logging::DebugSelector::K37:
      return appendf(buffer, capacity, length, " sbus_vra:%.2f sbus_vraf:%.2f sbus_vrb:%.2f sbus_vrbf:%.2f\n",
                     p.ballPosition.x, p.ballPosition.xFiltered, p.ballPosition.y, p.ballPosition.yFiltered);
    case Logging::DebugSelector::K38:
      return appendf(buffer, capacity, length, " X OUT:%.2f Y OUT:%.2f\n", p.balanceOutputPair.bodyPitch,
                     p.balanceOutputPair.controllerOutput);
    case Logging::DebugSelector::K41:
      return appendf(buffer, capacity, length, " roll_ok:%.5f pa:%.5f out:%.5f\n", p.rollOutput.rollOk,
                     p.rollOutput.bodyPitch, p.rollOutput.output);
    case Logging::DebugSelector::K42:
      return appendf(buffer, capacity, length, " P:%.5f P1:%.5f P3:%.5f\n", p.rollCorrection.rollOk,
                     p.rollCorrection.bodyPitch, p.rollCorrection.correctedBodyPitch);
    case Logging::DebugSelector::K43:
      return appendf(buffer, capacity, length, " Vdat:%d Vdatf:%.2f V:%.5f\n", static_cast<int>(p.voltageState.adc),
                     p.voltageState.filteredV, p.voltageState.voltageV);
    case Logging::DebugSelector::K44:
      return appendf(buffer, capacity, length, " PidParameterTuning:%d TargetBodyHeightM:%.6f\n",
                     static_cast<int>(p.tuningState.enabled), p.tuningState.targetBodyHeightM);
    case Logging::DebugSelector::K45:
      return appendf(buffer, capacity, length, " RobotTumble:%d roll_ok:%.6f Angle_Pid.error:%.6f\n",
                     static_cast<int>(p.tumbleState.tumbled), p.tumbleState.rollOk, p.tumbleState.angleError);
    case Logging::DebugSelector::K46:
      return appendf(buffer, capacity, length, "%lu,%.7f,%.7f,%.7f,%.7f,%.7f,%.7f\n",
                     static_cast<unsigned long>(p.gyroFft.timestampUs), p.gyroFft.gyroX, p.gyroFft.gyroY,
                     p.gyroFft.gyroZ, p.gyroFft.gyroXFiltered, p.gyroFft.gyroYFiltered, p.gyroFft.gyroZFiltered);
    default:
      if (Logging::isChannelDebugSelector(static_cast<int>(sample.selector)))
        return appendf(buffer, capacity, length, "%.3f,%d\n", p.channelState.seconds,
                       static_cast<int>(p.channelState.channel));
      return -1;
  }
}

}  // namespace

int formatRecord(const Record &record, char *buffer, size_t capacity, uint32_t droppedRecords, uint32_t writeFailures) {
  if (buffer == nullptr || capacity == 0) return -1;
  switch (record.kind) {
    case RecordKind::Message: {
      const MessageRecord &message = record.payload.message;
      const int length = snprintf(buffer, capacity, "%.*s: %.*s\n", static_cast<int>(message.tagLength), message.tag,
                                  static_cast<int>(message.textLength), message.text);
      return length < 0 || static_cast<size_t>(length) >= capacity ? -1 : length;
    }
    case RecordKind::Diagnostic: {
      const Logging::DiagnosticSample &s = record.payload.diagnostic;
      return snprintf(buffer, capacity, "%lu,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%ld,%d,%d,%lu,%lu\n",
                      static_cast<unsigned long>(s.timestampUs), static_cast<unsigned long>(s.sequence), s.gyroX,
                      s.gyroY, s.gyroZ, s.accelX, s.accelY, s.accelZ, s.rollDeg, s.pitchDeg, s.yawDeg, s.batteryRawV,
                      static_cast<long>(s.receiverFrameAgeMs), static_cast<int>(s.receiverFailsafe),
                      static_cast<int>(s.imuReady), static_cast<unsigned long>(droppedRecords),
                      static_cast<unsigned long>(writeFailures));
    }
    case RecordKind::Trace: {
      const Logging::TraceSample &s = record.payload.trace;
      return snprintf(
          buffer, capacity, "TRACE,%lu,%d,%d,%.3f,%.3f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%lu,%lu,%lu\n",
          static_cast<unsigned long>(s.timestampMs), static_cast<int>(s.gainMode),
          static_cast<int>(s.postureOrMarkMode), s.minimumBatteryRawV, s.batteryV, s.rollOk, s.pitchOk,
          static_cast<int>(s.servoAngle[0]), static_cast<int>(s.servoAngle[1]), static_cast<int>(s.servoAngle[2]),
          static_cast<int>(s.servoAngle[3]), static_cast<int>(s.servoRange[0]), static_cast<int>(s.servoRange[1]),
          static_cast<int>(s.servoRange[2]), static_cast<int>(s.servoRange[3]), s.leftMotorTarget, s.rightMotorTarget,
          static_cast<unsigned long>(s.sequence), static_cast<unsigned long>(droppedRecords),
          static_cast<unsigned long>(writeFailures));
    }
    case RecordKind::Control: {
      const Logging::ControlSample &s = record.payload.control;
      return snprintf(buffer, capacity,
                      "CTRL,%lu,%d,%.3f,%.3f,%.2f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%lu,%lu,%lu\n",
                      static_cast<unsigned long>(s.timestampMs), static_cast<int>(s.gainMode), s.minimumBatteryRawV,
                      s.batteryV, s.rollOk, s.yawRateRadPerSec, s.bodyTurn, s.angleError, s.angleOutput, s.yawError,
                      s.yawOutput, s.leftMotorTarget, s.rightMotorTarget, static_cast<int>(s.maxServoRange),
                      static_cast<unsigned long>(s.sequence), static_cast<unsigned long>(droppedRecords),
                      static_cast<unsigned long>(writeFailures));
    }
    case RecordKind::Balance: {
      const Logging::BalanceSample &s = record.payload.balance;
      return snprintf(buffer, capacity, "BAL,%lu,%d,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%d,%lu,%lu,%lu\n",
                      static_cast<unsigned long>(s.timestampMs), static_cast<int>(s.gainMode), s.minimumBatteryRawV,
                      s.rollOk, s.angleError, s.angleProportional, s.angleIntegral, s.angleDerivative, s.bodyX,
                      s.leftMotorTarget, s.rightMotorTarget, static_cast<int>(s.maxServoRange),
                      static_cast<unsigned long>(s.sequence), static_cast<unsigned long>(droppedRecords),
                      static_cast<unsigned long>(writeFailures));
    }
    case RecordKind::Drive: {
      const Logging::DriveSample &s = record.payload.drive;
      return snprintf(buffer, capacity,
                      "DRIVE,%lu,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%."
                      "4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%d,%lu,%lu,%lu\n",
                      static_cast<unsigned long>(s.timestampMs), static_cast<int>(s.gainMode), s.minimumBatteryRawV,
                      s.batteryV, s.requestedSpeed, s.effectiveSpeed, s.leftWheelVelocity, s.rightWheelVelocity,
                      s.controlDtSec, s.speedError, s.speedProportional, s.speedIntegral, s.speedDerivative,
                      s.speedOutput, s.driveBodyXRaw, s.bodyX, s.bodyPitchFiltered, s.rollOk, s.angleOutput,
                      s.angleProportional, s.angleIntegral, s.angleDerivative, s.wheelSpeedFeedbackOutput,
                      s.leftMotorTarget, s.rightMotorTarget, s.ballX, s.bodyPitch,
                      static_cast<int>(s.maxServoRange), static_cast<unsigned long>(s.sequence),
                      static_cast<unsigned long>(droppedRecords), static_cast<unsigned long>(writeFailures));
    }
    case RecordKind::Debug:
      return formatDebug(record.payload.debug, buffer, capacity);
  }
  return -1;
}

}  // namespace LoggingInternal
