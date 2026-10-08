#pragma once

#include <stdint.h>
#include <stdio.h>
#include <stdarg.h>

struct SerialLogSelected {
  int32_t selector;
  float values[26];
  int32_t integers[10];
};

enum SerialLogKind : uint8_t {
  SERIAL_LOG_DIAGNOSTIC,
  SERIAL_LOG_TRACE,
  SERIAL_LOG_CONTROL,
  SERIAL_LOG_BALANCE,
  SERIAL_LOG_DRIVE,
  SERIAL_LOG_SELECTED_DEBUG
};

struct SerialLogRecord {
  SerialLogKind kind;
  uint32_t timestamp;
  uint32_t sequence;
  uint32_t dropped;
  uint32_t writeFailures;
  union {
    float values[29];
    SerialLogSelected selected;
  };
};

static inline bool appendSerialLogFormat(char *buffer, size_t capacity, size_t &length,
                                         const char *format, ...) {
  if (length >= capacity) return false;
  va_list args;
  va_start(args, format);
  const int written = vsnprintf(buffer + length, capacity - length, format, args);
  va_end(args);
  if (written < 0 || static_cast<size_t>(written) >= capacity - length) return false;
  length += static_cast<size_t>(written);
  return true;
}


static inline bool formatSelectedDebug(char *buffer, size_t capacity, size_t &length, const SerialLogSelected &row) {
  const float *v = row.values;
  const int32_t *n = row.integers;
  if (row.selector >= 60 && row.selector <= 75)
    return appendSerialLogFormat(buffer, capacity, length, "%.3f,%d\n", v[0], static_cast<int>(n[0]));
  switch (row.selector) {
    case 1: return appendSerialLogFormat(buffer, capacity, length, "dt:%.6f Roll:%.2f Pitch:%.2f Yaw:%.2f\n", v[0], v[1], v[2], v[3]);
    case 2: return appendSerialLogFormat(buffer, capacity, length, "dt:%.6f accx:%.2f accy:%.2f accz:%.2f\n", v[0], v[1], v[2], v[3]);
    case 3: return appendSerialLogFormat(buffer, capacity, length, "dt:%.6f gyrox:%.4f gyroy:%.4f gyroz:%.4f\n", v[0], v[1], v[2], v[3]);
    case 4: return appendSerialLogFormat(buffer, capacity, length, "dt:%.6f Roll:%.2f Pitch:%.2f Yaw:%.2f\n", v[0], v[1], v[2], v[3]);
    case 5: return appendSerialLogFormat(buffer, capacity, length, "dt:%.6f eRoll:%.2f ePitch:%.2f eYaw:%.2f\n", v[0], v[1], v[2], v[3]);
    case 6: return appendSerialLogFormat(buffer, capacity, length, " v1:%.2f v2:%.2f\n", v[0], v[1]);
    case 7: return appendSerialLogFormat(buffer, capacity, length, " v1:%.2f v1f:%.2f\n", v[0], v[1]);
    case 8:
      for (size_t i = 0; i < 10; ++i)
        if (!appendSerialLogFormat(buffer, capacity, length, " ch:%d", static_cast<int>(n[i]))) return false;
      return appendSerialLogFormat(buffer, capacity, length, " sbus_dt_ms:%.2f \n", v[0]);
    case 9: return appendSerialLogFormat(buffer, capacity, length, " PP:%.2f PI:%.2f PD:%.2f SP:%.2f SI:%.2f SD:%.2f YP:%.2f YI:%.2f YD:%.2fdt:%.6f\n", v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9]);
    case 10: return appendSerialLogFormat(buffer, capacity, length, " twoKp:%.2f twoKi:%.2f Roll:%.2f Pitch:%.2fIMUdt:%.6f\n", v[0], v[1], v[2], v[3], v[4]);
    case 11: return appendSerialLogFormat(buffer, capacity, length, " x:%.2f y:%.2f z:%.2f gx:%.2f gy:%.2f gz:%.2f IMUdt:%.6f\n", v[0], v[1], v[2], v[3], v[4], v[5], v[6]);
    case 12: return appendSerialLogFormat(buffer, capacity, length, " x:%.2f Roll:%.2f\n", v[0], v[1]);
    case 13: return appendSerialLogFormat(buffer, capacity, length, "dt:%.6f gyroxf:%.2f gyroyf:%.2f gyrozf:%.2f\n", v[0], v[1], v[2], v[3]);
    case 14: return appendSerialLogFormat(buffer, capacity, length, "dt:%.6f accx:%.2f accy:%.2f accz:%.2f\n", v[0], v[1], v[2], v[3]);
    case 15: return appendSerialLogFormat(buffer, capacity, length, " accy:%.2f accyf:%.2f\n", v[0], v[1]);
    case 16: return appendSerialLogFormat(buffer, capacity, length, " gyro:%.2f gyrof:%.2f\n", v[0], v[1]);
    case 17: return appendSerialLogFormat(buffer, capacity, length, " current_sp:%.6f\n", v[0]);
    case 18: return appendSerialLogFormat(buffer, capacity, length, " t:%.6f a1:%.6f a11:%.6f a2:%.6f a22:%.6f\n", v[0], v[1], v[2], v[3], v[4]);
    case 19: return appendSerialLogFormat(buffer, capacity, length, " servo1:%.2f servo2:%.2f servo3:%.2f servo4:%.2f\n", v[0], v[1], v[2], v[3]);
    case 20: return appendSerialLogFormat(buffer, capacity, length, " vra:%.6f BodyRoll:%.6f LegLength:%.6f\n", v[0], v[1], v[2]);
    case 21: return appendSerialLogFormat(buffer, capacity, length, " roll_ok:%.6f BodyPitching:%.6f\n", v[0], v[1]);
    case 22: return appendSerialLogFormat(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f out:%.5f\n", v[0], v[1], v[2], v[3]);
    case 23: return appendSerialLogFormat(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f A:%.5f out:%.5f\n", v[0], v[1], v[2], v[3], v[4]);
    case 24: return appendSerialLogFormat(buffer, capacity, length, " EN:%.2f HZ:%.5f\n", v[0], v[1]);
    case 25: return appendSerialLogFormat(buffer, capacity, length, " LpfOut:%.6f BodyPitching:%.6f\n", v[0], v[1]);
    case 26:
      if (n[0] == 1) return appendSerialLogFormat(buffer, capacity, length, "  aX:%d  aY:%d\n", static_cast<int>(n[1]), static_cast<int>(n[2]));
      if (n[0] == 0) return appendSerialLogFormat(buffer, capacity, length, "  tX:%d  tY:%d\n", static_cast<int>(n[1]), static_cast<int>(n[2]));
      return true;
    case 27: return appendSerialLogFormat(buffer, capacity, length, "  aX:%d  aY:%d  aXF:%.2f  aYF:%.2f\n", static_cast<int>(n[0]), static_cast<int>(n[1]), v[0], v[1]);
    case 28: return appendSerialLogFormat(buffer, capacity, length, "  P:%.2f  R:%.5f  H:%.5f  S:%.2f  vra:%.2f  vra:%.2f\n", v[0], v[1], v[2], v[3], v[4], v[5]);
    case 29: return appendSerialLogFormat(buffer, capacity, length, " Kp:%.6f Ki:%.6f Kd:%.6f deriv:%.2f out:%.2f\n", v[0], v[1], v[2], v[3], v[4]);
    case 30: return appendSerialLogFormat(buffer, capacity, length, " deriv:%.2f\n", v[0]);
    case 31: return appendSerialLogFormat(buffer, capacity, length, " E:%.6f it:%.5f il:%.5f oI:%.5f out:%.5f\n", v[0], v[1], v[2], v[3], v[4]);
    case 32: return appendSerialLogFormat(buffer, capacity, length, " RP:%.6f RI:%.6f RD:%.6f\n", v[0], v[1], v[2]);
    case 33: return appendSerialLogFormat(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f A:%.5f out:%.5f\n", v[0], v[1], v[2], v[3], v[4]);
    case 34: return appendSerialLogFormat(buffer, capacity, length, " Kp:%.6f Ki:%.6f Kd:%.6f deriv:%.2f out:%.2f\n", v[0], v[1], v[2], v[3], v[4]);
    case 35: return appendSerialLogFormat(buffer, capacity, length, " deriv:%.2f\n", v[0]);
    case 36: return appendSerialLogFormat(buffer, capacity, length, " state:%d start:%d\n", static_cast<int>(n[0]), static_cast<int>(n[1]));
    case 37: return appendSerialLogFormat(buffer, capacity, length, " sbus_vra:%.2f sbus_vraf:%.2f sbus_vrb:%.2f sbus_vrbf:%.2f\n", v[0], v[1], v[2], v[3]);
    case 38: return appendSerialLogFormat(buffer, capacity, length, " X OUT:%.2f Y OUT:%.2f\n", v[0], v[1]);
    case 39: return appendSerialLogFormat(buffer, capacity, length, " it:%.5f il:%.5f oI:%.5f out:%.5f\n", v[0], v[1], v[2], v[3]);
    case 40:
      if (n[0] == 1) return appendSerialLogFormat(buffer, capacity, length, "  aX:%d  aY:%d\n", static_cast<int>(n[1]), static_cast<int>(n[2]));
      if (n[0] == 0) return appendSerialLogFormat(buffer, capacity, length, "  tX:%d  tX:%d\n", static_cast<int>(n[1]), static_cast<int>(n[2]));
      return true;
    case 41: return appendSerialLogFormat(buffer, capacity, length, " roll_ok:%.5f pa:%.5f out:%.5f\n", v[0], v[1], v[2]);
    case 42: return appendSerialLogFormat(buffer, capacity, length, " P:%.5f P1:%.5f P3:%.5f\n", v[0], v[1], v[2]);
    case 43: return appendSerialLogFormat(buffer, capacity, length, " Vdat:%d Vdatf:%.2f V:%.5f\n", static_cast<int>(n[0]), v[0], v[1]);
    case 44: return appendSerialLogFormat(buffer, capacity, length, " PidParameterTuning:%d TargetLegLength:%.6f\n", static_cast<int>(n[0]), v[0]);
    case 45: return appendSerialLogFormat(buffer, capacity, length, " RobotTumble:%d roll_ok:%.6f Angle_Pid.error:%.6f\n", static_cast<int>(n[0]), v[0], v[1]);
    default: return false;
  }
}

static inline int formatSerialLogRecord(const SerialLogRecord &row, char *buffer, size_t capacity) {
  if (row.kind == SERIAL_LOG_SELECTED_DEBUG) {
    size_t length = 0;
    return formatSelectedDebug(buffer, capacity, length, row.selected) ? static_cast<int>(length) : -1;
  }
  const float *v = row.values;
  switch (row.kind) {
    case SERIAL_LOG_DIAGNOSTIC:
      return snprintf(buffer, capacity,
                      "%lu,%lu,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.2f,%.2f,%.2f,%.3f,%ld,%d,%d,%lu,%lu\n",
                      (unsigned long)row.timestamp, (unsigned long)row.sequence,
                      v[0], v[1], v[2], v[3], v[4], v[5], v[6], v[7], v[8], v[9],
                      (long)v[10], (int)v[11], (int)v[12],
                      (unsigned long)row.dropped, (unsigned long)row.writeFailures);
    case SERIAL_LOG_TRACE:
      return snprintf(buffer, capacity,
                      "TRACE,%lu,%d,%d,%.3f,%.3f,%.2f,%.2f,%d,%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%lu,%lu,%lu\n",
                      (unsigned long)row.timestamp, (int)v[0], (int)v[1], v[2], v[3], v[4], v[5],
                      (int)v[6], (int)v[7], (int)v[8], (int)v[9],
                      (int)v[10], (int)v[11], (int)v[12], (int)v[13], v[14], v[15],
                      (unsigned long)row.sequence, (unsigned long)row.dropped,
                      (unsigned long)row.writeFailures);
    case SERIAL_LOG_CONTROL:
      return snprintf(buffer, capacity,
                      "CTRL,%lu,%d,%.3f,%.3f,%.2f,%.4f,%.4f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%d,%lu,%lu,%lu\n",
                      (unsigned long)row.timestamp, (int)v[0], v[1], v[2], v[3], v[4], v[5],
                      v[6], v[7], v[8], v[9], v[10], v[11], (int)v[12],
                      (unsigned long)row.sequence, (unsigned long)row.dropped,
                      (unsigned long)row.writeFailures);
    case SERIAL_LOG_BALANCE:
      return snprintf(buffer, capacity,
                      "BAL,%lu,%d,%.3f,%.2f,%.2f,%.2f,%.2f,%.2f,%.4f,%.2f,%.2f,%d,%lu,%lu,%lu\n",
                      (unsigned long)row.timestamp, (int)v[0], v[1], v[2], v[3], v[4], v[5],
                      v[6], v[7], v[8], v[9], (int)v[10],
                      (unsigned long)row.sequence, (unsigned long)row.dropped,
                      (unsigned long)row.writeFailures);
    case SERIAL_LOG_DRIVE:
      return snprintf(buffer, capacity,
                      "DRIVE,%lu,%d,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.6f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.3f,%.3f,%.3f,%d,%lu,%lu,%lu\n",
                      (unsigned long)row.timestamp, (int)v[0], v[1], v[2], v[3], v[4], v[5], v[6],
                      v[7], v[8], v[9], v[10], v[11], v[12], v[13], v[14], v[15], v[16],
                      v[17], v[18], v[19], v[20], v[21], v[22], v[23], v[24], v[25], v[26],
                      (int)v[27], (unsigned long)row.sequence, (unsigned long)row.dropped,
                      (unsigned long)row.writeFailures);
  }
  return -1;
}
