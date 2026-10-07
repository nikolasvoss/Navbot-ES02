#pragma once

#include <stdint.h>
#include <stdio.h>

enum SerialLogKind : uint8_t {
  SERIAL_LOG_DIAGNOSTIC,
  SERIAL_LOG_TRACE,
  SERIAL_LOG_CONTROL,
  SERIAL_LOG_BALANCE,
  SERIAL_LOG_DRIVE
};

struct SerialLogRecord {
  SerialLogKind kind;
  uint32_t timestamp;
  uint32_t sequence;
  uint32_t dropped;
  uint32_t writeFailures;
  float values[29];
};

static inline int formatSerialLogRecord(const SerialLogRecord &row, char *buffer, size_t capacity) {
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
