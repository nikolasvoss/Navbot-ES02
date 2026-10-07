#pragma once
#include <stddef.h>
#include <stdint.h>
class Print {
 public:
  virtual ~Print() = default;
  virtual size_t write(const uint8_t *, size_t size) { return size; }
};
extern Print Serial;
