#include "Telemetry.h"

#include <assert.h>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include "freertos/queue.h"
#include "freertos/task.h"

Print Serial;

QueueHandle_t xQueueCreateStatic(size_t length, size_t itemSize, uint8_t *storage,
                                 StaticQueue_t *control) {
  if (length == 0 || itemSize == 0 || storage == nullptr || control == nullptr) return nullptr;
  control->storage = storage;
  control->itemSize = itemSize;
  control->capacity = length;
  control->head = control->tail = control->count = 0;
  return control;
}

BaseType_t xQueueSend(QueueHandle_t queue, const void *item, TickType_t wait) {
  if (queue == nullptr || item == nullptr || wait != 0) return pdFAIL;
  std::lock_guard<std::mutex> lock(queue->mutex);
  if (queue->count == queue->capacity) return pdFAIL;
  std::memcpy(queue->storage + queue->tail * queue->itemSize, item, queue->itemSize);
  queue->tail = (queue->tail + 1) % queue->capacity;
  ++queue->count;
  return pdPASS;
}

BaseType_t xQueueReceive(QueueHandle_t queue, void *item, TickType_t wait) {
  if (queue == nullptr || item == nullptr || wait != 0) return pdFAIL;
  std::lock_guard<std::mutex> lock(queue->mutex);
  if (queue->count == 0) return pdFAIL;
  std::memcpy(item, queue->storage + queue->head * queue->itemSize, queue->itemSize);
  queue->head = (queue->head + 1) % queue->capacity;
  --queue->count;
  return pdPASS;
}

BaseType_t xTaskCreatePinnedToCore(TaskFunction_t, const char *, uint32_t, void *, uint32_t,
                                   void *, int32_t) {
  return pdPASS;
}

void vTaskDelay(TickType_t) {}

class StringSink : public Print {
 public:
  size_t write(const uint8_t *p, size_t n) override {
    output.append(reinterpret_cast<const char *>(p), n);
    return n;
  }
  std::string output;
};

class BlockingSink : public Print {
 public:
  size_t write(const uint8_t *p, size_t n) override {
    std::unique_lock<std::mutex> lock(mutex);
    if (!blockedOnce) {
      blockedOnce = true;
      entered = true;
      condition.notify_all();
      condition.wait(lock, [this] { return released; });
    }
    output.append(reinterpret_cast<const char *>(p), n);
    return n;
  }
  bool waitUntilBlocked() {
    std::unique_lock<std::mutex> lock(mutex);
    return condition.wait_for(lock, std::chrono::seconds(2), [this] { return entered; });
  }
  void release() {
    std::lock_guard<std::mutex> lock(mutex);
    released = true;
    condition.notify_all();
  }
  std::string contents() {
    std::lock_guard<std::mutex> lock(mutex);
    return output;
  }
 private:
  std::mutex mutex;
  std::condition_variable condition;
  bool blockedOnce = false;
  bool entered = false;
  bool released = false;
  std::string output;
};

Telemetry::SelectedDebug selected(int selector) {
  Telemetry::SelectedDebug row;
  row.selector = selector;
  return row;
}

void testDiagnosticAndOrdinary() {
  StringSink sink;
  Telemetry::Frame frame;
  for (size_t i = 0; i < Telemetry::kChannelCount; ++i) frame.values[i] = i;
  frame.timestampUs = 1234567;
  assert(Telemetry::enqueueDiagnostic(frame));
  assert(Telemetry::sendOne(sink));
  assert(sink.output == "0.000000,1.000000,2.000000,3.000000,4.000000,5.000000,6.000000,7.000000,8.000000,9.000000,10.000000,11.000000,12.000000,13.000000,14.000000,15.000000,16.000000,17.000000,18.000000,19.000000,20.000000,21.000000,22.000000,23.000000,1.234567\n");
  assert(!Telemetry::sendOne(sink));

  auto row = selected(1);
  row.values[0]=0.125f; row.values[1]=1.25f; row.values[2]=-2.5f; row.values[3]=3.75f;
  assert(Telemetry::enqueueSelected(row));
  assert(Telemetry::sendOne(sink));
  assert(sink.output.substr(sink.output.find("dt:")) == "dt:0.125000 Roll:1.25 Pitch:-2.50 Yaw:3.75\n");
}

void testTraceFormats() {
  const char *expected[] = {
      "TRACE,7,2,3,1.235,2.346,3.46,4.57,10,11,12,13,14,15,16,17,18.250,19.750\n",
      "CTRL,8,4,1.235,2.235,3.23,4.2346,5.2346,6.23,7.23,8.23,9.23,10.23,11.23,12\n",
      "BAL,9,5,1.235,2.23,3.23,4.23,5.23,6.23,7.2346,8.23,9.23,13\n",
      "DRIVE,10,6,1.012,2.012,3.012,4.012,5.012,6.012,7.012340,8.012,9.0123,10.0123,11.0123,12.0123,13.0123,14.0123,15.0123,16.012,17.0123,18.0123,19.0123,20.0123,21.0123,22.0123,23.0123,24.012,25.012,26.012,14\n"};
  for (int selector = 55; selector <= 58; ++selector) {
    StringSink sink;
    auto row = selected(selector);
    row.timestampMs = selector - 48;
    if (selector == 55) {
      row.integers[0]=2; row.integers[1]=3;
      for (int i=2;i<10;++i) row.integers[i]=i+8;
      row.values[0]=1.23456f; row.values[1]=2.34567f; row.values[2]=3.456f; row.values[3]=4.567f;
      row.values[4]=18.25f; row.values[5]=19.75f;
    } else if (selector == 56) {
      row.integers[0]=4; row.integers[1]=12;
      for (int i=0;i<=10;++i) row.values[i]=i+1.23456f;
    } else if (selector == 57) {
      row.integers[0]=5; row.integers[1]=13;
      for (int i=0;i<=8;++i) row.values[i]=i+1.23456f;
    } else {
      row.integers[0]=6; row.integers[1]=14;
      for (int i=0;i<26;++i) row.values[i]=i+1.01234f;
    }
    assert(Telemetry::enqueueSelected(row));
    assert(Telemetry::sendOne(sink));
    if (sink.output != expected[selector-55])
      std::cerr << "trace " << selector << " expected=[" << expected[selector-55]
                << "] actual=[" << sink.output << "]";
    assert(sink.output == expected[selector-55]);
  }
}

void testWrapAndReuse() {
  StringSink sink;
  for (int pass=0;pass<4;++pass) {
    for (uint32_t i=0;i<Telemetry::kQueueCapacity;++i) {
      auto row=selected(36); row.integers[0]=pass; row.integers[1]=i;
      assert(Telemetry::enqueueSelected(row));
    }
    for (uint32_t i=0;i<Telemetry::kQueueCapacity;++i) assert(Telemetry::sendOne(sink));
    assert(!Telemetry::sendOne(sink));
  }
  assert(sink.output.find(" state:0 start:0\n state:0 start:1\n") == 0);
  assert(sink.output.find(" state:3 start:31\n") != std::string::npos);
}

void testBlockedSinkAndOverflow() {
  BlockingSink sink;
  auto first=selected(36); first.integers[0]=9; first.integers[1]=99;
  assert(Telemetry::enqueueSelected(first));
  std::thread consumer([&sink] { assert(Telemetry::sendOne(sink)); });
  const bool blocked=sink.waitUntilBlocked();
  if (!blocked) sink.release();
  assert(blocked);
  for (uint32_t i=0;i<Telemetry::kQueueCapacity;++i) {
    auto row=selected(36); row.integers[0]=10; row.integers[1]=i;
    assert(Telemetry::enqueueSelected(row));
  }
  auto overflow=selected(36);
  assert(!Telemetry::enqueueSelected(overflow));
  assert(Telemetry::incompleteReason()==Telemetry::IncompleteReason::BufferFull);
  assert(Telemetry::rejectedCount()==1);
  assert(!Telemetry::enqueueSelected(overflow));
  assert(Telemetry::rejectedCount()==2);
  sink.release();
  consumer.join();
  for (uint32_t i=0;i<Telemetry::kQueueCapacity;++i) assert(Telemetry::sendOne(sink));
  assert(!Telemetry::sendOne(sink));
  std::string expected=" state:9 start:99\n";
  for (uint32_t i=0;i<Telemetry::kQueueCapacity;++i) expected += " state:10 start:"+std::to_string(i)+"\n";
  assert(sink.contents()==expected);
}

int main() {
  assert(Telemetry::startSender());
  testDiagnosticAndOrdinary();
  testTraceFormats();
  testWrapAndReuse();
  testBlockedSinkAndOverflow();
  std::cout << "serial telemetry host checks passed\n";
}
