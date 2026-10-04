import assert from "node:assert/strict";
import { setTimeout as delay } from "node:timers/promises";
import test from "node:test";

import { diagnosticFields, formatRosStamp, rangeDopplerRows, validateRangeDopplerMap } from "../../web/model.mjs";
import { APPROVED_TOPICS, MAP_TOPIC, STATUS_TOPIC } from "../../web/topic_registry.mjs";
import { RosbridgeClient } from "../../web/rosbridge_client.mjs";

function makeMap(values = Array.from({ length: 320 }, (_, index) => index)) {
  return {
    header: { stamp: { sec: 21, nanosec: 123 }, frame_id: "hmmd_sensor" },
    doppler_bins: 20,
    range_gates: 16,
    amplitude_squared: values,
  };
}

function makeStatus(connected, stale) {
  return {
    status: [{
      name: "hmmd_sensor",
      message: "receiving frames",
      values: [
        { key: "connected", value: String(connected) },
        { key: "stale", value: String(stale) },
        { key: "frames_received", value: "7" },
      ],
    }],
  };
}

class FakeWebSocket {
  static CONNECTING = 0;
  static OPEN = 1;
  static CLOSING = 2;
  static CLOSED = 3;
  static instances = [];

  constructor(url) {
    this.url = url;
    this.readyState = FakeWebSocket.CONNECTING;
    this.listeners = new Map();
    this.sent = [];
    FakeWebSocket.instances.push(this);
  }

  addEventListener(name, listener) {
    const listeners = this.listeners.get(name) || [];
    listeners.push(listener);
    this.listeners.set(name, listeners);
  }

  send(value) { this.sent.push(JSON.parse(value)); }

  open() {
    this.readyState = FakeWebSocket.OPEN;
    this.emit("open", new Event("open"));
  }

  publish(topic, message) {
    this.emit("message", new MessageEvent("message", {
      data: JSON.stringify({ op: "publish", topic, msg: message }),
    }));
  }

  close() {
    if (this.readyState >= FakeWebSocket.CLOSING) return;
    this.readyState = FakeWebSocket.CLOSED;
    this.emit("close", new Event("close"));
  }

  emit(name, event) {
    for (const listener of this.listeners.get(name) || []) listener(event);
  }
}

test("validates and reshapes the raw HMMD matrix without changing its values", () => {
  const message = makeMap();
  assert.equal(validateRangeDopplerMap(message), message);
  const rows = rangeDopplerRows(message);
  assert.equal(rows.length, 20);
  assert.equal(rows[0].length, 16);
  assert.equal(rows[0][0], 0);
  assert.equal(rows[1][0], 16);
  assert.equal(rows[19][15], 319);
  assert.equal(message.amplitude_squared[319], 319);
  assert.equal(rangeDopplerRows(message, true)[0][1], Math.log1p(1));
  assert.equal(formatRosStamp(message.header.stamp), "21.000000123");
});

test("rejects invalid dimensions, length, uint32 values, and header", () => {
  assert.throws(() => validateRangeDopplerMap({ ...makeMap(), doppler_bins: 19 }), /dimensions/);
  assert.throws(() => validateRangeDopplerMap({ ...makeMap(), amplitude_squared: [1] }), /320/);
  assert.throws(() => validateRangeDopplerMap(makeMap(Array(320).fill(-1))), /uint32/);
  assert.throws(() => validateRangeDopplerMap({ ...makeMap(), header: null }), /header/);
});

test("reads diagnostic key-values as text", () => {
  assert.deepEqual(diagnosticFields(makeStatus(true, false)), {
    connected: "true",
    stale: "false",
    frames_received: "7",
  });
  assert.equal(diagnosticFields({ status: [] }), null);
});

test("subscribes only approved topics with latest-only rate and matching QoS", () => {
  FakeWebSocket.instances = [];
  let now = 100;
  const client = new RosbridgeClient({
    url: "ws://127.0.0.1:9090",
    WebSocketImpl: FakeWebSocket,
    now: () => now,
  });
  client.start();
  const socket = FakeWebSocket.instances[0];
  socket.open();

  const subscriptions = socket.sent.filter((item) => item.op === "subscribe");
  assert.deepEqual(subscriptions.map((item) => item.topic).sort(), [MAP_TOPIC, STATUS_TOPIC].sort());
  const mapSubscription = subscriptions.find((item) => item.topic === MAP_TOPIC);
  assert.equal(mapSubscription.type, "hmmd_interfaces/msg/RangeDopplerMap");
  assert.equal(mapSubscription.throttle_rate, 100);
  assert.equal(mapSubscription.queue_length, 1);
  assert.deepEqual(mapSubscription.qos, {
    history: "keep_last", depth: 1, reliability: "best_effort", durability: "volatile",
  });
  assert.throws(() => client.selectTopic("/not/approved"), /not approved/);

  socket.publish(MAP_TOPIC, makeMap());
  now += 1499;
  assert.equal(client.snapshot(now).topics[MAP_TOPIC].freshness, "fresh");
  now += 2;
  assert.equal(client.snapshot(now).topics[MAP_TOPIC].freshness, "stale");

  const original = client.snapshot(now).topics[MAP_TOPIC].message;
  socket.publish(MAP_TOPIC, { ...makeMap(), doppler_bins: 21 });
  assert.equal(client.snapshot(now).topics[MAP_TOPIC].message, original);
  assert.match(client.snapshot(now).errors[MAP_TOPIC], /dimensions/);
  client.stop();
});

test("keeps bridge and sensor states separate and resubscribes after reconnect", async () => {
  FakeWebSocket.instances = [];
  let now = 500;
  const client = new RosbridgeClient({
    url: "ws://127.0.0.1:9090",
    WebSocketImpl: FakeWebSocket,
    now: () => now,
  });
  client.start();
  const first = FakeWebSocket.instances[0];
  first.open();
  first.publish(MAP_TOPIC, makeMap());
  first.publish(STATUS_TOPIC, makeStatus(true, true));
  assert.equal(client.snapshot(now).connection, "connected");
  assert.equal(client.snapshot(now).topics[MAP_TOPIC].freshness, "fresh");
  assert.equal(client.snapshot(now).topics[STATUS_TOPIC].message.status[0].values[1].value, "true");

  first.close();
  assert.equal(client.snapshot(now).connection, "reconnecting");
  await delay(550);
  const second = FakeWebSocket.instances[1];
  assert.ok(second);
  second.open();
  assert.equal(client.snapshot(now).topics[MAP_TOPIC].freshness, "waiting-after-reconnect");
  assert.deepEqual(second.sent.filter((item) => item.op === "subscribe").map((item) => item.topic).sort(),
    [MAP_TOPIC, STATUS_TOPIC].sort());
  second.publish(MAP_TOPIC, makeMap());
  assert.equal(client.snapshot(now).topics[MAP_TOPIC].freshness, "fresh");
  client.stop();
});

test("selects another approved topic through the same client", () => {
  FakeWebSocket.instances = [];
  const additional = { name: "/imu/data", type: "sensor_msgs/msg/Imu", label: "IMU", kind: "json", reliability: "reliable" };
  const client = new RosbridgeClient({
    url: "ws://127.0.0.1:9090",
    WebSocketImpl: FakeWebSocket,
    topics: [...APPROVED_TOPICS, additional],
  });
  client.start();
  const socket = FakeWebSocket.instances[0];
  socket.open();
  client.selectTopic("/imu/data");
  assert.ok(socket.sent.some((item) => item.op === "subscribe" && item.topic === "/imu/data"));
  socket.publish("/imu/data", { linear_acceleration: { x: 1 } });
  assert.deepEqual(client.snapshot().topics["/imu/data"].message, { linear_acceleration: { x: 1 } });
  client.stop();
});
