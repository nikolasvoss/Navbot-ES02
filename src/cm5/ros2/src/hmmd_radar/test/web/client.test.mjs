import assert from "node:assert/strict";
import { setTimeout as delay } from "node:timers/promises";
import test from "node:test";

import { diagnosticFields, formatRosStamp, rangeDopplerRows, validateRangeDopplerMap } from "../../web/model.mjs";
import { validateEndpointManifest } from "../../web/endpoint_manifest.mjs";
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

const MAP_TOPIC = "/hmmd/rdmap";
const STATUS_TOPIC = "/hmmd/status";
const RANGE_TOPIC = "/demo/range";
const topics = [
  { name: MAP_TOPIC, type: "hmmd_interfaces/msg/RangeDopplerMap", label: "Map", direction: "subscribe", reliability: "best_effort", throttleMs: 100, staleAfterMs: 1500, pinned: true },
  { name: STATUS_TOPIC, type: "diagnostic_msgs/msg/DiagnosticArray", label: "Status", direction: "subscribe", reliability: "reliable", throttleMs: 100, staleAfterMs: 3000, pinned: true },
  { name: RANGE_TOPIC, type: "sensor_msgs/msg/Range", label: "Range", direction: "both", reliability: "reliable", throttleMs: 50, staleAfterMs: 1000, pinned: false },
  { name: "/demo/write", type: "std_msgs/msg/String", label: "Write", direction: "publish", reliability: "reliable", throttleMs: 0, staleAfterMs: 1, pinned: false },
];
const services = [{ name: "/demo/echo", type: "example_interfaces/srv/SetBool", label: "Echo", timeoutMs: 100, defaultPayload: { data: true } }];
const makeClient = (extra = {}) => new RosbridgeClient({
  url: "ws://127.0.0.1:9090", topics, services, WebSocketImpl: FakeWebSocket,
  validators: { "hmmd_interfaces/msg/RangeDopplerMap": validateRangeDopplerMap }, ...extra,
});

test("validates endpoint kind, integer timing, and publish-only declarations", () => {
  const valid = { topics, services };
  assert.equal(validateEndpointManifest(valid), valid);
  assert.throws(() => validateEndpointManifest({ topics: [{ ...topics[0], type: "pkg/srv/S" }], services: [] }), /Invalid or duplicate topic/);
  assert.throws(() => validateEndpointManifest({ topics: [{ ...topics[3], throttleMs: 1.5 }], services: [] }), /Invalid topic metadata/);
  assert.throws(() => validateEndpointManifest({ topics: [{ ...topics[3], pinned: true }], services: [] }), /Invalid topic metadata/);
});

test("subscribes declared inputs with configured throttle and freshness; validators stay injectable", () => {
  FakeWebSocket.instances = [];
  let now = 100;
  const client = makeClient({ now: () => now });
  client.start();
  const socket = FakeWebSocket.instances[0];
  socket.open();
  const subscriptions = socket.sent.filter((item) => item.op === "subscribe");
  assert.deepEqual(subscriptions.map((item) => item.topic).sort(), [MAP_TOPIC, STATUS_TOPIC].sort());
  assert.equal(subscriptions.find((item) => item.topic === MAP_TOPIC).throttle_rate, 100);
  client.subscribe(RANGE_TOPIC);
  assert.ok(socket.sent.some((item) => item.op === "subscribe" && item.topic === RANGE_TOPIC && item.throttle_rate === 50));
  socket.publish(MAP_TOPIC, makeMap());
  now += 1501;
  assert.equal(client.snapshot(now).topics[MAP_TOPIC].freshness, "stale");
  const previous = client.snapshot(now).topics[MAP_TOPIC].message;
  socket.publish(MAP_TOPIC, { ...makeMap(), doppler_bins: 21 });
  assert.equal(client.snapshot(now).topics[MAP_TOPIC].message, previous);
  assert.match(client.snapshot(now).errors[MAP_TOPIC], /dimensions/);
  assert.throws(() => client.subscribe("/not/declared"), /not subscribable/);
  client.stop();
});

test("publishes declared JSON once per connection and never replays after reconnect", async () => {
  FakeWebSocket.instances = [];
  const client = makeClient();
  client.start();
  const first = FakeWebSocket.instances[0];
  first.open();
  client.publish(RANGE_TOPIC, { range: 1.25 });
  const outgoing = first.sent.filter((item) => ["advertise", "publish"].includes(item.op));
  assert.deepEqual(outgoing.map((item) => item.op), ["advertise", "publish"]);
  assert.equal(outgoing[0].qos.reliability, "reliable");
  assert.equal(client.snapshot().endpoints[RANGE_TOPIC].state, "submitted");
  assert.throws(() => client.publish("/unknown", {}), /not publishable/);
  first.close();
  assert.throws(() => client.callService("/demo/echo", { data: true }), /disconnected/);
  await delay(550);
  const second = FakeWebSocket.instances[1];
  second.open();
  assert.equal(second.sent.some((item) => item.op === "publish" || item.op === "call_service"), false);
  client.stop();
});

test("correlates service responses and rejects malformed, refused, timed out, stopped, and disconnected calls", async () => {
  FakeWebSocket.instances = [];
  const client = makeClient();
  client.start();
  const socket = FakeWebSocket.instances[0];
  socket.open();
  const success = client.callService("/demo/echo", { data: true });
  const request = socket.sent.find((item) => item.op === "call_service");
  assert.equal("type" in request, false);
  socket.emit("message", new MessageEvent("message", { data: JSON.stringify({ op: "service_response", id: request.id, service: request.service, result: true, values: { success: true } }) }));
  assert.deepEqual(await success, { success: true });

  const rejected = client.callService("/demo/echo", {});
  const second = socket.sent.filter((item) => item.op === "call_service").at(-1);
  socket.emit("message", new MessageEvent("message", { data: JSON.stringify({ op: "service_response", id: second.id, service: second.service, result: false, values: {} }) }));
  await assert.rejects(rejected, /Service rejected/);

  const malformed = client.callService("/demo/echo", {});
  const third = socket.sent.filter((item) => item.op === "call_service").at(-1);
  socket.emit("message", new MessageEvent("message", { data: JSON.stringify({ op: "service_response", id: third.id, service: third.service, result: true, values: [] }) }));
  await assert.rejects(malformed, /Service rejected/);

  const wrongService = client.callService("/demo/echo", {});
  const fourth = socket.sent.filter((item) => item.op === "call_service").at(-1);
  socket.emit("message", new MessageEvent("message", { data: JSON.stringify({ op: "service_response", id: fourth.id, service: "/demo/wrong", result: true, values: {} }) }));
  await assert.rejects(wrongService, /Unexpected service response/);

  const timed = client.callService("/demo/echo", {});
  await assert.rejects(timed, /timed out/);

  const stopped = client.callService("/demo/echo", {});
  client.stop();
  await assert.rejects(stopped, /Connection stopped/);
});

test("rejects non-JSON values in outgoing payloads", () => {
  FakeWebSocket.instances = [];
  const client = makeClient();
  client.start();
  const socket = FakeWebSocket.instances[0];
  socket.open();
  assert.throws(() => client.publish(RANGE_TOPIC, { range: Number.NaN }), /JSON values/);
  const circular = {};
  circular.self = circular;
  assert.throws(() => client.publish(RANGE_TOPIC, circular), /JSON values/);
  assert.throws(() => client.callService("/demo/echo", { value: undefined }), /JSON values/);
  client.stop();
});

test("rejects pending services on disconnect and routes bridge status errors", async () => {
  FakeWebSocket.instances = [];
  const client = makeClient();
  client.start();
  const socket = FakeWebSocket.instances[0];
  socket.open();
  const pending = client.callService("/demo/echo", {});
  const request = socket.sent.find((item) => item.op === "call_service");
  socket.emit("message", new MessageEvent("message", { data: JSON.stringify({ op: "status", id: request.id, level: "error", msg: "service unavailable" }) }));
  await assert.rejects(pending, /service unavailable/);
  const disconnected = client.callService("/demo/echo", {});
  socket.close();
  await assert.rejects(disconnected, /Bridge disconnected/);
  client.stop();
});
