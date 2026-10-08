const INITIAL_RETRY_MS = 500;
const MAX_RETRY_MS = 5000;

function jsonObject(value) {
  if (!value || typeof value !== "object" || Array.isArray(value)) throw new TypeError("payload must be a JSON object");
  const ancestors = new Set();
  const validate = (item) => {
    if (item === null || typeof item === "string" || typeof item === "boolean") return;
    if (typeof item === "number" && Number.isFinite(item)) return;
    if (Array.isArray(item)) {
      if (ancestors.has(item)) throw new TypeError("payload must contain only JSON values");
      ancestors.add(item);
      for (const entry of item) validate(entry);
      ancestors.delete(item);
      return;
    }
    if (typeof item === "object" && (Object.getPrototypeOf(item) === Object.prototype || Object.getPrototypeOf(item) === null)) {
      if (ancestors.has(item)) throw new TypeError("payload must contain only JSON values");
      ancestors.add(item);
      for (const entry of Object.values(item)) validate(entry);
      ancestors.delete(item);
      return;
    }
    throw new TypeError("payload must contain only JSON values");
  };
  validate(value);
  return JSON.parse(JSON.stringify(value));
}

export class RosbridgeClient {
  constructor({ url, topics, services, validators = {}, WebSocketImpl = globalThis.WebSocket, now = () => performance.now() }) {
    if (typeof WebSocketImpl !== "function") throw new TypeError("WebSocket is unavailable");
    this.url = url;
    this.WebSocketImpl = WebSocketImpl;
    this.now = now;
    this.topics = new Map(topics.map((topic) => [topic.name, topic]));
    this.services = new Map(services.map((service) => [service.name, service]));
    this.validators = validators;
    this.connection = "disconnected";
    this.generation = 0;
    this.retryMs = INITIAL_RETRY_MS;
    this.retryTimer = null;
    this.socket = null;
    this.stopped = true;
    this.listeners = new Set();
    this.latest = new Map();
    this.topicErrors = new Map();
    this.endpointStates = new Map();
    this.activeTopics = new Set(topics.filter((topic) => ["subscribe", "both"].includes(topic.direction) && topic.pinned).map((topic) => topic.name));
    this.subscribedGeneration = new Map();
    this.advertisedGeneration = new Map();
    this.pending = new Map();
    this.publishIds = new Map();
    this.requestSequence = 0;
  }

  start() {
    if (!this.stopped) return;
    this.stopped = false;
    this.#connect();
  }

  stop() {
    this.stopped = true;
    clearTimeout(this.retryTimer);
    this.retryTimer = null;
    this.connection = "disconnected";
    const socket = this.socket;
    this.socket = null;
    this.#rejectPending(new Error("Connection stopped"));
    this.#clearPublishIds();
    if (socket && socket.readyState < this.WebSocketImpl.CLOSING) socket.close();
    this.#notify();
  }

  subscribe(name) {
    const topic = this.topics.get(name);
    if (!topic || !["subscribe", "both"].includes(topic.direction)) throw new RangeError(`topic is not subscribable: ${name}`);
    this.activeTopics.add(name);
    this.#subscribe(name);
  }

  unsubscribe(name) {
    this.activeTopics.delete(name);
    if (this.subscribedGeneration.get(name) === this.generation) this.#send({ op: "unsubscribe", id: `sensor-channel:${name}`, topic: name });
    this.subscribedGeneration.delete(name);
  }

  onChange(listener) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  snapshot(now = this.now()) {
    const topics = Object.fromEntries([...this.latest].map(([name, sample]) => [name, {
      message: sample.message,
      receivedAt: sample.receivedAt,
      generation: sample.generation,
      ageMs: Math.max(0, now - sample.receivedAt),
      freshness: this.#freshness(this.topics.get(name), sample, now),
    }]));
    return { connection: this.connection, generation: this.generation, topics, errors: Object.fromEntries(this.topicErrors), endpoints: Object.fromEntries(this.endpointStates) };
  }

  publish(name, payload) {
    const topic = this.topics.get(name);
    if (!topic || !["publish", "both"].includes(topic.direction)) throw new RangeError(`topic is not publishable: ${name}`);
    const message = jsonObject(payload);
    if (this.connection !== "connected" || this.socket?.readyState !== this.WebSocketImpl.OPEN) throw new Error("Bridge is disconnected");
    if (this.advertisedGeneration.get(name) !== this.generation) {
      this.#sendOrThrow({
        op: "advertise", topic: name, type: topic.type,
        qos: { history: "keep_last", depth: 1, reliability: topic.reliability, durability: "volatile" },
      });
      this.advertisedGeneration.set(name, this.generation);
    }
    const id = `sensor-channel:publish:${this.generation}:${++this.requestSequence}`;
    this.publishIds.set(id, { name, timer: setTimeout(() => this.publishIds.delete(id), 5000) });
    try { this.#sendOrThrow({ op: "publish", id, topic: name, msg: message }); }
    catch (error) { this.#clearPublishId(id); throw error; }
    this.endpointStates.set(name, { state: "submitted", detail: "Submitted to rosbridge. Delivery is not acknowledged." });
    this.#notify();
  }

  callService(name, payload) {
    const service = this.services.get(name);
    if (!service) throw new RangeError(`service is not declared: ${name}`);
    const args = jsonObject(payload);
    if (this.connection !== "connected" || this.socket?.readyState !== this.WebSocketImpl.OPEN) throw new Error("Bridge is disconnected");
    const id = `sensor-channel:${this.generation}:${++this.requestSequence}`;
    return new Promise((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pending.delete(id);
        const error = new Error(`Service timed out: ${name}`);
        this.endpointStates.set(name, { state: "error", detail: error.message });
        reject(error);
        this.#notify();
      }, service.timeoutMs);
      this.pending.set(id, { id, name, resolve, reject, timer });
      try {
        this.#sendOrThrow({ op: "call_service", id, service: name, args, timeout: service.timeoutMs / 1000 });
        this.endpointStates.set(name, { state: "pending", detail: "Waiting for response." });
        this.#notify();
      } catch (error) {
        clearTimeout(timer);
        this.pending.delete(id);
        reject(error);
      }
    });
  }

  #connect() {
    if (this.stopped) return;
    this.connection = "connecting";
    this.#notify();
    let socket;
    try { socket = new this.WebSocketImpl(this.url); } catch { this.#scheduleRetry(); return; }
    this.socket = socket;
    socket.addEventListener("open", () => {
      if (socket !== this.socket || this.stopped) return;
      this.connection = "connected";
      this.generation += 1;
      this.retryMs = INITIAL_RETRY_MS;
      this.subscribedGeneration.clear();
      this.advertisedGeneration.clear();
      this.#clearPublishIds();
      for (const name of this.activeTopics) this.#subscribe(name);
      this.#notify();
    });
    socket.addEventListener("message", (event) => { if (socket === this.socket && !this.stopped) this.#receive(event.data); });
    socket.addEventListener("error", () => { if (socket === this.socket && !this.stopped) this.#notify(); });
    socket.addEventListener("close", () => {
      if (socket !== this.socket) return;
      this.socket = null;
      this.connection = "disconnected";
      this.#rejectPending(new Error("Bridge disconnected"));
      this.#notify();
      this.#scheduleRetry();
    });
  }

  #scheduleRetry() {
    if (this.stopped || this.retryTimer !== null) return;
    const delay = this.retryMs;
    this.retryMs = Math.min(this.retryMs * 2, MAX_RETRY_MS);
    this.connection = "reconnecting";
    this.#notify();
    this.retryTimer = setTimeout(() => { this.retryTimer = null; this.#connect(); }, delay);
  }

  #sendOrThrow(message) {
    if (this.socket?.readyState !== this.WebSocketImpl.OPEN) throw new Error("Bridge is disconnected");
    try { this.socket.send(JSON.stringify(message)); } catch (error) { throw new Error(`Bridge send failed: ${error.message}`); }
  }

  #send(message) {
    try { this.#sendOrThrow(message); return true; } catch { return false; }
  }

  #subscribe(name) {
    const topic = this.topics.get(name);
    if (!topic || this.connection !== "connected" || this.subscribedGeneration.get(name) === this.generation) return;
    if (this.#send({
      op: "subscribe", id: `sensor-channel:${name}`, topic: name, type: topic.type,
      throttle_rate: topic.throttleMs, queue_length: 1,
      qos: { history: "keep_last", depth: 1, reliability: topic.reliability, durability: "volatile" },
    })) this.subscribedGeneration.set(name, this.generation);
  }

  #receive(raw) {
    let envelope;
    try { envelope = JSON.parse(raw); } catch { return; }
    if (envelope?.op === "service_response") return this.#serviceResponse(envelope);
    if (envelope?.op === "status" && envelope.id) return this.#statusError(envelope);
    const name = envelope?.topic;
    const topic = this.topics.get(name);
    if (envelope?.op !== "publish" || !topic || !["subscribe", "both"].includes(topic.direction) || !this.activeTopics.has(name)) return;
    try {
      this.validators[topic.type]?.(envelope.msg);
      this.latest.set(name, { message: envelope.msg, receivedAt: this.now(), generation: this.generation });
      this.topicErrors.delete(name);
    } catch (error) { this.topicErrors.set(name, error.message); }
    this.#notify();
  }

  #serviceResponse(envelope) {
    const pending = this.pending.get(envelope.id);
    if (!pending) return;
    this.pending.delete(envelope.id);
    clearTimeout(pending.timer);
    if (envelope.service !== pending.name || envelope.result !== true || !envelope.values || typeof envelope.values !== "object" || Array.isArray(envelope.values)) {
      const reason = envelope.service !== pending.name ? `Unexpected service response for ${pending.name}` : `Service rejected: ${pending.name}`;
      const error = new Error(envelope.message || reason);
      this.endpointStates.set(pending.name, { state: "error", detail: error.message });
      pending.reject(error);
    } else {
      this.endpointStates.set(pending.name, { state: "response", detail: JSON.stringify(envelope.values), values: envelope.values });
      pending.resolve(envelope.values);
    }
    this.#notify();
  }

  #statusError(envelope) {
    const pending = this.pending.get(envelope.id);
    if (pending) {
      this.pending.delete(envelope.id);
      clearTimeout(pending.timer);
      const error = new Error(envelope.msg || "Bridge rejected the request");
      this.endpointStates.set(pending.name, { state: "error", detail: error.message });
      pending.reject(error);
    } else {
      const name = this.publishIds.get(envelope.id)?.name || [...this.topics.keys()].find((topic) => envelope.id === `sensor-channel:${topic}`);
      this.#clearPublishId(envelope.id);
      if (name) this.topicErrors.set(name, envelope.msg || "Bridge rejected the endpoint");
      this.endpointStates.set(name || envelope.id, { state: "error", detail: envelope.msg || "Bridge rejected the endpoint" });
    }
    this.#notify();
  }

  #rejectPending(error) {
    for (const pending of this.pending.values()) {
      clearTimeout(pending.timer);
      pending.reject(error);
      this.endpointStates.set(pending.name, { state: "error", detail: error.message });
    }
    this.pending.clear();
  }

  #clearPublishId(id) {
    const pending = this.publishIds.get(id);
    if (!pending) return;
    clearTimeout(pending.timer);
    this.publishIds.delete(id);
  }

  #clearPublishIds() {
    for (const id of this.publishIds.keys()) this.#clearPublishId(id);
  }

  #freshness(topic, sample, now) {
    if (this.connection !== "connected") return "bridge-disconnected";
    if (sample.generation !== this.generation) return "waiting-after-reconnect";
    return Math.max(0, now - sample.receivedAt) <= topic.staleAfterMs ? "fresh" : "stale";
  }

  #notify() {
    const snapshot = this.snapshot();
    for (const listener of this.listeners) listener(snapshot);
  }
}
