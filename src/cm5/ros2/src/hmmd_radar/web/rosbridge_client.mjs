import {
  APPROVED_TOPICS,
  MAP_TOPIC,
  STATUS_TOPIC,
  MAP_STALE_AFTER_MS,
  STATUS_STALE_AFTER_MS,
} from "./topic_registry.mjs";
import { validateRangeDopplerMap } from "./model.mjs";

const INITIAL_RETRY_MS = 500;
const MAX_RETRY_MS = 5000;

export class RosbridgeClient {
  constructor({ url, WebSocketImpl = globalThis.WebSocket, now = () => performance.now(), topics = APPROVED_TOPICS }) {
    if (typeof WebSocketImpl !== "function") throw new TypeError("WebSocket is unavailable");
    this.url = url;
    this.WebSocketImpl = WebSocketImpl;
    this.now = now;
    this.topics = new Map(topics.map((topic) => [topic.name, topic]));
    this.connection = "disconnected";
    this.generation = 0;
    this.retryMs = INITIAL_RETRY_MS;
    this.retryTimer = null;
    this.socket = null;
    this.stopped = true;
    this.listeners = new Set();
    this.selectedTopic = STATUS_TOPIC;
    this.latest = new Map();
    this.topicErrors = new Map();
    this.activeTopics = new Set([MAP_TOPIC, STATUS_TOPIC]);
    this.subscribedGeneration = new Map();
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
    if (socket && socket.readyState < this.WebSocketImpl.CLOSING) socket.close();
    this.#notify();
  }

  selectTopic(name) {
    if (name !== null && !this.topics.has(name)) throw new RangeError(`topic is not approved: ${name}`);
    const oldTopic = this.selectedTopic;
    this.selectedTopic = name;
    if (oldTopic && oldTopic !== MAP_TOPIC && oldTopic !== STATUS_TOPIC) {
      this.activeTopics.delete(oldTopic);
      this.#unsubscribe(oldTopic);
    }
    if (name) {
      this.activeTopics.add(name);
      this.#subscribe(name);
    }
    this.#notify();
  }

  onChange(listener) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  snapshot(now = this.now()) {
    const topics = Object.fromEntries([...this.latest].map(([name, sample]) => [name, {
      message: sample.message,
      receivedAt: sample.receivedAt,
      ageMs: Math.max(0, now - sample.receivedAt),
      freshness: this.#freshness(name, sample, now),
    }]));
    return {
      connection: this.connection,
      generation: this.generation,
      selectedTopic: this.selectedTopic,
      topics,
      errors: Object.fromEntries(this.topicErrors),
    };
  }

  #connect() {
    if (this.stopped) return;
    this.connection = "connecting";
    this.#notify();
    let socket;
    try {
      socket = new this.WebSocketImpl(this.url);
    } catch {
      this.#scheduleRetry();
      return;
    }
    this.socket = socket;
    socket.addEventListener("open", () => {
      if (socket !== this.socket || this.stopped) return;
      this.connection = "connected";
      this.generation += 1;
      this.retryMs = INITIAL_RETRY_MS;
      this.subscribedGeneration.clear();
      for (const name of this.activeTopics) this.#subscribe(name);
      this.#notify();
    });
    socket.addEventListener("message", (event) => {
      if (socket !== this.socket || this.stopped) return;
      this.#receive(event.data);
    });
    socket.addEventListener("error", () => {
      if (socket === this.socket && !this.stopped) this.#notify();
    });
    socket.addEventListener("close", () => {
      if (socket !== this.socket) return;
      this.socket = null;
      this.connection = "disconnected";
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
    this.retryTimer = setTimeout(() => {
      this.retryTimer = null;
      this.#connect();
    }, delay);
  }

  #send(message) {
    if (this.socket?.readyState !== this.WebSocketImpl.OPEN) return false;
    this.socket.send(JSON.stringify(message));
    return true;
  }

  #subscribe(name) {
    const topic = this.topics.get(name);
    if (!topic || this.connection !== "connected" ||
        this.subscribedGeneration.get(name) === this.generation) return;
    const sent = this.#send({
      op: "subscribe",
      id: `hmmd-browser:${name}`,
      topic: topic.name,
      type: topic.type,
      throttle_rate: 100,
      queue_length: 1,
      qos: {
        history: "keep_last",
        depth: 1,
        reliability: topic.reliability,
        durability: "volatile",
      },
    });
    if (sent) this.subscribedGeneration.set(name, this.generation);
  }

  #unsubscribe(name) {
    if (!this.topics.has(name)) return;
    this.#send({ op: "unsubscribe", id: `hmmd-browser:${name}`, topic: name });
    this.subscribedGeneration.delete(name);
  }

  #receive(raw) {
    let envelope;
    try {
      envelope = JSON.parse(raw);
    } catch {
      return;
    }
    const name = envelope?.topic;
    const topic = this.topics.get(name);
    if (envelope?.op !== "publish" || !topic || !this.activeTopics.has(name)) return;
    try {
      if (topic.kind === "range-doppler-map") validateRangeDopplerMap(envelope.msg);
      this.latest.set(name, {
        message: envelope.msg,
        receivedAt: this.now(),
        generation: this.generation,
      });
      this.topicErrors.delete(name);
    } catch (error) {
      this.topicErrors.set(name, error.message);
    }
    this.#notify();
  }

  #freshness(name, sample, now) {
    if (this.connection !== "connected") return "bridge-disconnected";
    if (sample.generation !== this.generation) return "waiting-after-reconnect";
    const timeout = name === MAP_TOPIC ? MAP_STALE_AFTER_MS : STATUS_STALE_AFTER_MS;
    return Math.max(0, now - sample.receivedAt) <= timeout ? "fresh" : "stale";
  }

  #notify() {
    const snapshot = this.snapshot();
    for (const listener of this.listeners) listener(snapshot);
  }
}
