export const BRIDGE_URL = "ws://127.0.0.1:9090";

export const APPROVED_TOPICS = Object.freeze([
  Object.freeze({
    name: "/hmmd/rdmap",
    type: "hmmd_interfaces/msg/RangeDopplerMap",
    label: "HMMD range-Doppler map",
    kind: "range-doppler-map",
    reliability: "best_effort",
  }),
  Object.freeze({
    name: "/hmmd/status",
    type: "diagnostic_msgs/msg/DiagnosticArray",
    label: "HMMD status",
    kind: "json",
    reliability: "reliable",
  }),
]);

export const MAP_TOPIC = "/hmmd/rdmap";
export const STATUS_TOPIC = "/hmmd/status";
export const MAP_STALE_AFTER_MS = 1500;
export const STATUS_STALE_AFTER_MS = 3000;
export const DISPLAY_INTERVAL_MS = 100;
