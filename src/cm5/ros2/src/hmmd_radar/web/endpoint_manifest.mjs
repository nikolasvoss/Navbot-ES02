export const BRIDGE_URL = "ws://127.0.0.1:9090";
export const DISPLAY_INTERVAL_MS = 100;

const ROS_NAME = /^\/[A-Za-z][A-Za-z0-9_]*(?:\/[A-Za-z][A-Za-z0-9_]*)*$/;
const ROS_TYPE = /^[A-Za-z][A-Za-z0-9_]*\/(?:msg|srv)\/[A-Za-z][A-Za-z0-9_]*$/;
const jsonObject = (value) => value !== null && typeof value === "object" && !Array.isArray(value);
const integerAtLeast = (value, minimum) => Number.isInteger(value) && value >= minimum;

export function validateEndpointManifest(manifest) {
  if (!manifest || !Array.isArray(manifest.topics) || !Array.isArray(manifest.services)) {
    throw new TypeError("Endpoint manifest must contain topic and service arrays");
  }
  const names = new Set();
  for (const [kind, endpoints] of [["topic", manifest.topics], ["service", manifest.services]]) {
    for (const endpoint of endpoints) {
      if (!endpoint || typeof endpoint !== "object" || !ROS_NAME.test(endpoint.name) || !ROS_TYPE.test(endpoint.type) ||
          (kind === "topic" ? !endpoint.type.includes("/msg/") : !endpoint.type.includes("/srv/")) ||
          typeof endpoint.label !== "string" || !endpoint.label.trim() || names.has(endpoint.name)) {
        throw new TypeError(`Invalid or duplicate ${kind} endpoint`);
      }
      names.add(endpoint.name);
      if (kind === "topic") {
        if (!["subscribe", "publish", "both"].includes(endpoint.direction) ||
            !["reliable", "best_effort"].includes(endpoint.reliability) ||
            !integerAtLeast(endpoint.throttleMs, 0) || typeof endpoint.pinned !== "boolean" ||
            !integerAtLeast(endpoint.staleAfterMs, 1) ||
            (endpoint.direction === "publish" && endpoint.pinned) ||
            ("defaultPayload" in endpoint && !jsonObject(endpoint.defaultPayload))) {
          throw new TypeError(`Invalid topic metadata for ${endpoint.name}`);
        }
      } else if (!integerAtLeast(endpoint.timeoutMs, 1) ||
                 ("defaultPayload" in endpoint && !jsonObject(endpoint.defaultPayload))) {
        throw new TypeError(`Invalid service metadata for ${endpoint.name}`);
      }
    }
  }
  return manifest;
}

export async function loadEndpointManifest(fetchImpl = fetch) {
  const response = await fetchImpl("./endpoint-manifest.json", { cache: "no-store" });
  if (!response.ok) throw new Error(`Endpoint manifest load failed (${response.status})`);
  return validateEndpointManifest(await response.json());
}
