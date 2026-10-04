export const DOPPLER_BINS = 20;
export const RANGE_GATES = 16;
export const VALUE_COUNT = DOPPLER_BINS * RANGE_GATES;
export const UINT32_MAX = 0xffffffff;

export function validateRangeDopplerMap(message) {
  if (!message || typeof message !== "object") {
    throw new TypeError("map message must be an object");
  }
  if (message.doppler_bins !== DOPPLER_BINS || message.range_gates !== RANGE_GATES) {
    throw new RangeError("map dimensions must be 20 by 16");
  }
  if (!Array.isArray(message.amplitude_squared) || message.amplitude_squared.length !== VALUE_COUNT) {
    throw new RangeError("map must contain exactly 320 amplitudes");
  }
  if (message.amplitude_squared.some((value) =>
    !Number.isInteger(value) || value < 0 || value > UINT32_MAX
  )) {
    throw new RangeError("map amplitudes must be uint32 values");
  }
  if (!message.header || !message.header.stamp || !Number.isInteger(message.header.stamp.sec) ||
      !Number.isInteger(message.header.stamp.nanosec) || typeof message.header.frame_id !== "string") {
    throw new TypeError("map header must include stamp and frame_id");
  }
  return message;
}

export function rangeDopplerRows(message, logarithmic = false) {
  validateRangeDopplerMap(message);
  const values = message.amplitude_squared;
  return Array.from({ length: DOPPLER_BINS }, (_, doppler) => {
    const start = doppler * RANGE_GATES;
    const row = values.slice(start, start + RANGE_GATES);
    return logarithmic ? row.map((value) => Math.log1p(value)) : row;
  });
}

export function diagnosticFields(message) {
  const status = message?.status?.[0];
  const values = status?.values;
  if (!Array.isArray(values)) return null;
  return Object.fromEntries(values
    .filter((entry) => typeof entry?.key === "string")
    .map((entry) => [entry.key, String(entry.value ?? "")]));
}

export function formatRosStamp(stamp) {
  if (!stamp || !Number.isInteger(stamp.sec) || !Number.isInteger(stamp.nanosec)) return "unknown";
  return `${stamp.sec}.${String(stamp.nanosec).padStart(9, "0")}`;
}
