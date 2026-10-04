import {
  APPROVED_TOPICS,
  BRIDGE_URL,
  DISPLAY_INTERVAL_MS,
  MAP_TOPIC,
  MAP_STALE_AFTER_MS,
  STATUS_TOPIC,
  STATUS_STALE_AFTER_MS,
} from "./topic_registry.mjs";
import { diagnosticFields, formatRosStamp, rangeDopplerRows } from "./model.mjs";
import { RosbridgeClient } from "./rosbridge_client.mjs";

const ui = {
  bridge: document.querySelector("#bridge-state"),
  sensor: document.querySelector("#sensor-state"),
  map: document.querySelector("#heatmap"),
  mapState: document.querySelector("#map-state"),
  mapMeta: document.querySelector("#map-meta"),
  mapScale: document.querySelector("#map-scale"),
  scale: document.querySelector("#scale-select"),
  topic: document.querySelector("#topic-select"),
  topicName: document.querySelector("#topic-name"),
  topicAge: document.querySelector("#topic-age"),
  topicMessage: document.querySelector("#topic-message"),
};

for (const topic of APPROVED_TOPICS) {
  const option = document.createElement("option");
  option.value = topic.name;
  option.textContent = topic.label;
  ui.topic.append(option);
}

const client = new RosbridgeClient({ url: BRIDGE_URL });
ui.topic.value = STATUS_TOPIC;
let state = client.snapshot();
let logarithmic = false;

client.onChange((next) => { state = next; });
ui.topic.addEventListener("change", () => client.selectTopic(ui.topic.value));
ui.scale.addEventListener("change", () => {
  logarithmic = ui.scale.value === "log1p";
  render();
});

function setBadge(element, label, tone = "muted") {
  element.textContent = label;
  element.className = `badge badge-${tone}`;
}

function bridgeLabel(connection) {
  if (connection === "connected") return ["Connected", "ok"];
  if (connection === "connecting") return ["Connecting", "warn"];
  if (connection === "reconnecting") return ["Reconnecting", "warn"];
  return ["Disconnected", "error"];
}

function formatAge(ageMs) {
  if (!Number.isFinite(ageMs)) return "unknown age";
  return `${(ageMs / 1000).toFixed(1)} s ago`;
}

function sensorLabel(statusSample, now) {
  if (state.connection !== "connected") return ["Unavailable while bridge is disconnected", "error"];
  if (!statusSample) return ["Waiting for HMMD status", "muted"];
  if (statusSample.freshness === "waiting-after-reconnect") return ["Waiting for fresh status after reconnect", "warn"];
  if (statusSample.freshness === "stale" || now - statusSample.receivedAt > STATUS_STALE_AFTER_MS) {
    return ["HMMD status publisher is silent", "warn"];
  }

  const message = statusSample.message?.status?.[0];
  const fields = diagnosticFields(statusSample.message);
  if (!fields) return [message?.message || "HMMD status has no values", "warn"];
  if (fields.connected !== "true") return ["UART disconnected", "error"];
  if (fields.stale !== "false") return ["UART open, no recent frames", "warn"];
  return ["Receiving sensor frames", "ok"];
}

function rgb(value, low, high) {
  const stops = [
    [8, 28, 65],
    [19, 105, 126],
    [53, 174, 126],
    [244, 211, 92],
  ];
  const t = high > low ? Math.max(0, Math.min(1, (value - low) / (high - low))) : 0.5;
  const scaled = t * (stops.length - 1);
  const index = Math.min(stops.length - 2, Math.floor(scaled));
  const mix = scaled - index;
  const channels = stops[index].map((start, channel) => Math.round(start + (stops[index + 1][channel] - start) * mix));
  return `rgb(${channels.join(",")})`;
}

function drawHeatmap(sample) {
  const canvas = ui.map;
  const context = canvas.getContext("2d");
  const width = 900;
  const height = 560;
  const left = 54;
  const top = 20;
  const right = 105;
  const bottom = 54;
  const plotWidth = width - left - right;
  const plotHeight = height - top - bottom;
  const cellWidth = plotWidth / 16;
  const cellHeight = plotHeight / 20;

  context.clearRect(0, 0, width, height);
  context.fillStyle = "#111821";
  context.fillRect(0, 0, width, height);
  context.font = "12px system-ui, sans-serif";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillStyle = "#aab7c8";

  let values = Array.from({ length: 20 }, () => Array(16).fill(0));
  let freshness = "waiting";
  let low = 0;
  let high = 1;
  if (sample) {
    values = rangeDopplerRows(sample.message, logarithmic);
    freshness = sample.freshness;
    low = Math.min(...values.flat());
    high = Math.max(...values.flat());
    if (high <= low) high = low + 1;
  }

  for (let doppler = 0; doppler < 20; doppler += 1) {
    const y = top + (19 - doppler) * cellHeight;
    for (let range = 0; range < 16; range += 1) {
      const x = left + range * cellWidth;
      context.fillStyle = rgb(values[doppler][range], low, high);
      context.fillRect(x, y, cellWidth, cellHeight);
      context.strokeStyle = "#10151d";
      context.lineWidth = 1;
      context.strokeRect(x, y, cellWidth, cellHeight);
    }
  }

  context.fillStyle = "#b7c3d3";
  for (let range = 0; range < 16; range += 1) {
    context.fillText(String(range), left + (range + 0.5) * cellWidth, top + plotHeight + 19);
  }
  for (let doppler = 0; doppler < 20; doppler += 1) {
    context.fillText(String(doppler), left - 23, top + (19 - doppler + 0.5) * cellHeight);
  }
  context.fillStyle = "#d5deea";
  context.font = "13px system-ui, sans-serif";
  context.fillText("Range gate index", left + plotWidth / 2, height - 12);
  context.save();
  context.translate(13, top + plotHeight / 2);
  context.rotate(-Math.PI / 2);
  context.fillText("Doppler bin index", 0, 0);
  context.restore();

  const barX = left + plotWidth + 30;
  const barY = top + 2;
  const barHeight = plotHeight - 4;
  const gradient = context.createLinearGradient(0, barY + barHeight, 0, barY);
  gradient.addColorStop(0, rgb(low, low, high));
  gradient.addColorStop(0.33, "rgb(19,105,126)");
  gradient.addColorStop(0.66, "rgb(53,174,126)");
  gradient.addColorStop(1, rgb(high, low, high));
  context.fillStyle = gradient;
  context.fillRect(barX, barY, 13, barHeight);
  context.textAlign = "left";
  context.textBaseline = "top";
  context.fillStyle = "#b7c3d3";
  context.font = "11px system-ui, sans-serif";
  context.fillText(high.toPrecision(4), barX + 19, barY);
  context.textBaseline = "bottom";
  context.fillText(low.toPrecision(4), barX + 19, barY + barHeight);

  if (freshness !== "fresh") {
    const labels = {
      waiting: "WAITING FOR FRAME",
      stale: "STALE DATA",
      "bridge-disconnected": "BRIDGE DISCONNECTED",
      "waiting-after-reconnect": "WAITING FOR NEW FRAME",
    };
    const label = labels[freshness] || "WAITING FOR FRAME";
    context.fillStyle = "#081018c9";
    context.fillRect(left + plotWidth * 0.18, top + plotHeight * 0.43, plotWidth * 0.64, 40);
    context.fillStyle = "#fff2bf";
    context.font = "bold 15px system-ui, sans-serif";
    context.textAlign = "center";
    context.textBaseline = "middle";
    context.fillText(label, left + plotWidth / 2, top + plotHeight * 0.43 + 20);
  }

  canvas.setAttribute("aria-label", `20 by 16 HMMD heatmap, ${freshness} data`);
  ui.mapScale.textContent = logarithmic ? "log1p(amplitude squared)" : "Amplitude squared";
}

function render() {
  const now = performance.now();
  state = client.snapshot(now);
  const bridge = bridgeLabel(state.connection);
  setBadge(ui.bridge, bridge[0], bridge[1]);

  const mapSample = state.topics[MAP_TOPIC];
  const statusSample = state.topics[STATUS_TOPIC];
  const sensor = sensorLabel(statusSample, now);
  setBadge(ui.sensor, sensor[0], sensor[1]);

  const mapFreshness = mapSample?.freshness || (state.connection === "connected" ? "waiting" : "bridge-disconnected");
  const mapLabels = {
    fresh: ["Fresh frames", "ok"],
    stale: ["No recent frames", "warn"],
    waiting: ["Waiting for frame", "muted"],
    "bridge-disconnected": ["Bridge disconnected", "error"],
    "waiting-after-reconnect": ["Waiting for a new frame", "warn"],
  };
  const mapBadge = mapLabels[mapFreshness] || mapLabels.waiting;
  setBadge(ui.mapState, mapBadge[0], mapBadge[1]);

  if (mapSample) {
    const stamp = formatRosStamp(mapSample.message.header.stamp);
    ui.mapMeta.textContent = `Frame ${mapSample.message.header.frame_id || "(no frame id)"} · ROS stamp ${stamp} · received ${formatAge(mapSample.ageMs)}`;
  } else {
    ui.mapMeta.textContent = "Waiting for the first HMMD frame.";
  }
  if (state.errors[MAP_TOPIC]) ui.mapMeta.textContent = `Rejected map: ${state.errors[MAP_TOPIC]}`;
  drawHeatmap(mapSample);

  const selectedName = state.selectedTopic;
  const selected = selectedName ? state.topics[selectedName] : null;
  ui.topicName.textContent = selectedName || "No topic selected";
  if (selected) {
    ui.topicAge.textContent = `${selected.freshness} · ${formatAge(selected.ageMs)}`;
    ui.topicMessage.textContent = JSON.stringify(selected.message, null, 2);
  } else {
    ui.topicAge.textContent = "Waiting for message";
    ui.topicMessage.textContent = "No message received.";
  }
}

setInterval(render, DISPLAY_INTERVAL_MS);
client.start();
render();
