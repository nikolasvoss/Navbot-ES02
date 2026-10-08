import { BRIDGE_URL, DISPLAY_INTERVAL_MS, loadEndpointManifest } from "./endpoint_manifest.mjs";
import { diagnosticFields, formatRosStamp, rangeDopplerRows, RANGE_GATES, validateRangeDopplerMap } from "./model.mjs";
import { RADAR_SETTINGS, RadarSettings } from "./radar_settings.mjs";
import { RosbridgeClient } from "./rosbridge_client.mjs";

const ui = {
  bridge: document.querySelector("#bridge-state"),
  sensor: document.querySelector("#sensor-state"),
  map: document.querySelector("#heatmap"),
  mapMeta: document.querySelector("#map-meta"),
  scale: document.querySelector("#scale-select"),
  maximumMode: document.querySelector("#maximum-mode"),
  fixedMaximum: document.querySelector("#fixed-maximum"),
  maximumHint: document.querySelector("#maximum-hint"),
  topic: document.querySelector("#topic-select"),
  topicName: document.querySelector("#topic-name"),
  topicAge: document.querySelector("#topic-age"),
  topicMessage: document.querySelector("#topic-message"),
  channelForm: document.querySelector("#channel-form"),
  endpoint: document.querySelector("#channel-endpoint"),
  payload: document.querySelector("#channel-payload"),
  channelResult: document.querySelector("#channel-result"),
  settingsState: document.querySelector("#radar-settings-state"),
  settingsRefresh: document.querySelector("#radar-settings-refresh"),
  settingsForm: document.querySelector("#radar-settings-form"),
};
let manifestError = "";
let manifest;
try { manifest = await loadEndpointManifest(); }
catch (error) { manifestError = error.message; manifest = { topics: [], services: [] }; }
const MAP_TOPIC = "/hmmd/rdmap";
const STATUS_TOPIC = "/hmmd/status";
const METERS_PER_RANGE_GATE = 0.7;
const topicByName = new Map(manifest.topics.map((topic) => [topic.name, topic]));
const mapTopic = topicByName.get(MAP_TOPIC);
const statusTopic = topicByName.get(STATUS_TOPIC);
const endpointChoices = [
  ...manifest.topics.filter((topic) => ["publish", "both"].includes(topic.direction)).map((topic) => ({ ...topic, operation: "publish" })),
  ...manifest.services.map((service) => ({ ...service, operation: "service" })),
];
for (const topic of manifest.topics.filter((entry) => ["subscribe", "both"].includes(entry.direction))) {
  const option = document.createElement("option");
  option.value = topic.name;
  option.textContent = topic.label;
  ui.topic.append(option);
}
for (const endpoint of endpointChoices) {
  const option = document.createElement("option");
  option.value = endpoint.name;
  option.textContent = `${endpoint.label} (${endpoint.operation})`;
  ui.endpoint.append(option);
}
const client = new RosbridgeClient({
  url: BRIDGE_URL,
  topics: manifest.topics,
  services: manifest.services,
  validators: { "hmmd_interfaces/msg/RangeDopplerMap": validateRangeDopplerMap },
});
const radarSettings = new RadarSettings(client);
const settingElements = {
  maximumDistanceGate: {
    input: document.querySelector("#maximum-distance-gate"),
    confirmed: document.querySelector("#maximum-distance-gate-confirmed"),
    state: document.querySelector("#maximum-distance-gate-state"),
  },
};
let selectedTopic = statusTopic?.name || null;
if (selectedTopic) ui.topic.value = selectedTopic;
let state = client.snapshot();
let logarithmic = false;
let fixedMaximum = Number(ui.fixedMaximum.value);
let fixedMaximumInitialized = false;

client.onChange((next) => { state = next; });
ui.topic.addEventListener("change", () => {
  if (selectedTopic && !topicByName.get(selectedTopic)?.pinned) client.unsubscribe(selectedTopic);
  selectedTopic = ui.topic.value;
  client.subscribe(selectedTopic);
});
ui.endpoint.addEventListener("change", () => {
  const endpoint = endpointChoices.find((entry) => entry.name === ui.endpoint.value);
  ui.payload.value = JSON.stringify(endpoint?.defaultPayload || {}, null, 2);
});
ui.channelForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const endpoint = endpointChoices.find((entry) => entry.name === ui.endpoint.value);
  if (!endpoint) return;
  try {
    const payload = JSON.parse(ui.payload.value);
    if (endpoint.operation === "publish") {
      client.publish(endpoint.name, payload);
      ui.channelResult.textContent = `Submitted to rosbridge. Delivery is not acknowledged.`;
    } else {
      ui.channelResult.textContent = "Request pending…";
      const values = await client.callService(endpoint.name, payload);
      ui.channelResult.textContent = JSON.stringify(values, null, 2);
    }
  } catch (error) {
    ui.channelResult.textContent = error.message;
  }
});
ui.scale.addEventListener("change", () => {
  logarithmic = ui.scale.value === "log1p";
  render();
});
ui.maximumMode.addEventListener("change", () => {
  const fixed = ui.maximumMode.value === "fixed";
  ui.fixedMaximum.disabled = !fixed;
  render();
});
ui.fixedMaximum.addEventListener("input", () => {
  const value = ui.fixedMaximum.valueAsNumber;
  const valid = Number.isFinite(value) && value > 0;
  ui.fixedMaximum.setCustomValidity(valid ? "" : "Enter a finite number greater than zero.");
  ui.fixedMaximum.setAttribute("aria-invalid", String(!valid));
  if (valid) {
    fixedMaximum = value;
    fixedMaximumInitialized = true;
  }
  render();
});
for (const [setting, elements] of Object.entries(settingElements)) {
  elements.input.addEventListener("input", () => radarSettings.setDraft(setting, elements.input.value));
}
ui.settingsForm.addEventListener("submit", async (event) => {
  event.preventDefault();
  const setting = event.submitter?.dataset.setting;
  if (!setting || !RADAR_SETTINGS[setting]) return;
  const input = settingElements[setting].input;
  input.setCustomValidity("");
  if (!input.validity.valid) input.setCustomValidity("Enter an integer in the allowed range.");
  if (!input.reportValidity()) return;
  try { await radarSettings.set(setting, input.valueAsNumber); }
  catch {}
});
ui.settingsRefresh.addEventListener("click", async () => {
  try { await radarSettings.read(); }
  catch {}
});
radarSettings.onChange(renderRadarSettings);

function setBadge(element, label, tone = "muted") {
  element.textContent = label;
  element.className = `badge badge-${tone}`;
}

function renderRadarSettings(settings) {
  const labels = {
    confirmed: ["Values confirmed", "ok"],
    pending: ["Request pending", "warn"],
    stale: ["Last values may be stale", "warn"],
    reconnect: ["Waiting for connection", "warn"],
    error: ["Could not read settings", "error"],
  };
  const label = labels[settings.state] || labels.reconnect;
  setBadge(ui.settingsState, label[0], label[1]);
  ui.settingsState.title = settings.state === "confirmed" ? "" : settings.detail;
  ui.settingsRefresh.disabled = settings.readPending || settings.writePending || state.connection !== "connected";
  for (const [setting, elements] of Object.entries(settingElements)) {
    const confirmed = settings.confirmed?.[setting];
    const info = RADAR_SETTINGS[setting];
    elements.confirmed.textContent = confirmed === undefined ? "Device value: not read" : `Device value: ${confirmed} ${info.unit}`;
    if (document.activeElement !== elements.input && settings.drafts[setting] !== "") elements.input.value = settings.drafts[setting];
    elements.input.disabled = settings.state !== "confirmed" || settings.readPending || settings.writePending || state.connection !== "connected" || confirmed === undefined;
    const apply = ui.settingsForm.querySelector(`button[data-setting="${setting}"]`);
    apply.disabled = elements.input.disabled;
    const write = settings.lastWrite;
    const writeAppliesToSetting = write?.setting === setting;
    const writeState = writeAppliesToSetting
      ? `Write ACK ${write.writeAcknowledged ? "yes" : "no"} · readback ${write.readbackMatched ? "matched" : "not confirmed"} · save ACK ${write.saveAcknowledged ? "yes" : "no"}. `
      : "";
    if (settings.state === "confirmed") {
      elements.state.textContent = writeState;
    } else if (write?.setting) {
      elements.state.textContent = writeAppliesToSetting ? `${writeState}${settings.detail}` : "";
    } else {
      elements.state.textContent = settings.detail;
    }
    elements.state.hidden = elements.state.textContent === "";
    elements.state.className = `setting-state ${settings.state === "error" ? "badge-error" : settings.state === "confirmed" ? "badge-ok" : "muted"}`;
  }
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
  if (statusSample.freshness === "stale" || (statusTopic && now - statusSample.receivedAt > statusTopic.staleAfterMs)) {
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

function drawHeatmap(sample, confirmedMaximumDistanceGate) {
  const canvas = ui.map;
  const context = canvas.getContext("2d");
  const width = 900;
  const height = 560;
  const left = 54;
  const top = 20;
  const right = 140;
  const bottom = 54;
  const plotWidth = width - left - right;
  const plotHeight = height - top - bottom;
  const rangeGateCount = Number.isInteger(confirmedMaximumDistanceGate)
    ? Math.max(0, Math.min(RANGE_GATES, confirmedMaximumDistanceGate))
    : RANGE_GATES;
  const cellWidth = plotWidth / Math.max(1, rangeGateCount);
  const cellHeight = plotHeight / 20;

  context.clearRect(0, 0, width, height);
  context.fillStyle = "#111821";
  context.fillRect(0, 0, width, height);
  context.font = "12px system-ui, sans-serif";
  context.textAlign = "center";
  context.textBaseline = "middle";
  context.fillStyle = "#aab7c8";

  let values = Array.from({ length: 20 }, () => Array(RANGE_GATES).fill(0));
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
  const fixed = ui.maximumMode.value === "fixed";
  if (fixed) {
    if (!fixedMaximumInitialized && sample && ui.fixedMaximum.validity.valid) {
      fixedMaximum = Math.max(1, ...sample.message.amplitude_squared);
      ui.fixedMaximum.value = String(fixedMaximum);
      fixedMaximumInitialized = true;
    }
    low = 0;
    high = logarithmic ? Math.log1p(fixedMaximum) : fixedMaximum;
  }
  const colorScaleHelp = !fixed
    ? "Dynamic: color limits follow each frame."
    : ui.fixedMaximum.validity.valid
      ? "Fixed: scale starts at zero; values above the maximum use the brightest color."
      : `Enter a number greater than zero. Using the last valid maximum: ${fixedMaximum}.`;
  ui.maximumHint.dataset.tooltip = colorScaleHelp;
  ui.maximumHint.setAttribute("aria-label", colorScaleHelp);

  for (let doppler = 0; doppler < 20; doppler += 1) {
    const y = top + (19 - doppler) * cellHeight;
    for (let range = 0; range < rangeGateCount; range += 1) {
      const x = left + range * cellWidth;
      context.fillStyle = rgb(values[doppler][range], low, high);
      context.fillRect(x, y, cellWidth, cellHeight);
      context.strokeStyle = "#10151d";
      context.lineWidth = 1;
      context.strokeRect(x, y, cellWidth, cellHeight);
    }
  }

  context.fillStyle = "#b7c3d3";
  for (let range = 0; range < rangeGateCount; range += 1) {
    const distanceMeters = ((range + 1) * METERS_PER_RANGE_GATE).toFixed(1);
    context.fillText(distanceMeters, left + (range + 0.5) * cellWidth, top + plotHeight + 19);
  }
  for (let doppler = 0; doppler < 20; doppler += 1) {
    context.fillText(String(doppler), left - 23, top + (19 - doppler + 0.5) * cellHeight);
  }
  context.fillStyle = "#d5deea";
  context.font = "13px system-ui, sans-serif";
  context.fillText("Approx. distance (m)", left + plotWidth / 2, height - 12);
  context.save();
  context.translate(13, top + plotHeight / 2);
  context.rotate(-Math.PI / 2);
  context.fillText("Doppler-bin index", 0, 0);
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
  const colorScaleLabel = logarithmic ? "log1p(amplitude squared)" : "Amplitude squared";
  const colorScaleLabelLines = logarithmic ? ["log1p", "(amplitude squared)"] : ["Amplitude", "squared"];
  context.save();
  context.textAlign = "center";
  context.textBaseline = "top";
  context.fillText(colorScaleLabelLines[0], barX + 43, barY + barHeight + 8);
  context.fillText(colorScaleLabelLines[1], barX + 43, barY + barHeight + 21);
  context.restore();

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

  canvas.setAttribute("aria-label", `20 by ${rangeGateCount} HMMD heatmap, ${freshness} data, color scale ${colorScaleLabel}`);
}

function render() {
  const now = performance.now();
  state = client.snapshot(now);
  const radarConfiguration = radarSettings.snapshot();
  renderRadarSettings(radarConfiguration);
  const bridge = bridgeLabel(state.connection);
  setBadge(ui.bridge, bridge[0], bridge[1]);

  const mapSample = state.topics[MAP_TOPIC];
  const statusSample = state.topics[STATUS_TOPIC];
  const sensor = sensorLabel(statusSample, now);
  setBadge(ui.sensor, sensor[0], sensor[1]);

  if (mapSample) {
    const stamp = formatRosStamp(mapSample.message.header.stamp);
    ui.mapMeta.textContent = `Frame ${mapSample.message.header.frame_id || "(no frame id)"} · ROS stamp ${stamp} · received ${formatAge(mapSample.ageMs)}`;
  } else {
    ui.mapMeta.textContent = "Waiting for the first HMMD frame.";
  }
  if (state.errors[MAP_TOPIC]) ui.mapMeta.textContent = `Rejected map: ${state.errors[MAP_TOPIC]}`;
  drawHeatmap(mapSample, radarConfiguration.confirmed?.maximumDistanceGate);

  const selectedName = selectedTopic;
  const selected = selectedName ? state.topics[selectedName] : null;
  ui.topicName.textContent = selectedName || "No topic selected";
  if (selected) {
    ui.topicAge.textContent = `${selected.freshness} · ${formatAge(selected.ageMs)}`;
    ui.topicMessage.textContent = JSON.stringify(selected.message, null, 2);
  } else {
    ui.topicAge.textContent = "Waiting for message";
    ui.topicMessage.textContent = "No message received.";
  }
  const endpointState = state.endpoints[ui.endpoint.value];
  if (endpointState && endpointState.state !== "pending") ui.channelResult.textContent = endpointState.detail;
}

if (!mapTopic || !statusTopic) {
  manifestError ||= "Endpoint manifest must declare HMMD map and status receive topics.";
  ui.channelResult.textContent = manifestError;
}
if (manifestError) ui.channelResult.textContent = `Manifest error: ${manifestError}`;
ui.endpoint.value = endpointChoices[0]?.name || "";
ui.endpoint.dispatchEvent(new Event("change"));
if (selectedTopic) client.subscribe(selectedTopic);
setInterval(render, DISPLAY_INTERVAL_MS);
if (!manifestError) client.start();
renderRadarSettings(radarSettings.snapshot());
render();
