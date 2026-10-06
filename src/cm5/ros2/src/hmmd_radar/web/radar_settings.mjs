export const RADAR_SETTINGS = Object.freeze({
  maximumDistanceGate: Object.freeze({
    label: "Maximum distance gate",
    minimum: 0,
    maximum: 15,
    unit: "gates",
    help: "Limits the farthest distance gate used for detections.",
    selector: 0,
    responseField: "maximum_distance_gate",
  }),
  targetDisappearanceDelaySeconds: Object.freeze({
    label: "Target disappearance delay",
    minimum: 0,
    maximum: 65535,
    unit: "seconds",
    help: "How long a target remains reported after it disappears.",
    selector: 1,
    responseField: "target_disappearance_delay_seconds",
  }),
});

const READ_SERVICE = "/hmmd_sensor/get_radar_config";
const WRITE_SERVICE = "/hmmd_sensor/set_radar_setting";

function settingInfo(setting) {
  const info = RADAR_SETTINGS[setting];
  if (!info) throw new RangeError(`Unknown radar setting: ${setting}`);
  return info;
}

export function validateRadarSettingValue(setting, value) {
  const info = settingInfo(setting);
  if (typeof value !== "number" || !Number.isSafeInteger(value)) {
    throw new TypeError(`${info.label} must be an integer.`);
  }
  if (value < info.minimum || value > info.maximum) {
    throw new RangeError(`${info.label} must be ${info.minimum}–${info.maximum} ${info.unit}.`);
  }
  return value;
}

function confirmedValues(response) {
  for (const [setting, info] of Object.entries(RADAR_SETTINGS)) {
    validateRadarSettingValue(setting, response[info.responseField]);
  }
  return Object.fromEntries(Object.entries(RADAR_SETTINGS).map(([setting, info]) => [setting, response[info.responseField]]));
}

function responseBoolean(response, field) {
  if (typeof response[field] !== "boolean") throw new TypeError(`Radar service field ${field} must be boolean.`);
  return response[field];
}

export class RadarSettings {
  constructor(client) {
    this.client = client;
    this.listeners = new Set();
    this.confirmed = null;
    this.drafts = Object.fromEntries(Object.keys(RADAR_SETTINGS).map((setting) => [setting, ""]));
    this.state = "reconnect";
    this.detail = "Waiting for the radar service connection.";
    this.lastWrite = null;
    this.readPending = false;
    this.writePending = false;
    this.previousConnection = client.snapshot().connection;
    this.unsubscribe = client.onChange((snapshot) => this.#connectionChanged(snapshot.connection));
  }

  snapshot() {
    return {
      state: this.state,
      detail: this.detail,
      confirmed: this.confirmed && { ...this.confirmed },
      drafts: { ...this.drafts },
      readPending: this.readPending,
      writePending: this.writePending,
      lastWrite: this.lastWrite && { ...this.lastWrite },
    };
  }

  onChange(listener) {
    this.listeners.add(listener);
    return () => this.listeners.delete(listener);
  }

  setDraft(setting, value) {
    settingInfo(setting);
    this.drafts[setting] = String(value);
    this.#notify();
  }

  async read() {
    if (this.readPending || this.writePending) throw new Error("A radar settings request is already pending.");
    this.readPending = true;
    this.lastWrite = null;
    this.state = "pending";
    this.detail = "Reading confirmed values from the radar.";
    this.#notify();
    try {
      const response = await this.client.callService(READ_SERVICE, {});
      if (responseBoolean(response, "success") !== true || response.outcome !== 0) throw new Error(response.detail || "Radar configuration read failed.");
      const values = confirmedValues(response);
      if (!this.confirmed) this.drafts = Object.fromEntries(Object.entries(values).map(([setting, value]) => [setting, String(value)]));
      this.confirmed = values;
      this.state = "confirmed";
      this.detail = response.detail || "Both values were read from the radar.";
      return this.snapshot();
    } catch (error) {
      this.state = this.confirmed ? "stale" : "error";
      this.detail = error.message;
      throw error;
    } finally {
      this.readPending = false;
      this.#notify();
    }
  }

  async set(setting, value) {
    const info = settingInfo(setting);
    const parsed = validateRadarSettingValue(setting, value);
    if (this.readPending || this.writePending) throw new Error("A radar settings request is already pending.");
    if (!this.confirmed || this.state === "stale" || this.state === "reconnect") {
      throw new Error("Read radar settings before writing.");
    }
    this.writePending = true;
    this.lastWrite = null;
    this.state = "pending";
    this.detail = `Writing ${info.label.toLowerCase()} and waiting for device confirmation.`;
    this.#notify();
    try {
      const response = await this.client.callService(WRITE_SERVICE, { setting: info.selector, value: parsed });
      const responseFlags = {
        writeAcknowledged: responseBoolean(response, "write_acknowledged"),
        hasObservedValue: responseBoolean(response, "has_observed_value"),
        readbackMatched: responseBoolean(response, "readback_matched"),
        saveAcknowledged: responseBoolean(response, "save_acknowledged"),
      };
      this.lastWrite = {
        setting,
        writeAcknowledged: responseFlags.writeAcknowledged,
        readbackMatched: responseFlags.readbackMatched,
        saveAcknowledged: responseFlags.saveAcknowledged,
      };
      if (responseFlags.hasObservedValue) {
        const observed = validateRadarSettingValue(setting, response.observed_value);
        if (responseFlags.readbackMatched) {
          this.confirmed = { ...this.confirmed, [setting]: observed };
        }
      }
      if (responseBoolean(response, "success") !== true || response.outcome !== 0) {
        throw new Error(response.detail || "Radar setting was not confirmed.");
      }
      if (!responseFlags.writeAcknowledged || !responseFlags.hasObservedValue || !responseFlags.readbackMatched || !responseFlags.saveAcknowledged || this.confirmed[setting] !== parsed) {
        throw new Error("The response did not confirm the requested value through readback and save ACK.");
      }
      this.drafts[setting] = String(this.confirmed[setting]);
      this.state = "confirmed";
      this.detail = response.detail || "Readback matched and the radar acknowledged save.";
      return this.snapshot();
    } catch (error) {
      this.state = this.confirmed ? "stale" : "error";
      this.detail = error.message;
      throw error;
    } finally {
      this.writePending = false;
      this.#notify();
    }
  }

  #connectionChanged(connection) {
    if (connection === this.previousConnection) return;
    this.previousConnection = connection;
    if (connection !== "connected") {
      this.state = this.confirmed ? "stale" : "reconnect";
      this.detail = this.confirmed ? "Connection lost. Last confirmed values are stale." : "Waiting for the radar service connection.";
      this.#notify();
      return;
    }
    this.state = this.confirmed ? "stale" : "reconnect";
    this.detail = "Connected. Refreshing values from the radar.";
    this.#notify();
    this.read().catch(() => {});
  }

  #notify() {
    const value = this.snapshot();
    for (const listener of this.listeners) listener(value);
  }
}
