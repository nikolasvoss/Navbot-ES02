#include "imu.h"

#include <Arduino.h>
#include <cmath>
#include <cstdlib>

#include "ICM42688.h"
#include "Logging.h"
#include "MahonyFilter.h"
#include "filter.h"

namespace imu {
namespace {

constexpr float kAccelRangeG = 8.0f;
constexpr float kGyroRangeDegreesPerSecond = 2000.0f;
constexpr float kDegreesToRadians = 3.1415926f / 180.0f;
constexpr float kTemperatureScale = 132.48f;
constexpr float kTemperatureOffsetC = 25.0f;
constexpr float kComplementaryGyroWeight = 0.98f;
constexpr float kComplementaryAccelWeight = 0.02f;
constexpr int kGyroCalibrationThreshold = 50;
constexpr int kGyroCalibrationSamples = 1000;
constexpr uint32_t kLongGapThresholdUs = (1500000U / kSampleRateHz);

struct ImuState {
    bool started = false;
    Settings activeSettings{};
    CalibrationValues activeCalibration{};
    Snapshot published{};
    TimingStatistics timing{};
    bool hasPreviousRead = false;
    uint32_t previousReadTimeUs = 0;
    biquadFilter_t gyroFilters[3]{};
    biquadFilter_t accelFilters[3]{};
    MahonyFilter mahony{};
};

ImuState state{};

bool validSettings(const Settings& candidate) {
    const uint16_t nyquistHz = kSampleRateHz / 2;
    return candidate.gyroCutoffHz > 0 && candidate.gyroCutoffHz < nyquistHz &&
           candidate.accelCutoffHz > 0 && candidate.accelCutoffHz < nyquistHz &&
           std::isfinite(candidate.mahonyTwoKp) && candidate.mahonyTwoKp >= 0.0f &&
           std::isfinite(candidate.mahonyTwoKi) && candidate.mahonyTwoKi >= 0.0f;
}

void resetGyroFilters(const Settings& settings) {
    for (biquadFilter_t& filter : state.gyroFilters) {
        biquadFilterInitLPF(&filter, settings.gyroCutoffHz, kSampleRateHz);
    }
}

void resetAccelFilters(const Settings& settings) {
    for (biquadFilter_t& filter : state.accelFilters) {
        biquadFilterInitLPF(&filter, settings.accelCutoffHz, kSampleRateHz);
    }
}

int16_t biasCorrectedCount(int16_t raw, float bias) {
    int16_t corrected = raw;
    corrected -= static_cast<int16_t>(bias);
    return corrected;
}

Snapshot processAccepted(const ICM42688RawSample& raw, uint32_t readTimeUs,
                         uint32_t intervalUs, bool hasInterval) {
    const int16_t gyroX = biasCorrectedCount(raw.gyroscopeX, state.activeCalibration.gyro.xRawCounts);
    const int16_t gyroY = biasCorrectedCount(raw.gyroscopeY, state.activeCalibration.gyro.yRawCounts);
    const int16_t gyroZ = biasCorrectedCount(raw.gyroscopeZ, state.activeCalibration.gyro.zRawCounts);

    Snapshot next{};
    next.valid = true;
    next.sequence = state.timing.acceptedSamples + 1;
    next.readTimeUs = readTimeUs;
    next.dtSec = hasInterval ? intervalUs * 1e-6f : 0.0f;
    next.accelerationG = {
        static_cast<float>(raw.accelerationX) * kAccelRangeG / 32768.0f,
        static_cast<float>(raw.accelerationY) * kAccelRangeG / 32768.0f,
        static_cast<float>(raw.accelerationZ) * kAccelRangeG / 32768.0f};
    next.angularVelocityRadPerSec = {
        static_cast<float>(gyroX) * kGyroRangeDegreesPerSecond / 32768.0f * kDegreesToRadians,
        static_cast<float>(gyroY) * kGyroRangeDegreesPerSecond / 32768.0f * kDegreesToRadians,
        static_cast<float>(gyroZ) * kGyroRangeDegreesPerSecond / 32768.0f * kDegreesToRadians};
    next.filteredAngularVelocityRadPerSec = {
        biquadFilterApply(&state.gyroFilters[0], next.angularVelocityRadPerSec.x),
        biquadFilterApply(&state.gyroFilters[1], next.angularVelocityRadPerSec.y),
        biquadFilterApply(&state.gyroFilters[2], next.angularVelocityRadPerSec.z)};
    next.filteredAccelerationG = {
        biquadFilterApply(&state.accelFilters[0], next.accelerationG.x),
        biquadFilterApply(&state.accelFilters[1], next.accelerationG.y),
        biquadFilterApply(&state.accelFilters[2], next.accelerationG.z)};
    next.temperatureC = static_cast<float>(raw.temperature) / kTemperatureScale + kTemperatureOffsetC;

    state.mahony.update(next.filteredAngularVelocityRadPerSec.x,
                        next.filteredAngularVelocityRadPerSec.y,
                        next.filteredAngularVelocityRadPerSec.z,
                        next.filteredAccelerationG.x,
                        next.filteredAccelerationG.y,
                        next.filteredAccelerationG.z,
                        next.dtSec);
    float q0 = 0.0f;
    float q1 = 0.0f;
    float q2 = 0.0f;
    float q3 = 0.0f;
    state.mahony.getQuaternion(q0, q1, q2, q3);
    quaternionToEuler(q0, q1, q2, q3,
                      &next.attitudeDegrees.roll,
                      &next.attitudeDegrees.pitch,
                      &next.attitudeDegrees.yaw);
    next.correctedRollDegrees =
        next.attitudeDegrees.roll - state.activeCalibration.attitude.rollDegrees;
    next.correctedPitchDegrees =
        next.attitudeDegrees.pitch - state.activeCalibration.attitude.pitchDegrees;

    const float accelAngleX = atan2f(
        next.accelerationG.y,
        next.accelerationG.z + fabsf(next.accelerationG.x)) * 360.0f / 2.0f / PI;
    const float accelAngleY = atan2f(
        next.accelerationG.x,
        next.accelerationG.z + fabsf(next.accelerationG.y)) * 360.0f / -2.0f / PI;
    const float gyroRateX = static_cast<float>(gyroX) * kGyroRangeDegreesPerSecond / 32768.0f;
    const float gyroRateY = static_cast<float>(gyroY) * kGyroRangeDegreesPerSecond / 32768.0f;
    const float gyroRateZ = static_cast<float>(gyroZ) * kGyroRangeDegreesPerSecond / 32768.0f;
    const Snapshot& previous = state.published;
    next.integratedGyroAnglesDegrees = {
        previous.integratedGyroAnglesDegrees.x + gyroRateX * next.dtSec,
        previous.integratedGyroAnglesDegrees.y + gyroRateY * next.dtSec,
        previous.integratedGyroAnglesDegrees.z + gyroRateZ * next.dtSec};
    next.complementaryAnglesDegrees = {
        kComplementaryGyroWeight *
            (previous.complementaryAnglesDegrees.x + gyroRateX * next.dtSec) +
            kComplementaryAccelWeight * accelAngleX,
        kComplementaryGyroWeight *
            (previous.complementaryAnglesDegrees.y + gyroRateY * next.dtSec) +
            kComplementaryAccelWeight * accelAngleY,
        next.integratedGyroAnglesDegrees.z};
    return next;
}

} // namespace

bool begin(const Settings& settings, const CalibrationValues& calibration) {
    state = ImuState{};
    if (!validSettings(settings) || !initICM42688(kSampleRateHz)) {
        return false;
    }

    ImuState initial{};
    initial.started = true;
    initial.activeSettings = settings;
    initial.activeCalibration = calibration;
    initial.mahony = MahonyFilter(settings.mahonyTwoKp, settings.mahonyTwoKi);
    for (biquadFilter_t& filter : initial.gyroFilters) {
        biquadFilterInitLPF(&filter, settings.gyroCutoffHz, kSampleRateHz);
    }
    for (biquadFilter_t& filter : initial.accelFilters) {
        biquadFilterInitLPF(&filter, settings.accelCutoffHz, kSampleRateHz);
    }
    state = initial;
    return true;
}

bool service() {
    if (!state.started) {
        return false;
    }

    ICM42688RawSample raw{};
    if (!readIMUDataIfReady(raw)) {
        return false;
    }

    const uint32_t readTimeUs = micros();
    const uint32_t intervalUs = readTimeUs - state.previousReadTimeUs;
    const bool hasInterval = state.hasPreviousRead;
    Snapshot next = processAccepted(raw, readTimeUs, intervalUs, hasInterval);

    ++state.timing.acceptedSamples;
    if (hasInterval) {
        if (intervalUs > kLongGapThresholdUs) {
            ++state.timing.longGapCount;
        }
        if (intervalUs > state.timing.maxIntervalUs) {
            state.timing.maxIntervalUs = intervalUs;
        }
    }
    state.hasPreviousRead = true;
    state.previousReadTimeUs = readTimeUs;
    state.published = next;
    return true;
}

const Snapshot& latest() {
    return state.published;
}

Settings settings() {
    return state.activeSettings;
}

CalibrationValues calibration() {
    return state.activeCalibration;
}

TimingStatistics timingStatistics() {
    return state.timing;
}

bool setSettings(const Settings& settings) {
    if (!validSettings(settings)) {
        return false;
    }

    if (settings.gyroCutoffHz != state.activeSettings.gyroCutoffHz) {
        resetGyroFilters(settings);
    }
    if (settings.accelCutoffHz != state.activeSettings.accelCutoffHz) {
        resetAccelFilters(settings);
    }
    state.mahony.twoKp = settings.mahonyTwoKp;
    state.mahony.twoKi = settings.mahonyTwoKi;
    state.activeSettings = settings;
    return true;
}

void setCalibration(const CalibrationValues& calibration) {
    state.activeCalibration = calibration;
    state.published.correctedRollDegrees =
        state.published.attitudeDegrees.roll - calibration.attitude.rollDegrees;
    state.published.correctedPitchDegrees =
        state.published.attitudeDegrees.pitch - calibration.attitude.pitchDegrees;
}

void calibrateGyroBiasBlocking() {
    if (Logging::profile() == Logging::Profile::Idle) {
        Serial.println("Calibrating gyroscope. Keep the device still...");
    }

    int16_t gyroXSum = 0;
    int16_t gyroYSum = 0;
    int16_t gyroZSum = 0;
    int validSamples = 0;
    while (validSamples < kGyroCalibrationSamples) {
        ICM42688RawSample raw{};
        if (!readIMUDataIfReady(raw)) {
            delay(1);
            continue;
        }

        if (abs(raw.gyroscopeX) < kGyroCalibrationThreshold &&
            abs(raw.gyroscopeY) < kGyroCalibrationThreshold &&
            abs(raw.gyroscopeZ) < kGyroCalibrationThreshold) {
            gyroXSum += raw.gyroscopeX;
            gyroYSum += raw.gyroscopeY;
            gyroZSum += raw.gyroscopeZ;
            ++validSamples;
        } else {
            if (Logging::profile() == Logging::Profile::Idle) {
                Serial.print(" | x: ");
                Serial.print(raw.gyroscopeX);
                Serial.print(" y: ");
                Serial.print(raw.gyroscopeY);
                Serial.print(" Z: ");
                Serial.print(raw.gyroscopeZ);
                Serial.println("Device moved during calibration. Restarting...");
            }
            gyroXSum = 0;
            gyroYSum = 0;
            gyroZSum = 0;
            validSamples = 0;
        }
        delay(3);
    }

    state.activeCalibration.gyro = {
        static_cast<float>(gyroXSum) / kGyroCalibrationSamples,
        static_cast<float>(gyroYSum) / kGyroCalibrationSamples,
        static_cast<float>(gyroZSum) / kGyroCalibrationSamples};
    state.hasPreviousRead = false;

    if (Logging::profile() == Logging::Profile::Idle) {
        Serial.println("Gyroscope calibration completed.");
    }
}

} // namespace imu
