#include "ICM42688.h"
#include <SPI.h>
#include <Arduino.h>
#include <cstdint>

namespace {
SPIClass * const hspi = new SPIClass(HSPI);

constexpr uint8_t kDataReadyBit = 1U << 3;

bool outputRateRegisterValue(uint16_t sampleRateHz, uint8_t& registerValue) {
    switch (sampleRateHz) {
        case 1000:
            registerValue = ODR_1KHZ;
            return true;
        default:
            return false;
    }
}

int16_t signedWord(const uint8_t* bytes) {
    const uint16_t value = static_cast<uint16_t>(bytes[0]) << 8 | bytes[1];
    return static_cast<int16_t>(value);
}

/**
 * @brief Read multiple bytes of data starting from the specified register address of the ICM42688 sensor
 * 
 * @param reg Starting register address
 * @param data Pointer to array for storing read data
 * @param length Number of bytes to read
 */
void readBytes(uint8_t reg, uint8_t *data, uint8_t length) {
    // Pull CS pin low to select ICM42688 sensor
    digitalWrite(CS_PIN, LOW);
    // Send read command by setting the highest bit of register address to 1 through bitwise OR operation
    hspi->transfer(reg | 0x80); 
    // Loop to read specified number of bytes
    for (uint8_t i = 0; i < length; i++) {
        // Transfer 0x00 through SPI bus and receive data returned by sensor
        data[i] = hspi->transfer(0x00);
    }
    // Pull CS pin high to deselect ICM42688 sensor
    digitalWrite(CS_PIN, HIGH);
}

/**
 * @brief Write multiple bytes of data starting from the specified register address of the ICM42688 sensor
 * 
 * @param reg Starting register address
 * @param data Pointer to array containing data to write
 * @param length Number of bytes to write
 */
void writeBytes(uint8_t reg, uint8_t *data, uint8_t length) {
    // Pull CS pin low to select ICM42688 sensor
    digitalWrite(CS_PIN, LOW);
    // Send register address to write
    hspi->transfer(reg);
    // Loop to write specified number of bytes
    for (uint8_t i = 0; i < length; i++) {
        // Send data to write through SPI bus
        hspi->transfer(data[i]);
    }
    // Pull CS pin high to deselect ICM42688 sensor
    digitalWrite(CS_PIN, HIGH);
}

/**
 * @brief Select user bank of ICM42688 sensor
 * 
 * @param bank User bank number to select
 */
void selectUserBank(uint8_t bank) {
    uint8_t data = bank;
    // Write user bank number to REG_BANK_SEL register
    writeBytes(REG_BANK_SEL, &data, 1);
}

}  // namespace

/**
 * @brief Initialize ICM42688 sensor, including HSPI interface, sensor reset, device ID check,
 *        enable accelerometer and gyroscope, configure range and sampling rate, and set up
 *        low-pass filter and notch filter
 * 
 * @return bool Returns true if initialization successful, false if failed
 */
bool initICM42688(uint16_t sampleRateHz) {
    uint8_t outputRate = 0;
    if (!outputRateRegisterValue(sampleRateHz, outputRate)) {
        return false;
    }

    pinMode(CS_PIN, OUTPUT);
    digitalWrite(CS_PIN, HIGH);
    // Initialize HSPI interface, set SPI pins and working mode
    hspi->begin(SCK_PIN, MISO_PIN, MOSI_PIN, CS_PIN);
    // Set SPI data mode to mode 3
    hspi->setDataMode(SPI_MODE3);
    // Set SPI clock divider, modify SPI clock frequency to system clock/32
    hspi->setClockDivider(SPI_CLOCK_DIV32); 

    // Select user bank 0
    selectUserBank(0);

    // Software reset sensor, write reset command to device configuration register
    uint8_t resetData = 0x01;
    writeBytes(UB0_REG_DEVICE_CONFIG, &resetData, 1);
    // Delay 100ms, wait for reset operation to complete
    delay(100);

    // Check device ID, read device ID from WHO_AM_I register
    uint8_t whoAmI;
    readBytes(UB0_REG_WHO_AM_I, &whoAmI, 1);
    // Compare if read device ID matches preset value
    if (whoAmI != WHO_AM_I) {
        // If not matching, print error message and return false
        Serial.println("ICM42688 device ID check failed!");
        return false;
    }

    // Enable accelerometer and gyroscope, write enable command to power management register
    uint8_t pwrData = 0x0F;
    writeBytes(UB0_REG_PWR_MGMT0, &pwrData, 1);
    // Delay 100ms, wait for enable operation to take effect
    delay(100);

    // Configure accelerometer
    // Set accelerometer range to ±8g, sampling rate to 1KHZ
    sAccelConfig0_t accelConfig0;
    accelConfig0.accelODR = outputRate;
    accelConfig0.accelFsSel = gpm8;
    accelConfig0.reserved = 0;
    // Convert structure data to byte data
    uint8_t accelConfigByte = *((uint8_t*)&accelConfig0);
    // Write configuration data to accelerometer configuration register 0
    writeBytes(UB0_REG_ACCEL_CONFIG0, &accelConfigByte, 1);

    // Configure gyroscope
    // Set gyroscope range to ±2000dps, sampling rate to 1KHZ
    sGyroConfig0_t gyroConfig0;
    gyroConfig0.gyroODR = outputRate;
    gyroConfig0.gyroFsSel = dps2000;
    gyroConfig0.reserved = 0;
    // Convert structure data to byte data
    uint8_t gyroConfigByte = *((uint8_t*)&gyroConfig0);
    // Write configuration data to gyroscope configuration register 0
    writeBytes(UB0_REG_GYRO_CONFIG0, &gyroConfigByte, 1);

    // Configure low-pass filter for accelerometer and gyroscope
    // Accelerometer LPF bandwidth is max(400Hz, ODR)/4
    uint8_t accelLPF = 1;
    // Gyroscope LPF bandwidth is max(400Hz, ODR)/4
    uint8_t gyroLPF = 1;
    sGyroAccelConfig0_t gyroAccelConfig0;
    gyroAccelConfig0.accelUIFiltBW = accelLPF;
    gyroAccelConfig0.gyroUIFiltBW = gyroLPF;
    // Convert structure data to byte data
    uint8_t gyroAccelConfigByte = *((uint8_t*)&gyroAccelConfig0);
    // Write configuration data to gyroscope and accelerometer configuration register 0
    writeBytes(UB0_REG_GYRO_ACCEL_CONFIG0, &gyroAccelConfigByte, 1);

    // Configure gyroscope notch filter
    // Enable gyroscope notch filter, enable anti-aliasing filter
    uint8_t gyroNotchConfig = 0x00;
    // Select user bank 1, as notch filter configuration register is in user bank 1
    selectUserBank(1);
    sGyroConfigStatic2_t gyroConfigStatic2;
    // Extract notch filter enable flag through bit operation
    gyroConfigStatic2.gyroNFDis = (gyroNotchConfig & 0x01);
    // Extract anti-aliasing filter enable flag through bit operation
    gyroConfigStatic2.gyroAAFDis = (gyroNotchConfig & 0x02) >> 1;
    gyroConfigStatic2.reserved = 0;
    // Convert structure data to byte data
    uint8_t gyroConfigStatic2Byte = *((uint8_t*)&gyroConfigStatic2);
    // Write configuration data to gyroscope static configuration register 2
    writeBytes(UB1_REG_GYRO_CONFIG_STATIC2, &gyroConfigStatic2Byte, 1);

    // Return to user bank 0
    selectUserBank(0);

    return true;
}

bool readIMUDataIfReady(ICM42688RawSample& sample) {
    // INT_STATUS is read-clear: this indicates a pending data-ready event, not a
    // conversion counter. A new event can race the following burst read.
    selectUserBank(0);
    uint8_t status = 0;
    readBytes(UB0_REG_INT_STATUS, &status, 1);
    if ((status & kDataReadyBit) == 0) {
        return false;
    }

    uint8_t data[14];
    readBytes(UB0_REG_TEMP_DATA1, data, 14);

    const ICM42688RawSample next{
        signedWord(&data[2]), signedWord(&data[4]), signedWord(&data[6]),
        signedWord(&data[8]), signedWord(&data[10]), signedWord(&data[12]),
        signedWord(&data[0])};
    sample = next;
    return true;
}
