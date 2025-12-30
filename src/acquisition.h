#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "deviceConfig.h"

// ======================== SENSOR TYPE DEFINITIONS ========================

// Specific sensor models for complex acquisition methods
enum SensorType : uint8_t {
    SENSOR_NONE = 0,

    // HX711 variants
    SENSOR_HX711 = 1,

    // I2C sensors
    SENSOR_AHT10 = 10,      // Temperature + Humidity
    SENSOR_BMP280 = 11,     // Pressure + Temperature
    SENSOR_ADS1115 = 12,    // 16-bit ADC

    // SPI sensors
    SENSOR_MAX6675 = 20,    // K-Type thermocouple
    SENSOR_MCP3008 = 21,    // 10-bit 8-channel ADC
};

// Sensor state tracking for non-blocking operation
struct SensorState {
    uint32_t last_value;           // Last successfully read value
    unsigned long last_read_ms;    // Timestamp of last successful read
    bool conversion_pending;       // Conversion started, waiting for result
    bool initialized;              // Sensor has been initialized
    uint8_t error_count;           // Consecutive error counter
};

// ======================== HX711 LOAD CELL (24-bit ADC) ========================

class HX711Reader {
private:
    uint8_t dout_pin_;
    uint8_t sck_pin_;
    uint32_t last_value_;
    int32_t offset_;  // Tare offset
    static portMUX_TYPE mux;

public:
    void begin(uint8_t dout_pin, uint8_t sck_pin) {
        dout_pin_ = dout_pin;
        sck_pin_ = sck_pin;
        pinMode(dout_pin_, INPUT);
        pinMode(sck_pin_, OUTPUT);
        digitalWrite(sck_pin_, LOW);
        last_value_ = 0;
        offset_ = 0;

        // Wait for HX711 to be ready (first conversion after power-on)
        // HX711 needs time to settle after power-on
        unsigned long start = millis();
        while (digitalRead(dout_pin_) == HIGH && (millis() - start) < 2000) {
            delay(10);
        }

        // If still not ready after 2 seconds, give up on tare
        if (digitalRead(dout_pin_) == HIGH) {
            return;
        }

        // Now do tare: read and average 10 samples
        int64_t sum = 0;
        uint8_t count = 0;

        for (uint8_t i = 0; i < 10; i++) {
            // Wait for next conversion (up to 200ms at 10Hz)
            unsigned long wait_start = millis();
            while (digitalRead(dout_pin_) == HIGH && (millis() - wait_start) < 200) {
                delay(1);
            }

            // Read if ready
            uint32_t value;
            if (digitalRead(dout_pin_) == LOW && read(value)) {
                sum += (int32_t)value;
                count++;
            }

            delay(10); // Small delay before next sample
        }

        // Set offset if we got any readings
        if (count > 0) {
            offset_ = (int32_t)(sum / count);
        }
    }

    // Non-blocking read - returns true if new value available
    bool read(uint32_t& value) {
        // Check if HX711 is ready (DOUT goes LOW when conversion complete)
        if (digitalRead(dout_pin_) == HIGH) {
            return false; // Not ready yet
        }

        // CRITICAL: Disable interrupts during bit-banging
        // HX711 enters power-down mode if clock pulses are delayed >60µs
        portENTER_CRITICAL(&mux);

        // Read 24 bits (MSB first)
        uint32_t data = 0;
        for (int i = 0; i < 24; i++) {
            digitalWrite(sck_pin_, HIGH);
            delayMicroseconds(1);
            data = (data << 1) | digitalRead(dout_pin_);
            digitalWrite(sck_pin_, LOW);
            delayMicroseconds(1);
        }

        // Set gain for next conversion (1 pulse = channel A, gain 128)
        // This also triggers the next conversion (~100ms @ 10Hz rate)
        digitalWrite(sck_pin_, HIGH);
        delayMicroseconds(1);
        digitalWrite(sck_pin_, LOW);

        portEXIT_CRITICAL(&mux);

        // Convert 24-bit two's complement to 32-bit signed
        if (data & 0x800000) {
            data |= 0xFF000000;
        }

        value = data;
        last_value_ = data;
        return true;
    }

    uint32_t getLastValue() const { return last_value_; }

    // Get tared value (raw - offset)
    int32_t getTaredValue() const {
        return (int32_t)last_value_ - offset_;
    }

    // Get the tare offset
    int32_t getOffset() const { return offset_; }
};

// ======================== AHT10 TEMP/HUMIDITY SENSOR ========================

class AHT10Reader {
private:
    static constexpr uint8_t AHT10_ADDR = 0x38;
    static constexpr uint8_t AHT10_CMD_INIT = 0xE1;
    static constexpr uint8_t AHT10_CMD_MEASURE = 0xAC;

    float last_temp_;
    float last_humidity_;
    unsigned long conversion_start_ms_;

public:
    bool begin() {
        // NOTE: Wire.begin() must be called ONCE before initializing any I2C sensors
        delay(40); // AHT10 requires 40ms power-on delay

        // Soft reset the sensor
        Wire.beginTransmission(AHT10_ADDR);
        Wire.write(0xBA); // Soft reset command
        uint8_t error = Wire.endTransmission();
        if (error != 0) {
            return false;
        }
        delay(20); // Wait for reset to complete

        // Skip explicit init - AHT10 auto-initializes after reset
        // Just verify the sensor responds
        Wire.beginTransmission(AHT10_ADDR);
        error = Wire.endTransmission();
        if (error != 0) {
            return false;
        }

        last_temp_ = 0;
        last_humidity_ = 0;
        conversion_start_ms_ = 0;
        return true;
    }

    // Start measurement (non-blocking)
    bool startConversion() {
        Wire.beginTransmission(AHT10_ADDR);
        Wire.write(AHT10_CMD_MEASURE);
        Wire.write(0x33);
        Wire.write(0x00);
        if (Wire.endTransmission() != 0) return false;

        conversion_start_ms_ = millis();
        return true;
    }

    // Check if conversion is ready and read (non-blocking)
    bool read(float& temp_c, float& humidity_percent) {
        // AHT10 needs 75ms minimum for measurement
        if (millis() - conversion_start_ms_ < 80) {
            return false; // Not ready yet
        }

        // Read 6 bytes
        if (Wire.requestFrom(AHT10_ADDR, 6) != 6) return false;

        uint8_t status = Wire.read();
        if (status & 0x80) return false; // Busy bit set

        // Read the 5 data bytes (humidity and temp share byte 3)
        uint8_t byte1 = Wire.read();  // Humidity[19:12]
        uint8_t byte2 = Wire.read();  // Humidity[11:4]
        uint8_t byte3 = Wire.read();  // Humidity[3:0] (upper nibble) | Temp[19:16] (lower nibble)
        uint8_t byte4 = Wire.read();  // Temp[15:8]
        uint8_t byte5 = Wire.read();  // Temp[7:0]

        // Parse 20-bit humidity (upper 2.5 bytes)
        uint32_t raw_humidity = ((uint32_t)byte1 << 12) |
                                ((uint32_t)byte2 << 4) |
                                ((uint32_t)byte3 >> 4);

        // Parse 20-bit temperature (lower 2.5 bytes, shares byte3)
        uint32_t raw_temp = (((uint32_t)byte3 & 0x0F) << 16) |
                            ((uint32_t)byte4 << 8) |
                            (uint32_t)byte5;

        // Convert to physical units
        humidity_percent = ((float)raw_humidity / 1048576.0f) * 100.0f;
        temp_c = ((float)raw_temp / 1048576.0f) * 200.0f - 50.0f;

        last_humidity_ = humidity_percent;
        last_temp_ = temp_c;
        return true;
    }

    float getLastTemp() const { return last_temp_; }
    float getLastHumidity() const { return last_humidity_; }
};

// ======================== BMP280 PRESSURE/TEMP SENSOR ========================

class BMP280Reader {
private:
    static constexpr uint8_t BMP280_ADDR = 0x76;
    static constexpr uint8_t BMP280_REG_CTRL_MEAS = 0xF4;
    static constexpr uint8_t BMP280_REG_DATA = 0xF7;
    static constexpr uint8_t BMP280_REG_CALIB = 0x88;

    // Calibration coefficients
    uint16_t dig_T1;
    int16_t dig_T2, dig_T3;
    uint16_t dig_P1;
    int16_t dig_P2, dig_P3, dig_P4, dig_P5, dig_P6, dig_P7, dig_P8, dig_P9;

    int32_t t_fine; // Temperature compensation value
    float last_temp_;
    float last_pressure_;
    unsigned long conversion_start_ms_;

public:
    bool begin() {
        // NOTE: Wire.begin() must be called ONCE before initializing any I2C sensors

        // Read calibration data
        Wire.beginTransmission(BMP280_ADDR);
        Wire.write(BMP280_REG_CALIB);
        if (Wire.endTransmission() != 0) return false;

        if (Wire.requestFrom(BMP280_ADDR, 24) != 24) return false;

        dig_T1 = Wire.read() | (Wire.read() << 8);
        dig_T2 = Wire.read() | (Wire.read() << 8);
        dig_T3 = Wire.read() | (Wire.read() << 8);
        dig_P1 = Wire.read() | (Wire.read() << 8);
        dig_P2 = Wire.read() | (Wire.read() << 8);
        dig_P3 = Wire.read() | (Wire.read() << 8);
        dig_P4 = Wire.read() | (Wire.read() << 8);
        dig_P5 = Wire.read() | (Wire.read() << 8);
        dig_P6 = Wire.read() | (Wire.read() << 8);
        dig_P7 = Wire.read() | (Wire.read() << 8);
        dig_P8 = Wire.read() | (Wire.read() << 8);
        dig_P9 = Wire.read() | (Wire.read() << 8);

        last_temp_ = 0;
        last_pressure_ = 0;
        conversion_start_ms_ = 0;
        return true;
    }

    // Start forced measurement mode (non-blocking)
    bool startConversion() {
        // Forced mode, oversampling x1 for temp and pressure
        Wire.beginTransmission(BMP280_ADDR);
        Wire.write(BMP280_REG_CTRL_MEAS);
        Wire.write(0x25); // osrs_t=1, osrs_p=1, mode=forced
        if (Wire.endTransmission() != 0) return false;

        conversion_start_ms_ = millis();
        return true;
    }

    // Read compensated values (non-blocking)
    bool read(float& temp_c, float& pressure_pa) {
        // BMP280 needs ~10ms for measurement (x1 oversampling)
        if (millis() - conversion_start_ms_ < 15) {
            return false;
        }

        // Timeout safety: if conversion took >100ms, something is wrong
        if (millis() - conversion_start_ms_ > 100) {
            return false; // Force retry by resetting pending flag
        }

        // Read raw data (pressure + temperature)
        Wire.beginTransmission(BMP280_ADDR);
        Wire.write(BMP280_REG_DATA);
        if (Wire.endTransmission() != 0) return false;

        if (Wire.requestFrom(BMP280_ADDR, 6) != 6) return false;

        // Read pressure (MSB first, 20-bit value in upper bits of 3 bytes)
        uint8_t press_msb = Wire.read();   // [19:12]
        uint8_t press_lsb = Wire.read();   // [11:4]
        uint8_t press_xlsb = Wire.read();  // [3:0] in upper nibble

        // Read temperature (MSB first, 20-bit value in upper bits of 3 bytes)
        uint8_t temp_msb = Wire.read();    // [19:12]
        uint8_t temp_lsb = Wire.read();    // [11:4]
        uint8_t temp_xlsb = Wire.read();   // [3:0] in upper nibble

        int32_t adc_P = ((int32_t)press_msb << 12) | ((int32_t)press_lsb << 4) | ((int32_t)press_xlsb >> 4);
        int32_t adc_T = ((int32_t)temp_msb << 12) | ((int32_t)temp_lsb << 4) | ((int32_t)temp_xlsb >> 4);

        // Compensate temperature
        int32_t var1 = ((((adc_T >> 3) - ((int32_t)dig_T1 << 1))) * ((int32_t)dig_T2)) >> 11;
        int32_t var2 = (((((adc_T >> 4) - ((int32_t)dig_T1)) * ((adc_T >> 4) - ((int32_t)dig_T1))) >> 12) * ((int32_t)dig_T3)) >> 14;
        t_fine = var1 + var2;
        temp_c = ((t_fine * 5 + 128) >> 8) / 100.0f;

        // Compensate pressure
        int64_t var1_64 = ((int64_t)t_fine) - 128000;
        int64_t var2_64 = var1_64 * var1_64 * (int64_t)dig_P6;
        var2_64 = var2_64 + ((var1_64 * (int64_t)dig_P5) << 17);
        var2_64 = var2_64 + (((int64_t)dig_P4) << 35);
        var1_64 = ((var1_64 * var1_64 * (int64_t)dig_P3) >> 8) + ((var1_64 * (int64_t)dig_P2) << 12);
        var1_64 = (((((int64_t)1) << 47) + var1_64)) * ((int64_t)dig_P1) >> 33;

        if (var1_64 == 0) return false;

        int64_t p = 1048576 - adc_P;
        p = (((p << 31) - var2_64) * 3125) / var1_64;
        var1_64 = (((int64_t)dig_P9) * (p >> 13) * (p >> 13)) >> 25;
        var2_64 = (((int64_t)dig_P8) * p) >> 19;
        p = ((p + var1_64 + var2_64) >> 8) + (((int64_t)dig_P7) << 4);
        pressure_pa = (float)p / 256.0f;

        last_temp_ = temp_c;
        last_pressure_ = pressure_pa;
        return true;
    }

    float getLastTemp() const { return last_temp_; }
    float getLastPressure() const { return last_pressure_; }
};

// ======================== MAX6675 THERMOCOUPLE (SPI) ========================

class MAX6675Reader {
private:
    uint8_t cs_pin_;
    float last_temp_;
    unsigned long last_read_ms_;

public:
    void begin(uint8_t cs_pin) {
        cs_pin_ = cs_pin;
        pinMode(cs_pin_, OUTPUT);
        digitalWrite(cs_pin_, HIGH);
        SPI.begin();
        last_temp_ = 0;
        last_read_ms_ = 0;
    }

    // Read temperature (MAX6675 needs 220ms between conversions)
    bool read(float& temp_c) {
        // Enforce 220ms minimum between reads
        if (millis() - last_read_ms_ < 220) {
            return false;
        }

        SPI.beginTransaction(SPISettings(4000000, MSBFIRST, SPI_MODE0));
        digitalWrite(cs_pin_, LOW);
        delayMicroseconds(1);

        uint16_t data = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);

        digitalWrite(cs_pin_, HIGH);
        SPI.endTransaction();

        // Check for errors
        if (data & 0x04) return false; // Thermocouple open circuit

        // Extract temperature (bits 14-3, 0.25°C resolution)
        data >>= 3;
        temp_c = data * 0.25f;

        last_temp_ = temp_c;
        last_read_ms_ = millis();
        return true;
    }

    float getLastTemp() const { return last_temp_; }
};

// ======================== SENSOR CACHE (Thread-safe) ========================

// Global cache for complex sensor values:
// - HX711: Read directly in DAQ task (fast ~50µs), cached for consistency
// - I2C/SPI sensors: Polled in background task, cached to avoid DAQ task blocking
struct SensorCache {
    // HX711 Load Cell (tared value)
    uint32_t hx711_value;

    // AHT10 Temperature + Humidity
    float aht10_temp;
    float aht10_humidity;

    // BMP280 Pressure + Temperature
    float bmp280_temp;
    float bmp280_pressure;

    // MAX6675 Thermocouple
    float max6675_temp;
};

// ======================== UNIFIED ACQUISITION SYSTEM ========================

class Acquisition {
private:
    // Sensor instances (static, shared across channels)
    static HX711Reader hx711_;
    static AHT10Reader aht10_;
    static BMP280Reader bmp280_;
    static MAX6675Reader max6675_;

    // Per-channel state tracking
    static SensorState channel_states_[16];

    // Global sensor initialization flags (checked once at init, not in hot path)
    static bool sensors_initialized_;

    // Global sensor cache
    static SensorCache sensor_cache_;

    // Conversion pending flags for I2C/SPI sensors (background polling task)
    static bool aht10_pending_;
    static bool bmp280_pending_;

public:
    static void init() {
        for (int i = 0; i < 16; i++) {
            channel_states_[i].last_value = 0;
            channel_states_[i].last_read_ms = 0;
            channel_states_[i].conversion_pending = false;
            channel_states_[i].initialized = false;
            channel_states_[i].error_count = 0;
        }

        sensors_initialized_ = false;

        // Initialize sensor cache
        sensor_cache_.hx711_value = 0;
        sensor_cache_.aht10_temp = 0;
        sensor_cache_.aht10_humidity = 0;
        sensor_cache_.bmp280_temp = 0;
        sensor_cache_.bmp280_pressure = 0;
        sensor_cache_.max6675_temp = 0;

        aht10_pending_ = false;
        bmp280_pending_ = false;
    }

    // Pre-initialize sensors for enabled channels (called once from main)
    // NOTE: I2C sensors are initialized lazily from the sensor polling task
    static void initializeSensors(const ChannelConfig* channels, uint8_t num_channels) {
        if (sensors_initialized_) return;

        for (uint8_t i = 0; i < num_channels; i++) {
            if (!channels[i].enabled) continue;

            switch (channels[i].acquisition_method) {
                case ACQ_HX711:
                    hx711_.begin(channels[i].gpio_pin, channels[i].peripheral_pin1);
                    channel_states_[channels[i].channel_id].initialized = true;
                    break;

                case ACQ_I2C_ADC:
                    // I2C sensors initialized lazily in polling task (Core 0)
                    break;

                case ACQ_SPI_ADC:
                    if (channels[i].sensor_type == 20) {
                        max6675_.begin(channels[i].peripheral_pin1);
                        channel_states_[channels[i].channel_id].initialized = true;
                    }
                    break;

                default:
                    break;
            }
        }

        sensors_initialized_ = true;
    }

    // Optimized read function with minimal branching for hot path
    static inline uint32_t read(const ChannelConfig& ch) {
        // Fast path: GPIO and ADC (most common, no state)
        if (ch.acquisition_method == ACQ_GPIO) {
            return digitalRead(ch.gpio_pin) ? 1 : 0;
        }

        if (ch.acquisition_method == ACQ_ADC) {
            return analogRead(ch.gpio_pin);
        }

        // Slow path: complex sensors (already decimated by caller)
        return readComplexSensor(ch);
    }

    // Background sensor polling function (called by sensorPollingTask)
    // This handles all I2C/SPI communication and timeouts off the critical path
    // NOTE: HX711 is NOT polled here - it's fast enough to read directly in DAQ task
    // NOTE: Polls each unique sensor once (not per-channel) to avoid state conflicts
    static void pollSensors(const ChannelConfig* channels, uint8_t num_channels) {
        // One-time initialization: Wire.begin() must be called from this task (Core 0)
        static bool i2c_initialized = false;
        if (!i2c_initialized) {
            Wire.begin();

            // Give bus time to stabilize
            delay(50);

            i2c_initialized = true;
        }

        // Check which sensors are enabled by scanning all channels
        bool aht10_enabled = false;
        bool bmp280_enabled = false;
        bool max6675_enabled = false;

        for (uint8_t i = 0; i < num_channels; i++) {
            if (!channels[i].enabled) continue;
            if (channels[i].acquisition_method == ACQ_I2C_ADC) {
                if (channels[i].sensor_type == 10) aht10_enabled = true;
                if (channels[i].sensor_type == 11) bmp280_enabled = true;
            }
            if (channels[i].acquisition_method == ACQ_SPI_ADC) {
                if (channels[i].sensor_type == 20) max6675_enabled = true;
            }
        }

        // Lazy initialization for I2C sensors (must be done from this task)
        // Initialize AHT10 first (simpler sensor)
        if (aht10_enabled && !channel_states_[10].initialized) {
            if (aht10_.begin()) {
                channel_states_[10].initialized = true;
                channel_states_[11].initialized = true;
                delay(20); // Let sensor stabilize
            }
        }

        // Initialize BMP280 second
        if (bmp280_enabled && !channel_states_[12].initialized) {
            if (bmp280_.begin()) {
                channel_states_[12].initialized = true;
                channel_states_[13].initialized = true;
                delay(20); // Let sensor stabilize
            }
        }

        // Poll each enabled sensor once (not per-channel!)
        if (aht10_enabled && channel_states_[10].initialized) {
            if (!aht10_pending_) {
                aht10_.startConversion();
                aht10_pending_ = true;
            } else {
                float temp, humidity;
                if (aht10_.read(temp, humidity)) {
                    sensor_cache_.aht10_temp = temp;
                    sensor_cache_.aht10_humidity = humidity;
                    aht10_pending_ = false;
                }
            }
        }

        if (bmp280_enabled && channel_states_[12].initialized) {
            if (!bmp280_pending_) {
                bmp280_.startConversion();
                bmp280_pending_ = true;
            } else {
                float temp, pressure;
                if (bmp280_.read(temp, pressure)) {
                    sensor_cache_.bmp280_temp = temp;
                    sensor_cache_.bmp280_pressure = pressure;
                    bmp280_pending_ = false;
                }
            }
        }

        if (max6675_enabled) {
            float temp;
            if (max6675_.read(temp)) {
                sensor_cache_.max6675_temp = temp;
            }
        }
    }

private:
    // Read complex sensors (HX711 is fast non-blocking, I2C/SPI use cache)
    static uint32_t readComplexSensor(const ChannelConfig& ch) {
        switch (ch.acquisition_method) {
            case ACQ_HX711: {
                // HX711 is fast (~50µs when ready), read directly
                uint32_t raw_value;
                if (hx711_.read(raw_value)) {
                    // Store tared value (raw - offset) in cache
                    sensor_cache_.hx711_value = (uint32_t)hx711_.getTaredValue();
                }
                return sensor_cache_.hx711_value; // Return tared value
            }

            case ACQ_I2C_ADC:
                if (ch.sensor_type == 10) { // SENSOR_AHT10
                    float value = (ch.channel_id == 10) ? sensor_cache_.aht10_temp : sensor_cache_.aht10_humidity;
                    return *(uint32_t*)&value;
                }
                if (ch.sensor_type == 11) { // SENSOR_BMP280
                    float value = (ch.channel_id == 12) ? sensor_cache_.bmp280_temp : sensor_cache_.bmp280_pressure;
                    return *(uint32_t*)&value;
                }
                return 0;

            case ACQ_SPI_ADC:
                if (ch.sensor_type == 20) { // SENSOR_MAX6675
                    return *(uint32_t*)&sensor_cache_.max6675_temp;
                }
                return 0;

            default:
                return 0;
        }
    }
};

// Static member initialization
portMUX_TYPE HX711Reader::mux = portMUX_INITIALIZER_UNLOCKED;
HX711Reader Acquisition::hx711_;
AHT10Reader Acquisition::aht10_;
BMP280Reader Acquisition::bmp280_;
MAX6675Reader Acquisition::max6675_;
SensorState Acquisition::channel_states_[16];
bool Acquisition::sensors_initialized_ = false;
SensorCache Acquisition::sensor_cache_;
bool Acquisition::aht10_pending_ = false;
bool Acquisition::bmp280_pending_ = false;
