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

    // PCNT sensors (hardware pulse counter)
    SENSOR_NJK5002C = 30,   // Hall effect RPM sensor (NPN output)
    SENSOR_HP705A = 31,     // HP705A Clamp (Pulse output)
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

        // Soft reset the sensor (AHT10 auto-initializes after reset, no 0xE1 needed)
        Wire.beginTransmission(AHT10_ADDR);
        Wire.write(0xBA); // Soft reset command
        if (Wire.endTransmission() != 0) {
            return false;
        }
        delay(20); // Wait for reset to complete

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
    unsigned long last_read_ms_;

public:
    bool begin(uint8_t cs_pin) {
        // NOTE: SPI.begin() must be called ONCE before initializing any SPI sensors
        cs_pin_ = cs_pin;
        pinMode(cs_pin_, OUTPUT);
        digitalWrite(cs_pin_, HIGH);
        last_read_ms_ = 0;
        delay(250); // MAX6675 requires 240ms power-on delay
        return true;
    }

    // Read temperature (MAX6675 needs 220ms between conversions)
    bool read(float& temp_c) {
        // Enforce 220ms minimum between reads
        if (millis() - last_read_ms_ < 220) {
            return false;
        }

        // Timeout safety: prevent runaway state
        if (last_read_ms_ != 0 && millis() - last_read_ms_ > 5000) {
            last_read_ms_ = millis(); // Reset to prevent overflow issues
        }

        SPI.beginTransaction(SPISettings(5000000, MSBFIRST, SPI_MODE0));
        digitalWrite(cs_pin_, LOW);
        delayMicroseconds(1);

        uint16_t data = (SPI.transfer(0x00) << 8) | SPI.transfer(0x00);

        digitalWrite(cs_pin_, HIGH);
        SPI.endTransaction();

        // Enhanced error detection
        if (data & 0x04) return false; // Thermocouple open circuit (bit 2)
        if (data == 0xFFFF || data == 0x0000) return false; // Bus error or sensor fault

        // Extract temperature (bits 14-3, 0.25°C resolution)
        uint16_t raw_temp = data >> 3;
        temp_c = raw_temp * 0.25f;

        // Validate temperature range (MAX6675 is 0-1024°C)
        if (temp_c < 0 || temp_c > 1024.0f) return false;

        last_read_ms_ = millis();
        return true;
    }
};

// ======================== NJK-5002C RPM SENSOR (PCNT) ========================

#include "driver/pcnt.h"

class PCNTReader {
private:
    static constexpr uint8_t MAX_PULSE_HISTORY = 128;  // Track last 128 pulses (enough for 12000+ RPM over 500ms)
    static constexpr uint16_t WINDOW_MS = 250;         // 250ms sliding window for responsive, smooth readings
    static constexpr uint8_t MIN_PULSES = 3;           // Minimum pulses for valid RPM calculation
    
    pcnt_unit_t pcnt_unit_;
    uint8_t gpio_pin_;
    uint16_t pulses_per_rev_;
    int16_t last_count_;
    uint16_t last_rpm_;
    float smoothed_rpm_;  // Exponentially smoothed RPM for continuous, smooth readings
    
    // Circular buffer for pulse timestamps (sliding window)
    unsigned long pulse_times_[MAX_PULSE_HISTORY];
    uint8_t pulse_head_;   // Next position to write
    uint8_t pulse_count_;  // Number of valid pulses in buffer
    unsigned long last_read_ms_;

public:
    bool begin(uint8_t gpio_pin, pcnt_unit_t unit = PCNT_UNIT_0, uint16_t pulses_per_rev = 1,
               pcnt_count_mode_t pos_mode = PCNT_COUNT_DIS, pcnt_count_mode_t neg_mode = PCNT_COUNT_INC,
               uint8_t pin_mode = INPUT_PULLUP, uint16_t filter_value = 0) {
        gpio_pin_ = gpio_pin;
        pcnt_unit_ = unit;
        pulses_per_rev_ = pulses_per_rev;
        last_count_ = 0;
        last_rpm_ = 0;
        smoothed_rpm_ = 0.0f;
        pulse_head_ = 0;
        pulse_count_ = 0;
        last_read_ms_ = 0;
        
        // Clear pulse history
        for (uint8_t i = 0; i < MAX_PULSE_HISTORY; i++) {
            pulse_times_[i] = 0;
        }

        // Configure GPIO with appropriate pull resistor BEFORE initializing PCNT
        pinMode(gpio_pin_, pin_mode);
        
        // Configure PCNT unit
        pcnt_config_t pcnt_config = {};
        pcnt_config.pulse_gpio_num = gpio_pin_;
        pcnt_config.ctrl_gpio_num = PCNT_PIN_NOT_USED;
        pcnt_config.channel = PCNT_CHANNEL_0;
        pcnt_config.unit = pcnt_unit_;
        pcnt_config.pos_mode = pos_mode;
        pcnt_config.neg_mode = neg_mode;
        pcnt_config.lctrl_mode = PCNT_MODE_KEEP;
        pcnt_config.hctrl_mode = PCNT_MODE_KEEP;
        pcnt_config.counter_h_lim = 32767;
        pcnt_config.counter_l_lim = -32768;

        if (pcnt_unit_config(&pcnt_config) != ESP_OK) {
            return false;
        }

        // Configure glitch filter if specified
        // Filter value = number of APB clock cycles (80MHz) a pulse must be stable
        // filter_value=100 = ~1.25µs glitch filter (good for noisy signals)
        // filter_value=0 = disabled (for clean high-speed signals like RPM sensor)
        if (filter_value > 0) {
            pcnt_set_filter_value(pcnt_unit_, filter_value);
            pcnt_filter_enable(pcnt_unit_);
        } else {
            pcnt_filter_disable(pcnt_unit_);
        }

        // Clear and start counter
        pcnt_counter_clear(pcnt_unit_);
        pcnt_counter_resume(pcnt_unit_);

        return true;
    }

    // Sliding window RPM calculation
    // Tracks individual pulse timestamps and calculates RPM from pulses within last 500ms
    // Called at 100Hz for smooth, responsive updates
    bool read(uint16_t& rpm) {
        unsigned long now = millis();

        // Read hardware pulse count
        int16_t count;
        if (pcnt_get_counter_value(pcnt_unit_, &count) != ESP_OK) {
            rpm = last_rpm_;
            return false;
        }

        // On first call, initialize
        if (last_read_ms_ == 0) {
            last_count_ = count;
            last_read_ms_ = now;
            rpm = 0;
            last_rpm_ = 0;
            smoothed_rpm_ = 0.0f;
            return true;
        }

        // Calculate new pulses since last read
        int16_t delta_count = count - last_count_;
        if (delta_count < 0) {
            // Counter overflow or reset - clear history and restart
            pulse_count_ = 0;
            pulse_head_ = 0;
            last_count_ = count;
            last_read_ms_ = now;
            rpm = last_rpm_;
            return true;
        }

        // Add new pulses to circular buffer with interpolated timestamps
        // Distribute pulses evenly across the time since last read for smooth aging
        unsigned long time_since_last = now - last_read_ms_;
        for (int16_t i = 0; i < delta_count; i++) {
            // Interpolate timestamp: spread pulses evenly across time_since_last
            unsigned long pulse_time = last_read_ms_ + ((time_since_last * (i + 1)) / delta_count);
            pulse_times_[pulse_head_] = pulse_time;
            pulse_head_ = (pulse_head_ + 1) % MAX_PULSE_HISTORY;
            if (pulse_count_ < MAX_PULSE_HISTORY) {
                pulse_count_++;
            }
        }
        
        last_count_ = count;
        last_read_ms_ = now;

        // Count pulses within the sliding window (exactly WINDOW_MS)
        uint8_t valid_pulses = 0;
        unsigned long window_start = now - WINDOW_MS;
        unsigned long oldest_pulse_in_buffer = now;
        
        // First pass: find the oldest pulse in the entire buffer (not just valid ones)
        for (uint8_t i = 0; i < pulse_count_; i++) {
            uint8_t idx = (pulse_head_ + MAX_PULSE_HISTORY - pulse_count_ + i) % MAX_PULSE_HISTORY;
            if (pulse_times_[idx] < oldest_pulse_in_buffer) {
                oldest_pulse_in_buffer = pulse_times_[idx];
            }
        }
        
        // Second pass: count valid pulses within window
        for (uint8_t i = 0; i < pulse_count_; i++) {
            uint8_t idx = (pulse_head_ + MAX_PULSE_HISTORY - pulse_count_ + i) % MAX_PULSE_HISTORY;
            if (pulse_times_[idx] >= window_start && pulse_times_[idx] <= now) {
                valid_pulses++;
            }
        }

        // Calculate RPM from valid pulses in sliding window
        // Always use WINDOW_MS as the time base for smooth, consistent averaging
        if (valid_pulses >= MIN_PULSES && pulses_per_rev_ > 0) {
            // RPM = (pulses / pulses_per_rev) * (60000 ms/min / WINDOW_MS)
            // Use 64-bit intermediate to prevent overflow at high RPM
            float raw_rpm = (valid_pulses * 60000.0f) / (pulses_per_rev_ * WINDOW_MS);
            
            // Apply exponential moving average for smooth continuous readings
            // Alpha = 0.3 gives good balance between responsiveness and smoothness
            const float alpha = 0.3f;
            if (smoothed_rpm_ == 0.0f) {
                // First valid reading - initialize smoothed value
                smoothed_rpm_ = raw_rpm;
            } else {
                // Exponential smoothing: new_value = alpha * raw + (1 - alpha) * old
                smoothed_rpm_ = alpha * raw_rpm + (1.0f - alpha) * smoothed_rpm_;
            }
            
            rpm = (uint16_t)(smoothed_rpm_ + 0.5f);  // Round to nearest integer
            last_rpm_ = rpm;
        } else if (pulse_count_ == 0 || (now - oldest_pulse_in_buffer) > 1000) {
            // No pulses in buffer OR oldest pulse is >1 second old - stopped
            rpm = 0;
            last_rpm_ = 0;
            smoothed_rpm_ = 0.0f;
            // Clear old pulses from buffer
            if (pulse_count_ > 0 && (now - oldest_pulse_in_buffer) > 1000) {
                pulse_count_ = 0;
                pulse_head_ = 0;
            }
        } else {
            // Not enough pulses yet, return last valid RPM
            rpm = last_rpm_;
        }

        return true;
    }

    uint16_t getLastRPM() const { return last_rpm_; }
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

    // NJK-5002C RPM Sensor
    uint16_t njk5002c_rpm;

    // HP705A Clamp
    uint16_t hp705a_clamp;
};

// ======================== UNIFIED ACQUISITION SYSTEM ========================

class Acquisition {
private:
    // Sensor instances (static, shared across channels)
    static HX711Reader hx711_;
    static AHT10Reader aht10_;
    static BMP280Reader bmp280_;
    static MAX6675Reader max6675_;
    static PCNTReader pcnt_;
    static PCNTReader pcnt_hp705a_;

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
                    // SPI sensors initialized lazily in polling task (Core 0)
                    break;

                case ACQ_PCNT:
                    if (channels[i].sensor_type == 30) { // SENSOR_NJK5002C
                        // pulses_per_rev stored in peripheral_pin1 (typically 1 for single magnet)
                        // Count on FALLING edge only for clean, reliable triggering
                        // NJK-5002C NPN output goes LOW when magnet detected
                        uint16_t ppr = (channels[i].peripheral_pin1 == 255) ? 1 : channels[i].peripheral_pin1;
                        uint8_t pin_mode = (channels[i].pin_mode == PIN_MODE_INPUT_PULLUP) ? INPUT_PULLUP :
                                          (channels[i].pin_mode == PIN_MODE_INPUT_PULLDOWN) ? INPUT_PULLDOWN : INPUT;
                        pcnt_.begin(channels[i].gpio_pin, PCNT_UNIT_0, ppr, PCNT_COUNT_DIS, PCNT_COUNT_INC, pin_mode);
                        channel_states_[channels[i].channel_id].initialized = true;
                    } else if (channels[i].sensor_type == 31) { // SENSOR_HP705A
                        // pulses_per_rev stored in peripheral_pin1
                        uint16_t ppr = (channels[i].peripheral_pin1 == 255) ? 1 : channels[i].peripheral_pin1;
                        // Count on RISING edge with pull-down (using external signal conditioning)
                        // Use 1023 APB cycle filter (~12.8µs, max value) to eliminate spark ringing at high RPM
                        uint8_t pin_mode = (channels[i].pin_mode == PIN_MODE_INPUT_PULLUP) ? INPUT_PULLUP :
                                          (channels[i].pin_mode == PIN_MODE_INPUT_PULLDOWN) ? INPUT_PULLDOWN : INPUT;
                        pcnt_hp705a_.begin(channels[i].gpio_pin, PCNT_UNIT_1, ppr, PCNT_COUNT_INC, PCNT_COUNT_DIS, pin_mode, 1023);
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
    static inline uint32_t read(const ChannelConfig& ch, const ChannelConfig* all_channels, uint8_t num_channels) {
        // Fast path: GPIO and ADC (most common, no state)
        if (ch.acquisition_method == ACQ_GPIO) {
            int state = digitalRead(ch.gpio_pin);
            // If pull-up is used, logic is inverted (LOW = Active/1)
            if (ch.pin_mode == PIN_MODE_INPUT_PULLUP) {
                return (state == LOW) ? 1 : 0;
            }
            return (state == HIGH) ? 1 : 0;
        }

        if (ch.acquisition_method == ACQ_ADC) {
            return analogRead(ch.gpio_pin);
        }

        // Slow path: complex sensors (already decimated by caller)
        return readComplexSensor(ch, all_channels, num_channels);
    }

    // Background sensor polling function (called by sensorPollingTask)
    // This handles all I2C/SPI communication and timeouts off the critical path
    // NOTE: HX711 is NOT polled here - it's fast enough to read directly in DAQ task
    // NOTE: Polls each unique sensor once (not per-channel) to avoid state conflicts
    static void pollSensors(const ChannelConfig* channels, uint8_t num_channels) {
        // One-time initialization: Wire.begin() and SPI.begin() must be called from this task (Core 0)
        static bool i2c_initialized = false;
        static bool spi_initialized = false;

        if (!i2c_initialized) {
            Wire.begin();
            delay(50); // Give bus time to stabilize
            i2c_initialized = true;
        }

        if (!spi_initialized) {
            SPI.begin();
            delay(10); // Give bus time to stabilize
            spi_initialized = true;
        }

        // Check which sensors are enabled and capture their first channel_id
        // Using 255 as sentinel value for "not found"
        uint8_t aht10_ch = 255, bmp280_ch = 255, max6675_ch = 255, pcnt_ch = 255, hp705a_ch = 255;

        for (uint8_t i = 0; i < num_channels; i++) {
            if (!channels[i].enabled) continue;

            if (channels[i].acquisition_method == ACQ_I2C_ADC) {
                if (channels[i].sensor_type == 10 && aht10_ch == 255) {
                    aht10_ch = channels[i].channel_id;
                }
                if (channels[i].sensor_type == 11 && bmp280_ch == 255) {
                    bmp280_ch = channels[i].channel_id;
                }
            }
            if (channels[i].acquisition_method == ACQ_SPI_ADC) {
                if (channels[i].sensor_type == 20 && max6675_ch == 255) {
                    max6675_ch = channels[i].channel_id;
                }
            }
            if (channels[i].acquisition_method == ACQ_PCNT) {
                if (channels[i].sensor_type == 30 && pcnt_ch == 255) {
                    pcnt_ch = channels[i].channel_id;
                }
                if (channels[i].sensor_type == 31 && hp705a_ch == 255) {
                    hp705a_ch = channels[i].channel_id;
                }
            }
        }

        // Lazy initialization for I2C sensors (must be done from this task)
        if (aht10_ch != 255 && !channel_states_[aht10_ch].initialized) {
            if (aht10_.begin()) {
                // Mark all AHT10 channels as initialized
                for (uint8_t i = 0; i < num_channels; i++) {
                    if (channels[i].enabled && channels[i].sensor_type == 10) {
                        channel_states_[channels[i].channel_id].initialized = true;
                    }
                }
                delay(20);
            }
        }

        if (bmp280_ch != 255 && !channel_states_[bmp280_ch].initialized) {
            if (bmp280_.begin()) {
                // Mark all BMP280 channels as initialized
                for (uint8_t i = 0; i < num_channels; i++) {
                    if (channels[i].enabled && channels[i].sensor_type == 11) {
                        channel_states_[channels[i].channel_id].initialized = true;
                    }
                }
                delay(20);
            }
        }

        // Lazy initialization for SPI sensors (must be done from this task)
        if (max6675_ch != 255 && !channel_states_[max6675_ch].initialized) {
            // Find the CS pin from channel config
            for (uint8_t i = 0; i < num_channels; i++) {
                if (channels[i].enabled && channels[i].sensor_type == 20) {
                    if (max6675_.begin(channels[i].peripheral_pin1)) {
                        channel_states_[channels[i].channel_id].initialized = true;
                    }
                    break;
                }
            }
        }

        // Poll each enabled sensor once (not per-channel!)
        if (aht10_ch != 255 && channel_states_[aht10_ch].initialized) {
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

        if (bmp280_ch != 255 && channel_states_[bmp280_ch].initialized) {
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

        if (max6675_ch != 255 && channel_states_[max6675_ch].initialized) {
            float temp;
            if (max6675_.read(temp)) {
                sensor_cache_.max6675_temp = temp;
            }
        }

        if (pcnt_ch != 255 && channel_states_[pcnt_ch].initialized) {
            uint16_t rpm;
            if (pcnt_.read(rpm)) {
                sensor_cache_.njk5002c_rpm = rpm;
            }
        }

        if (hp705a_ch != 255 && channel_states_[hp705a_ch].initialized) {
            uint16_t val;
            if (pcnt_hp705a_.read(val)) {
                sensor_cache_.hp705a_clamp = val;
            }
        }
    }

private:
    // Helper: Find which output index this channel is for multi-output sensors
    // Returns 0 for first channel with this sensor_type, 1 for second, etc.
    static inline uint8_t getOutputIndex(const ChannelConfig& ch, const ChannelConfig* all_channels, uint8_t num_channels) {
        uint8_t index = 0;
        for (uint8_t i = 0; i < num_channels; i++) {
            if (all_channels[i].sensor_type == ch.sensor_type) {
                if (all_channels[i].channel_id == ch.channel_id) {
                    return index; // Found our channel
                }
                index++;
            }
        }
        return 0; // Default to first output
    }

    // Read complex sensors (HX711 is fast non-blocking, I2C/SPI use cache)
    static uint32_t readComplexSensor(const ChannelConfig& ch, const ChannelConfig* all_channels, uint8_t num_channels) {
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
                    // Use channel ordering: 0 = temp, 1 = humidity
                    uint8_t idx = getOutputIndex(ch, all_channels, num_channels);
                    float value = (idx == 0) ? sensor_cache_.aht10_temp : sensor_cache_.aht10_humidity;
                    return *(uint32_t*)&value;
                }
                if (ch.sensor_type == 11) { // SENSOR_BMP280
                    // Use channel ordering: 0 = temp, 1 = pressure
                    uint8_t idx = getOutputIndex(ch, all_channels, num_channels);
                    float value = (idx == 0) ? sensor_cache_.bmp280_temp : sensor_cache_.bmp280_pressure;
                    return *(uint32_t*)&value;
                }
                return 0;

            case ACQ_SPI_ADC:
                if (ch.sensor_type == 20) { // SENSOR_MAX6675
                    return *(uint32_t*)&sensor_cache_.max6675_temp;
                }
                return 0;

            case ACQ_PCNT:
                if (ch.sensor_type == 30) { // SENSOR_NJK5002C
                    return (uint32_t)sensor_cache_.njk5002c_rpm;
                }
                if (ch.sensor_type == 31) { // SENSOR_HP705A
                    return (uint32_t)sensor_cache_.hp705a_clamp;
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
PCNTReader Acquisition::pcnt_;
PCNTReader Acquisition::pcnt_hp705a_;
SensorState Acquisition::channel_states_[16];
bool Acquisition::sensors_initialized_ = false;
SensorCache Acquisition::sensor_cache_;
bool Acquisition::aht10_pending_ = false;
bool Acquisition::bmp280_pending_ = false;
