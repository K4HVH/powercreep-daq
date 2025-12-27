#pragma once

#include <Arduino.h>
#include "deviceConfig.h"
#include "frameBuilder.h"

// Sample buffer for batching
constexpr size_t SAMPLES_PER_BATCH = 10;  // Send 10 samples per DATA_BATCH frame

struct Sample {
    uint8_t channel_id;
    uint16_t value;  // 12-bit ADC value (0-4095)
};

// Global flag for timer ISR (must be in DRAM for ISR access)
volatile bool DRAM_ATTR g_sample_flag = false;

// Timer ISR - must be outside class and in IRAM
void IRAM_ATTR onTimerISR() {
    g_sample_flag = true;
}

class DataAcquisition {
public:
    DataAcquisition() :
        sequence_number_(0),
        sample_index_(0),
        sim_time_(0) {}

    void begin() {
        // Configure ADC
        analogReadResolution(12);  // 12-bit resolution (0-4095)
        analogSetAttenuation(ADC_11db);  // Full 0-3.3V range

        // Setup hardware timer for 1000 Hz sampling
        timer_ = timerBegin(0, 80, true);  // Timer 0, prescaler 80 (1 MHz), count up
        timerAttachInterrupt(timer_, &onTimerISR, true);  // Edge interrupt
        timerAlarmWrite(timer_, 1000, true);  // 1000 microseconds = 1000 Hz
        timerAlarmEnable(timer_);  // Enable timer
    }

    void stop() {
        if (timer_) {
            timerAlarmDisable(timer_);
            timerDetachInterrupt(timer_);
            timerEnd(timer_);
            timer_ = nullptr;
        }
    }

    // Process samples in main loop (not in ISR)
    void process() {
        if (!g_sample_flag) {
            return;
        }
        g_sample_flag = false;

        // Sample all enabled channels
        for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
            if (!channels[i].enabled) {
                continue;
            }

            uint16_t value;
            if (channels[i].is_analog) {
                if (channels[i].gpio_pin == 255) {
                    // Simulated channel
                    value = generateSimulatedValue(i);
                } else {
                    // Real ADC channel
                    value = analogRead(channels[i].gpio_pin);
                }
            } else {
                // Digital input
                value = digitalRead(channels[i].gpio_pin) ? 4095 : 0;
            }

            // Add to batch buffer
            sample_buffer_[sample_index_].channel_id = channels[i].channel_id;
            sample_buffer_[sample_index_].value = value;
            sample_index_++;

            // Send batch if full
            if (sample_index_ >= SAMPLES_PER_BATCH) {
                sendBatch();
            }
        }

        sim_time_++;  // Increment simulation time counter
    }

    // Force send any remaining samples
    void flush() {
        if (sample_index_ > 0) {
            sendBatch();
        }
    }

    static DataAcquisition* instance_;

private:
    hw_timer_t* timer_;
    uint32_t sequence_number_;
    Sample sample_buffer_[SAMPLES_PER_BATCH];
    size_t sample_index_;
    uint32_t sim_time_;  // For simulated waveform generation

    void sendBatch() {
        FrameBuilder frame;
        frame.begin(DATA_BATCH);

        // Add sequence number
        frame.addUint32(sequence_number_++);

        // Add number of samples
        frame.addByte((uint8_t)sample_index_);

        // Add samples (channel_id + value for each)
        for (size_t i = 0; i < sample_index_; i++) {
            frame.addByte(sample_buffer_[i].channel_id);
            frame.addUint16(sample_buffer_[i].value);
        }

        frame.send();
        sample_index_ = 0;  // Reset buffer
    }

    uint16_t generateSimulatedValue(uint8_t channel_idx) {
        // Generate different waveforms based on channel
        float time_sec = sim_time_ / 1000.0f;  // Convert to seconds
        float value_normalized = 0.5f;  // Default midpoint

        switch (channel_idx) {
            case 6:  // 1 Hz sine wave
                value_normalized = 0.5f + 0.5f * sin(2.0f * PI * 1.0f * time_sec);
                break;
            case 7:  // 10 Hz sine wave
                value_normalized = 0.5f + 0.5f * sin(2.0f * PI * 10.0f * time_sec);
                break;
            case 8:  // 5 Hz square wave
                value_normalized = (sin(2.0f * PI * 5.0f * time_sec) > 0) ? 1.0f : 0.0f;
                break;
            case 9:  // 2 Hz ramp/sawtooth
                value_normalized = fmod(time_sec * 2.0f, 1.0f);
                break;
            case 10:  // Noise
                value_normalized = random(0, 4096) / 4095.0f;
                break;
            default:
                value_normalized = 0.5f;
                break;
        }

        // Convert to 12-bit value (0-4095)
        return (uint16_t)(value_normalized * 4095.0f);
    }
};

// Static instance pointer for ISR access
DataAcquisition* DataAcquisition::instance_ = nullptr;
