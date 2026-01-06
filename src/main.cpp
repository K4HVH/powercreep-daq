#include <Arduino.h>
#include "protocol.h"
#include "frameParser.h"
#include "frameBuilder.h"
#include "deviceConfig.h"
#include "commandHandler.h"
#include "handshake.h"
#include "acquisition.h"

// ======================== DUAL-CORE ARCHITECTURE ========================
// Core 0 (PRO_CPU):  Communication - Serial RX/TX, frame parsing, commands
// Core 1 (APP_CPU):  Data Acquisition - ADC sampling, batching, simulation
// ========================================================================

// FreeRTOS task handles
TaskHandle_t commTaskHandle = NULL;
TaskHandle_t daqTaskHandle = NULL;
TaskHandle_t sensorPollingTaskHandle = NULL;

// v3: No queue needed - DAQ task sends DATA_BATCH frames directly via Serial
// This eliminates inter-core communication overhead
// v3.1: Sensor polling moved to background task to avoid I2C/SPI timeout impact

// Heartbeat state
Heartbeat heartbeat;

// Frame parser
FrameParser parser;

// Handshake completion flag (prevents data transmission until handshake done)
volatile bool g_handshake_complete = false;

// Sequence number for DATA_BATCH frames (MUST reset to 0 after each HANDSHAKE_RESP)
volatile uint32_t g_sequence_number = 0;

// Track last DATA_BATCH send time for heartbeat logic (v3 requirement)
volatile unsigned long g_last_data_batch_ms = 0;

// ======================== DATA ACQUISITION TASK (Core 1) ========================

// v3: Pre-computed list of enabled channel indices (O(1) lookup optimization)
uint8_t enabled_channels[NUM_CHANNELS];
uint8_t num_enabled_channels = 0;

// v3: Batch sending parameters
constexpr uint8_t MAX_SAMPLES_PER_BATCH = 5;  // Max samples per batch (low for responsiveness)
constexpr uint32_t BATCH_SEND_INTERVAL_MS = 2;  // Send batch every 2ms for near real-time updates (500 batches/sec)

void IRAM_ATTR daqTimerISR() {
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(daqTaskHandle, &xHigherPriorityTaskWoken);
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
}

// v3: Completely rewritten DAQ task with variable-length sample support
void daqTask(void* parameter) {
    hw_timer_t* timer = nullptr;
    uint32_t tick_count = 0;
    unsigned long last_batch_send = 0;

    // v3: Variable-length batch buffer (channel_id + value based on data_type)
    uint8_t batch_buffer[256];  // Raw bytes for variable-length samples
    size_t batch_bytes = 0;
    uint8_t batch_count = 0;

    // Configure ADC
    analogReadResolution(12);
    initializeADC();  // Configure per-channel attenuation

    // Initialize acquisition subsystem
    Acquisition::init();

    // Pre-initialize sensors for all enabled channels (removes init overhead from hot path)
    Acquisition::initializeSensors(channels, NUM_CHANNELS);

    // Pre-compute enabled channels list (O(n) optimization)
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (channels[i].enabled) {
            enabled_channels[num_enabled_channels++] = i;
        }
    }

    // Setup hardware timer for 1000 Hz base rate
    timer = timerBegin(0, 80, true);
    timerAttachInterrupt(timer, &daqTimerISR, true);
    timerAlarmWrite(timer, 1000, true);  // 1ms per tick (1000 Hz)
    timerAlarmEnable(timer);

    last_batch_send = millis();

    while (true) {
        // Wait for timer notification (1000 Hz tick)
        if (ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) {

            // Periodic batch sending FIRST (even if not full) - ensures batches sent every 2ms
            // This prevents accumulation of samples when channels fire together
            if (g_handshake_complete && batch_count > 0 &&
                (millis() - last_batch_send) >= BATCH_SEND_INTERVAL_MS) {
                FrameBuilder frame;
                frame.begin(DATA_BATCH);
                frame.addUint32(g_sequence_number++);
                frame.addByte(batch_count);
                frame.addBytes(batch_buffer, batch_bytes);
                frame.send();
                last_batch_send = millis();
                g_last_data_batch_ms = last_batch_send;  // Track for heartbeat logic
                batch_bytes = 0;
                batch_count = 0;
            }

            // Sample all enabled channels
            for (uint8_t idx = 0; idx < num_enabled_channels; idx++) {
                uint8_t i = enabled_channels[idx];
                const ChannelConfig& ch = channels[i];

                // Decimation: check if it's time to sample this channel
                // Base rate is 1000Hz, interval = 1000 / sample_rate
                uint32_t interval = 1000 / ch.default_sample_rate_hz;
                if (interval == 0) interval = 1;  // Safety
                if (tick_count % interval != 0) continue;

                // Read value using unified acquisition interface
                uint32_t raw_value = Acquisition::read(ch, channels, NUM_CHANNELS);

                // Add sample to batch buffer (variable-length based on data_type)
                uint8_t value_size = getValueSize(ch.data_type);

                // Check if batch would overflow
                if (batch_count >= MAX_SAMPLES_PER_BATCH || (batch_bytes + 1 + value_size) > sizeof(batch_buffer)) {
                    // Send current batch before adding new sample
                    if (g_handshake_complete && batch_count > 0) {
                        FrameBuilder frame;
                        frame.begin(DATA_BATCH);
                        frame.addUint32(g_sequence_number++);
                        frame.addByte(batch_count);
                        frame.addBytes(batch_buffer, batch_bytes);
                        frame.send();
                        last_batch_send = millis();
                        g_last_data_batch_ms = last_batch_send;  // Track for heartbeat logic
                    }
                    batch_bytes = 0;
                    batch_count = 0;
                }

                // Add channel ID
                batch_buffer[batch_bytes++] = ch.channel_id;

                // Add value (little-endian, variable-length based on data_type)
                switch (value_size) {
                    case 1:  // 8-bit
                        batch_buffer[batch_bytes++] = (uint8_t)raw_value;
                        break;
                    case 2:  // 16-bit
                        batch_buffer[batch_bytes++] = (uint8_t)(raw_value & 0xFF);
                        batch_buffer[batch_bytes++] = (uint8_t)((raw_value >> 8) & 0xFF);
                        break;
                    case 3:  // 24-bit
                        batch_buffer[batch_bytes++] = (uint8_t)(raw_value & 0xFF);
                        batch_buffer[batch_bytes++] = (uint8_t)((raw_value >> 8) & 0xFF);
                        batch_buffer[batch_bytes++] = (uint8_t)((raw_value >> 16) & 0xFF);
                        break;
                    case 4:  // 32-bit (or float)
                        batch_buffer[batch_bytes++] = (uint8_t)(raw_value & 0xFF);
                        batch_buffer[batch_bytes++] = (uint8_t)((raw_value >> 8) & 0xFF);
                        batch_buffer[batch_bytes++] = (uint8_t)((raw_value >> 16) & 0xFF);
                        batch_buffer[batch_bytes++] = (uint8_t)((raw_value >> 24) & 0xFF);
                        break;
                }
                batch_count++;
            }

            tick_count++;
        }
    }
}

// ======================== SENSOR POLLING TASK (Core 0, Background) ========================

// Global state for software PWM control
volatile uint32_t g_soft_pwm_freq = 1000; // Default 1kHz
volatile uint8_t g_soft_pwm_duty_percent = 0;
volatile bool g_pwm_initialized = false;

void setSoftwarePWM(uint16_t freq, uint8_t duty_percent) {
    g_soft_pwm_freq = freq;
    g_soft_pwm_duty_percent = duty_percent;
}

void updatePWMOutput() {
    // Pin definitions from deviceConfig.h
    // Switch A: 15, Switch B: 27, Pot: 32, PWM: 25
    const uint8_t PIN_SWITCH_A = 15; 
    const uint8_t PIN_SWITCH_B = 27; 
    const uint8_t PIN_POT = 32;
    const uint8_t PIN_PWM = 25;
    
    // LEDC Configuration
    const uint8_t LEDC_CHANNEL = PIN_PWM % 16;
    const uint8_t LEDC_RESOLUTION = 10; // 10-bit (0-1023)

    // Read switches (Active LOW for INPUT_PULLUP)
    bool mode_manual = (digitalRead(PIN_SWITCH_A) == LOW);
    bool mode_software = (digitalRead(PIN_SWITCH_B) == LOW);

    uint32_t target_freq = g_soft_pwm_freq;
    uint32_t target_duty = 0;

    if (mode_software) {
        // Software control (Switch B)
        target_freq = g_soft_pwm_freq;
        target_duty = (g_soft_pwm_duty_percent * 1023) / 100;
    } else if (mode_manual) {
        // Manual control (Switch A) -> Potentiometer
        // Read Pot (0-4095)
        uint32_t pot_val = analogRead(PIN_POT);
        // Map to 10-bit PWM (0-1023)
        target_duty = pot_val / 4; 
    } else {
        // OFF (Neither) -> PWM 0
        target_duty = 0;
    }

    // Initialize or Update LEDC
    static uint32_t last_freq = 0;
    
    if (!g_pwm_initialized || last_freq != target_freq) {
        ledcSetup(LEDC_CHANNEL, target_freq, LEDC_RESOLUTION);
        ledcAttachPin(PIN_PWM, LEDC_CHANNEL);
        last_freq = target_freq;
        g_pwm_initialized = true;
    }

    ledcWrite(LEDC_CHANNEL, target_duty);
}

// Background task that polls complex sensors (HX711, I2C, SPI) and updates cache.
// This allows DAQ task to read cached values instantly without I2C/SPI timeout overhead.
// Runs at low priority so it doesn't interfere with time-critical DAQ or comm tasks.
void sensorPollingTask(void* parameter) {
    while (true) {
        // Update PWM output based on switch state
        updatePWMOutput();

        // Poll all enabled complex sensors (handles I2C/SPI timeouts gracefully)
        Acquisition::pollSensors(channels, NUM_CHANNELS);

        // Yield to other tasks (poll every 10ms - fast enough for even 100Hz sensors)
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ======================== COMMUNICATION TASK (Core 0) ========================

void onFrameReceived(uint8_t frame_type, const uint8_t* payload, uint8_t payload_len) {
    switch (frame_type) {
        case HANDSHAKE_REQ:
            // MUST validate version compatibility (v3 spec requirement)
            if (payload_len >= 2) {
                uint8_t max_version = payload[0];
                uint8_t min_version = payload[1];

                // Check if our protocol version (3) is within requested range
                if (PROTOCOL_VERSION >= min_version && PROTOCOL_VERSION <= max_version) {
                    Handshake::sendHandshakeResponse();
                    g_sequence_number = 0;  // MUST reset sequence to 0 after HANDSHAKE_RESP
                    g_handshake_complete = true;  // Enable data transmission
                } else {
                    // Incompatible version - send ERROR frame
                    CommandHandler::sendError(ERROR_INVALID_COMMAND, "Protocol version mismatch");
                    g_handshake_complete = false;
                }
            } else {
                // Invalid HANDSHAKE_REQ payload
                CommandHandler::sendError(ERROR_INVALID_COMMAND, "Invalid handshake request");
                g_handshake_complete = false;
            }
            break;

        case HEARTBEAT_ACK:
            heartbeat.handleAck();
            break;

        case CMD_GPIO:
            CommandHandler::handleGPIO(payload, payload_len);
            break;

        case CMD_PWM:
            CommandHandler::handlePWM(payload, payload_len);
            break;

        case CMD_DAC:
            CommandHandler::handleDAC(payload, payload_len);
            break;

        case CMD_CONFIG:
            CommandHandler::handleConfig(payload, payload_len);
            break;

        default:
            CommandHandler::sendError(ERROR_INVALID_COMMAND, "Unknown command");
            break;
    }
}

// v3: Simplified comm task (DAQ task sends frames directly, no queue needed)
void commTask(void* parameter) {
    parser.setCallback(onFrameReceived);
    heartbeat.begin();

    unsigned long last_heartbeat = millis();

    while (true) {
        // Process incoming serial data
        parser.processSerial();

        // Send heartbeats (only if no DATA_BATCH sent recently - v3 requirement)
        if (millis() - last_heartbeat >= 1000) {
            heartbeat.process(g_last_data_batch_ms);
            last_heartbeat = millis();
        }

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}
// ======================== ARDUINO SETUP/LOOP ========================

void setup() {
    // Initialize serial at 2 Mbaud
    Serial.begin(2000000);
    delay(100);  // Brief delay for serial to stabilize

    // Initialize GPIO
    initializeDigitalInputs();
    initializeOutputs();

    // v3: No queue needed - DAQ task sends frames directly

    // Create tasks pinned to specific cores
    xTaskCreatePinnedToCore(
        sensorPollingTask,  // Task function
        "SensorPoll",       // Task name
        8192,               // Stack size (bytes)
        NULL,               // Parameters
        1,                  // Priority (lowest - background task)
        &sensorPollingTaskHandle, // Task handle
        0                   // Core 0 (PRO_CPU) - Background polling
    );

    xTaskCreatePinnedToCore(
        commTask,           // Task function
        "CommTask",         // Task name
        8192,               // Stack size (bytes)
        NULL,               // Parameters
        2,                  // Priority (higher = more important)
        &commTaskHandle,    // Task handle
        0                   // Core 0 (PRO_CPU) - Communication
    );

    xTaskCreatePinnedToCore(
        daqTask,            // Task function
        "DaqTask",          // Task name
        8192,               // Stack size (bytes)
        NULL,               // Parameters
        3,                  // Priority (highest - time-critical)
        &daqTaskHandle,     // Task handle
        1                   // Core 1 (APP_CPU) - Data acquisition
    );
}

void loop() {
    // Nothing here - all work done in FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
