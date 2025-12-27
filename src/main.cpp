#include <Arduino.h>
#include "protocol.h"
#include "frameParser.h"
#include "frameBuilder.h"
#include "deviceConfig.h"
#include "commandHandler.h"
#include "handshake.h"

// ======================== DUAL-CORE ARCHITECTURE ========================
// Core 0 (PRO_CPU):  Communication - Serial RX/TX, frame parsing, commands
// Core 1 (APP_CPU):  Data Acquisition - ADC sampling, batching, simulation
// ========================================================================

// FreeRTOS task handles
TaskHandle_t commTaskHandle = NULL;
TaskHandle_t daqTaskHandle = NULL;

// Queue for sending DATA_BATCH frames from DAQ to Comm task
QueueHandle_t dataBatchQueue = NULL;
constexpr size_t DATA_BATCH_QUEUE_SIZE = 10;

// Structure for passing data batches between cores
struct DataBatchMessage {
    uint32_t sequence_number;
    uint8_t num_samples;
    struct {
        uint8_t channel_id;
        uint16_t value;
    } samples[10];
};

// Heartbeat state
Heartbeat heartbeat;

// Frame parser
FrameParser parser;

// Handshake completion flag (prevents data transmission until handshake done)
volatile bool g_handshake_complete = false;

// ======================== DATA ACQUISITION TASK (Core 1) ========================
constexpr size_t SAMPLES_PER_BATCH = 10;

// Pre-computed list of enabled channel indices (optimization)
uint8_t enabled_channels[NUM_CHANNELS];
uint8_t num_enabled_channels = 0;

struct DaqState {
    hw_timer_t* timer;
    uint32_t sequence_number;
    struct {
        uint8_t channel_id;
        uint16_t value;
    } sample_buffer[SAMPLES_PER_BATCH];
    size_t sample_index;
    uint32_t sim_time;
};

volatile bool DRAM_ATTR g_daq_sample_flag = false;

void IRAM_ATTR daqTimerISR() {
    g_daq_sample_flag = true;
}

uint16_t IRAM_ATTR generateSimulatedValue(uint8_t channel_idx, uint32_t sim_time) {
    float time_sec = sim_time / 1000.0f;
    float value_normalized = 0.5f;

    switch (channel_idx) {
        case 6:  // 1 Hz sine
            value_normalized = 0.5f + 0.5f * sin(2.0f * PI * 1.0f * time_sec);
            break;
        case 7:  // 10 Hz sine
            value_normalized = 0.5f + 0.5f * sin(2.0f * PI * 10.0f * time_sec);
            break;
        case 8:  // 5 Hz square
            value_normalized = (sin(2.0f * PI * 5.0f * time_sec) > 0) ? 1.0f : 0.0f;
            break;
        case 9:  // 2 Hz ramp
            value_normalized = fmod(time_sec * 2.0f, 1.0f);
            break;
        case 10:  // Noise
            value_normalized = random(0, 4096) / 4095.0f;
            break;
    }

    return (uint16_t)(value_normalized * 4095.0f);
}

void daqTask(void* parameter) {
    DaqState state = {nullptr, 0, {}, 0, 0};

    // Configure ADC
    analogReadResolution(12);
    initializeADC();  // Configure per-channel attenuation

    // Pre-compute enabled channels list (optimization)
    for (uint8_t i = 0; i < NUM_CHANNELS; i++) {
        if (channels[i].enabled) {
            enabled_channels[num_enabled_channels++] = i;
        }
    }

    // Setup hardware timer for 1000 Hz
    state.timer = timerBegin(0, 80, true);
    timerAttachInterrupt(state.timer, &daqTimerISR, true);
    timerAlarmWrite(state.timer, 1000, true);
    timerAlarmEnable(state.timer);

    while (true) {
        // Wait for timer flag
        if (g_daq_sample_flag) {
            g_daq_sample_flag = false;

            // Sample all enabled channels (using pre-computed list for speed)
            for (uint8_t idx = 0; idx < num_enabled_channels; idx++) {
                uint8_t i = enabled_channels[idx];
                const ChannelConfig& ch = channels[i];

                uint16_t value;
                if (ch.is_analog) {
                    if (ch.gpio_pin == 255) {
                        value = generateSimulatedValue(i, state.sim_time);
                    } else {
                        value = analogRead(ch.gpio_pin);
                    }
                } else {
                    value = digitalRead(ch.gpio_pin) ? 4095 : 0;
                }

                // Add to batch buffer
                state.sample_buffer[state.sample_index].channel_id = ch.channel_id;
                state.sample_buffer[state.sample_index].value = value;
                state.sample_index++;

                // Send batch if full
                if (state.sample_index >= SAMPLES_PER_BATCH) {
                    // Only send data after handshake is complete
                    if (g_handshake_complete) {
                        DataBatchMessage msg;
                        msg.sequence_number = state.sequence_number++;
                        msg.num_samples = state.sample_index;
                        memcpy(msg.samples, state.sample_buffer, sizeof(msg.samples));

                        // Send to comm task (non-blocking)
                        xQueueSend(dataBatchQueue, &msg, 0);
                    }

                    state.sample_index = 0;
                }
            }

            state.sim_time++;
        }

        // Yield to other tasks
        vTaskDelay(pdMS_TO_TICKS(1));
    }
}

// ======================== COMMUNICATION TASK (Core 0) ========================

void onFrameReceived(uint8_t frame_type, const uint8_t* payload, uint8_t payload_len) {
    switch (frame_type) {
        case HANDSHAKE_REQ:
            Handshake::sendHandshakeResponse();
            g_handshake_complete = true;  // Enable data transmission
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

void commTask(void* parameter) {
    parser.setCallback(onFrameReceived);
    heartbeat.begin();

    unsigned long last_heartbeat = millis();

    while (true) {
        // Process incoming serial data
        parser.processSerial();

        // Send heartbeats every 1 second
        if (millis() - last_heartbeat >= 1000) {
            heartbeat.process();
            last_heartbeat = millis();
        }

        // Check for outgoing data batches from DAQ task
        DataBatchMessage msg;
        if (xQueueReceive(dataBatchQueue, &msg, 0) == pdTRUE) {
            // Build and send DATA_BATCH frame
            FrameBuilder frame;
            frame.begin(DATA_BATCH);
            frame.addUint32(msg.sequence_number);
            frame.addByte(msg.num_samples);

            for (uint8_t i = 0; i < msg.num_samples; i++) {
                frame.addByte(msg.samples[i].channel_id);
                frame.addUint16(msg.samples[i].value);
            }

            frame.send();
        }

        // Small delay to prevent watchdog timeout
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

    // Create queue for inter-core communication
    dataBatchQueue = xQueueCreate(DATA_BATCH_QUEUE_SIZE, sizeof(DataBatchMessage));

    if (dataBatchQueue == NULL) {
        return;  // Fatal error - cannot continue
    }

    // Create tasks pinned to specific cores
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
        3,                  // Priority (higher than comm - time-critical)
        &daqTaskHandle,     // Task handle
        1                   // Core 1 (APP_CPU) - Data acquisition
    );
}

void loop() {
    // Nothing here - all work done in FreeRTOS tasks
    vTaskDelay(pdMS_TO_TICKS(1000));
}
