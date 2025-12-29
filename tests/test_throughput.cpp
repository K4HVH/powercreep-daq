#include <windows.h>
#include <iostream>
#include <vector>
#include <string>
#include <chrono>
#include <iomanip>
#include <cstdint>
#include <map>

// --- Protocol v3 Constants ---
constexpr uint8_t SYNC1 = 0xAA;
constexpr uint8_t SYNC2 = 0x55;
constexpr uint8_t PROTOCOL_VERSION = 3;

constexpr uint8_t HANDSHAKE_REQ  = 0x01;
constexpr uint8_t HANDSHAKE_RESP = 0x02;
constexpr uint8_t DATA_BATCH     = 0x10;

constexpr size_t CRC_WINDOW_SIZE = 1024;  // v3: CRC windowing

// v3: DATA_TYPE enumeration (bits 3-5 of FLAGS)
enum DataType : uint8_t {
    DATA_UINT8  = 0b000,
    DATA_INT8   = 0b001,
    DATA_UINT16 = 0b010,
    DATA_INT16  = 0b011,
    DATA_INT24  = 0b100,
    DATA_UINT32 = 0b101,
    DATA_INT32  = 0b110,
    DATA_FLOAT32= 0b111
};

// Value size lookup table (indexed by DATA_TYPE)
constexpr uint8_t VALUE_SIZE_TABLE[8] = {
    1,  // DATA_UINT8
    1,  // DATA_INT8
    2,  // DATA_UINT16
    2,  // DATA_INT16
    3,  // DATA_INT24
    4,  // DATA_UINT32
    4,  // DATA_INT32
    4   // DATA_FLOAT32
};

inline uint8_t getValueSize(DataType data_type) {
    return VALUE_SIZE_TABLE[(uint8_t)data_type & 0x07];
}

inline DataType getDataType(uint8_t flags) {
    return (DataType)((flags >> 3) & 0x07);
}

// --- CRC16-CCITT Implementation ---
// Polynomial: 0x1021, Initial: 0xFFFF
static const uint16_t crc16_table[256] = {
    0x0000, 0x1021, 0x2042, 0x3063, 0x4084, 0x50A5, 0x60C6, 0x70E7,
    0x8108, 0x9129, 0xA14A, 0xB16B, 0xC18C, 0xD1AD, 0xE1CE, 0xF1EF,
    0x1231, 0x0210, 0x3273, 0x2252, 0x52B5, 0x4294, 0x72F7, 0x62D6,
    0x9339, 0x8318, 0xB37B, 0xA35A, 0xD3BD, 0xC39C, 0xF3FF, 0xE3DE,
    0x2462, 0x3443, 0x0420, 0x1401, 0x64E6, 0x74C7, 0x44A4, 0x5485,
    0xA56A, 0xB54B, 0x8528, 0x9509, 0xE5EE, 0xF5CF, 0xC5AC, 0xD58D,
    0x3653, 0x2672, 0x1611, 0x0630, 0x76D7, 0x66F6, 0x5695, 0x46B4,
    0xB75B, 0xA77A, 0x9719, 0x8738, 0xF7DF, 0xE7FE, 0xD79D, 0xC7BC,
    0x48C4, 0x58E5, 0x6886, 0x78A7, 0x0840, 0x1861, 0x2802, 0x3823,
    0xC9CC, 0xD9ED, 0xE98E, 0xF9AF, 0x8948, 0x9969, 0xA90A, 0xB92B,
    0x5AF5, 0x4AD4, 0x7AB7, 0x6A96, 0x1A71, 0x0A50, 0x3A33, 0x2A12,
    0xDBFD, 0xCBDC, 0xFBBF, 0xEB9E, 0x9B79, 0x8B58, 0xBB3B, 0xAB1A,
    0x6CA6, 0x7C87, 0x4CE4, 0x5CC5, 0x2C22, 0x3C03, 0x0C60, 0x1C41,
    0xEDAE, 0xFD8F, 0xCDEC, 0xDDCD, 0xAD2A, 0xBD0B, 0x8D68, 0x9D49,
    0x7E97, 0x6EB6, 0x5ED5, 0x4EF4, 0x3E13, 0x2E32, 0x1E51, 0x0E70,
    0xFF9F, 0xEFBE, 0xDFDD, 0xCFFC, 0xBF1B, 0xAF3A, 0x9F59, 0x8F78,
    0x9188, 0x81A9, 0xB1CA, 0xA1EB, 0xD10C, 0xC12D, 0xF14E, 0xE16F,
    0x1080, 0x00A1, 0x30C2, 0x20E3, 0x5004, 0x4025, 0x7046, 0x6067,
    0x83B9, 0x9398, 0xA3FB, 0xB3DA, 0xC33D, 0xD31C, 0xE37F, 0xF35E,
    0x02B1, 0x1290, 0x22F3, 0x32D2, 0x4235, 0x5214, 0x6277, 0x7256,
    0xB5EA, 0xA5CB, 0x95A8, 0x8589, 0xF56E, 0xE54F, 0xD52C, 0xC50D,
    0x34E2, 0x24C3, 0x14A0, 0x0481, 0x7466, 0x6447, 0x5424, 0x4405,
    0xA7DB, 0xB7FA, 0x8799, 0x97B8, 0xE75F, 0xF77E, 0xC71D, 0xD73C,
    0x26D3, 0x36F2, 0x0691, 0x16B0, 0x6657, 0x7676, 0x4615, 0x5634,
    0xD94C, 0xC96D, 0xF90E, 0xE92F, 0x99C8, 0x89E9, 0xB98A, 0xA9AB,
    0x5844, 0x4865, 0x7806, 0x6827, 0x18C0, 0x08E1, 0x3882, 0x28A3,
    0xCB7D, 0xDB5C, 0xEB3F, 0xFB1E, 0x8BF9, 0x9BD8, 0xABBB, 0xBB9A,
    0x4A75, 0x5A54, 0x6A37, 0x7A16, 0x0AF1, 0x1AD0, 0x2AB3, 0x3A92,
    0xFD2E, 0xED0F, 0xDD6C, 0xCD4D, 0xBDAA, 0xAD8B, 0x9DE8, 0x8DC9,
    0x7C26, 0x6C07, 0x5C64, 0x4C45, 0x3CA2, 0x2C83, 0x1CE0, 0x0CC1,
    0xEF1F, 0xFF3E, 0xCF5D, 0xDF7C, 0xAF9B, 0xBFBA, 0x8FD9, 0x9FF8,
    0x6E17, 0x7E36, 0x4E55, 0x5E74, 0x2E93, 0x3EB2, 0x0ED1, 0x1EF0
};

// v3: CRC with windowing (only first 1024 bytes if payload > 1024)
inline uint16_t crc16_ccitt(const uint8_t* data, size_t len) {
    uint16_t crc = 0xFFFF;
    size_t crc_len = (len <= CRC_WINDOW_SIZE) ? len : CRC_WINDOW_SIZE;
    for (size_t i = 0; i < crc_len; i++) {
        uint8_t index = (crc >> 8) ^ data[i];
        crc = (crc << 8) ^ crc16_table[index];
    }
    return crc;
}

// --- Serial Port Wrapper ---
class SerialPort {
public:
    SerialPort(const std::string& portName, DWORD baudRate) {
        std::string fullPortName = "\\\\.\\" + portName;
        hSerial = CreateFileA(fullPortName.c_str(),
                              GENERIC_READ | GENERIC_WRITE,
                              0,
                              0,
                              OPEN_EXISTING,
                              FILE_ATTRIBUTE_NORMAL,
                              0);

        if (hSerial == INVALID_HANDLE_VALUE) {
            throw std::runtime_error("Error opening serial port");
        }

        DCB dcbSerialParams = {0};
        dcbSerialParams.DCBlength = sizeof(dcbSerialParams);

        if (!GetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw std::runtime_error("Error getting serial state");
        }

        dcbSerialParams.BaudRate = baudRate;
        dcbSerialParams.ByteSize = 8;
        dcbSerialParams.StopBits = ONESTOPBIT;
        dcbSerialParams.Parity = NOPARITY;

        if (!SetCommState(hSerial, &dcbSerialParams)) {
            CloseHandle(hSerial);
            throw std::runtime_error("Error setting serial state");
        }

        COMMTIMEOUTS timeouts = {0};
        timeouts.ReadIntervalTimeout = 1;
        timeouts.ReadTotalTimeoutConstant = 1;
        timeouts.ReadTotalTimeoutMultiplier = 1;
        timeouts.WriteTotalTimeoutConstant = 1;
        timeouts.WriteTotalTimeoutMultiplier = 1;

        if (!SetCommTimeouts(hSerial, &timeouts)) {
            CloseHandle(hSerial);
            throw std::runtime_error("Error setting timeouts");
        }
    }

    ~SerialPort() {
        if (hSerial != INVALID_HANDLE_VALUE) {
            CloseHandle(hSerial);
        }
    }

    void write(const std::vector<uint8_t>& data) {
        DWORD bytesWritten;
        if (!WriteFile(hSerial, data.data(), (DWORD)data.size(), &bytesWritten, NULL)) {
            throw std::runtime_error("Error writing to serial port");
        }
    }

    int read(uint8_t* buffer, int size) {
        DWORD bytesRead;
        if (!ReadFile(hSerial, buffer, size, &bytesRead, NULL)) {
            return -1;
        }
        return (int)bytesRead;
    }

private:
    HANDLE hSerial;
};

// --- Frame Parser ---
enum ParserState {
    WAIT_SYNC1,
    WAIT_SYNC2,
    READ_LEN_LOW,
    READ_LEN_HIGH,
    READ_TYPE,
    READ_PAYLOAD,
    READ_CRC_LOW,
    READ_CRC_HIGH
};

struct Frame {
    uint8_t type;
    std::vector<uint8_t> payload;
};

class Parser {
public:
    Parser() : state(WAIT_SYNC1) {}

    // Returns true if a full frame was parsed
    bool feed(uint8_t byte, Frame& outFrame) {
        switch (state) {
            case WAIT_SYNC1:
                if (byte == SYNC1) state = WAIT_SYNC2;
                break;
            case WAIT_SYNC2:
                if (byte == SYNC2) state = READ_LEN_LOW;
                else state = WAIT_SYNC1;
                break;
            case READ_LEN_LOW:
                len_low = byte;
                state = READ_LEN_HIGH;
                break;
            case READ_LEN_HIGH:
                len_high = byte;
                payload_len = (uint16_t)len_low | ((uint16_t)len_high << 8);
                // v3: Support payloads up to 65535 bytes
                if (payload_len > 2048) { // Practical limit for this test
                    state = WAIT_SYNC1;
                } else {
                    state = READ_TYPE;
                }
                break;
            case READ_TYPE:
                frame_type = byte;
                payload_buffer.clear();
                payload_buffer.reserve(payload_len);
                if (payload_len == 0) {
                    state = READ_CRC_LOW;
                } else {
                    state = READ_PAYLOAD;
                }
                break;
            case READ_PAYLOAD:
                payload_buffer.push_back(byte);
                if (payload_buffer.size() == payload_len) {
                    state = READ_CRC_LOW;
                }
                break;
            case READ_CRC_LOW:
                crc_low = byte;
                state = READ_CRC_HIGH;
                break;
            case READ_CRC_HIGH:
                crc_high = byte;
                uint16_t received_crc = (uint16_t)crc_low | ((uint16_t)crc_high << 8);
                
                // Validate CRC
                // CRC is calculated over LEN_LOW, LEN_HIGH, TYPE, PAYLOAD
                std::vector<uint8_t> crc_data;
                crc_data.push_back(len_low);
                crc_data.push_back(len_high);
                crc_data.push_back(frame_type);
                crc_data.insert(crc_data.end(), payload_buffer.begin(), payload_buffer.end());
                
                uint16_t calculated_crc = crc16_ccitt(crc_data.data(), crc_data.size());
                
                state = WAIT_SYNC1; // Reset for next frame
                
                if (calculated_crc == received_crc) {
                    outFrame.type = frame_type;
                    outFrame.payload = payload_buffer;
                    return true;
                } else {
                    std::cerr << "CRC Mismatch! Recv: " << std::hex << received_crc << " Calc: " << calculated_crc << std::dec << std::endl;
                }
                break;
        }
        return false;
    }

private:
    ParserState state;
    uint8_t len_low, len_high;
    uint16_t payload_len;
    uint8_t frame_type;
    std::vector<uint8_t> payload_buffer;
    uint8_t crc_low, crc_high;
};

// --- Main ---
int main(int argc, char* argv[]) {
    if (argc < 2) {
        std::cerr << "Usage: " << argv[0] << " <COM_PORT>" << std::endl;
        return 1;
    }

    std::string portName = argv[1];
    DWORD baudRate = 2000000; // 2 Mbps

    std::cout << "Opening " << portName << " at " << baudRate << " baud..." << std::endl;

    try {
        SerialPort serial(portName, baudRate);
        Parser parser;

        // 1. Send Handshake Request (v3 format)
        std::cout << "Sending Handshake Request (v3)..." << std::endl;
        std::vector<uint8_t> req;
        req.push_back(SYNC1);
        req.push_back(SYNC2);
        req.push_back(0x02); // Len Low (2 bytes payload)
        req.push_back(0x00); // Len High
        req.push_back(HANDSHAKE_REQ); // Type
        req.push_back(0x03); // MAX_VERSION = 3
        req.push_back(0x03); // MIN_VERSION = 3

        // CRC over Len(2) + Type(1) + Payload(2)
        uint8_t crc_data[] = {0x02, 0x00, HANDSHAKE_REQ, 0x03, 0x03};
        uint16_t crc = crc16_ccitt(crc_data, 5);
        req.push_back(crc & 0xFF);
        req.push_back((crc >> 8) & 0xFF);

        serial.write(req);

        // 2. Wait for Handshake Response
        std::cout << "Waiting for Handshake Response..." << std::endl;
        bool handshakeReceived = false;
        uint8_t buffer[1024];
        std::map<uint8_t, DataType> channelDataTypes; // Map channel_id -> DATA_TYPE

        auto startWait = std::chrono::steady_clock::now();
        while (!handshakeReceived) {
            if (std::chrono::steady_clock::now() - startWait > std::chrono::seconds(5)) {
                std::cerr << "Timeout waiting for handshake!" << std::endl;
                return 1;
            }

            int n = serial.read(buffer, sizeof(buffer));
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    Frame frame;
                    if (parser.feed(buffer[i], frame)) {
                        if (frame.type == HANDSHAKE_RESP) {
                            std::cout << "Handshake Received!" << std::endl;

                            // Parse HANDSHAKE_RESP payload:
                            // [VERSION(1)][DEVICE_ID(4)][NUM_CHANNELS(1)][CHANNELS...][NUM_OUTPUTS(1)][OUTPUTS...]
                            if (frame.payload.size() >= 6) {
                                uint8_t version = frame.payload[0];
                                uint32_t deviceId = frame.payload[1] | (frame.payload[2] << 8) |
                                                   (frame.payload[3] << 16) | (frame.payload[4] << 24);
                                uint8_t numChannels = frame.payload[5];

                                std::cout << "Protocol Version: " << (int)version << std::endl;
                                std::cout << "Device ID: 0x" << std::hex << deviceId << std::dec << std::endl;
                                std::cout << "Number of Channels: " << (int)numChannels << std::endl;

                                // Parse channel configurations
                                // Format: [CHANNEL_ID(1)][FLAGS(1)][SAMPLE_RATE(2)][NAME_LEN(1)][NAME(n)][UNIT_LEN(1)][UNIT(n)]
                                size_t offset = 6;
                                for (int ch = 0; ch < numChannels; ch++) {
                                    if (offset + 5 <= frame.payload.size()) {
                                        uint8_t channelId = frame.payload[offset];
                                        uint8_t flags = frame.payload[offset + 1];
                                        uint16_t sampleRateHz = frame.payload[offset + 2] | (frame.payload[offset + 3] << 8);
                                        uint8_t nameLen = frame.payload[offset + 4];

                                        // Skip NAME string
                                        offset += 5 + nameLen;

                                        if (offset >= frame.payload.size()) break;

                                        uint8_t unitLen = frame.payload[offset];

                                        // Extract DATA_TYPE from FLAGS (bits 3-5)
                                        DataType dataType = getDataType(flags);
                                        channelDataTypes[channelId] = dataType;

                                        // Skip UNIT_LEN + UNIT string for next channel
                                        offset += 1 + unitLen;

                                        std::cout << "  CH" << (int)channelId << ": " << sampleRateHz
                                                  << "Hz, DataType=" << (int)dataType
                                                  << " (" << (int)getValueSize(dataType) << " bytes)" << std::endl;
                                    }
                                }
                            }

                            handshakeReceived = true;
                            break;
                        }
                    }
                }
            }
        }

        // 3. Throughput Test Loop
        std::cout << "Starting Throughput Test (running for 10 seconds)..." << std::endl;
        
        uint64_t totalBytes = 0;
        uint64_t totalFrames = 0;
        std::map<uint8_t, uint64_t> channelSamples;
        
        auto startTime = std::chrono::steady_clock::now();
        auto lastReportTime = startTime;
        const auto testDuration = std::chrono::seconds(10);

        while (std::chrono::steady_clock::now() - startTime < testDuration) {
            int n = serial.read(buffer, sizeof(buffer));
            if (n > 0) {
                for (int i = 0; i < n; i++) {
                    Frame frame;
                    if (parser.feed(buffer[i], frame)) {
                        if (frame.type == DATA_BATCH) {
                            totalBytes += frame.payload.size();
                            totalFrames++;

                            // Parse DATA_BATCH payload (v3 variable-length)
                            // [SeqNum(4)][Count(1)][Sample1][Sample2]...[SampleN]
                            // Each sample: [ChannelID(1)][Value(1-4 bytes based on DATA_TYPE)]
                            if (frame.payload.size() >= 5) {
                                uint8_t sampleCount = frame.payload[4];
                                size_t offset = 5;

                                for (int s = 0; s < sampleCount; s++) {
                                    if (offset < frame.payload.size()) {
                                        uint8_t chId = frame.payload[offset];
                                        offset++; // Move past channel ID

                                        // Look up DATA_TYPE for this channel
                                        auto it = channelDataTypes.find(chId);
                                        if (it != channelDataTypes.end()) {
                                            uint8_t valueSize = getValueSize(it->second);

                                            // Skip the value bytes (we don't need to parse the actual value for throughput test)
                                            if (offset + valueSize <= frame.payload.size()) {
                                                channelSamples[chId]++;
                                                offset += valueSize;
                                            } else {
                                                std::cerr << "Warning: Incomplete sample for CH" << (int)chId << std::endl;
                                                break;
                                            }
                                        } else {
                                            std::cerr << "Warning: Unknown channel ID " << (int)chId << std::endl;
                                            break; // Unknown channel, can't determine value size
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
            }

            auto now = std::chrono::steady_clock::now();
            auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - lastReportTime).count();
            
            if (elapsed >= 1000) {
                double mbps = (double)totalBytes / (1024.0 * 1024.0);
                
                std::cout << "--- Report ---" << std::endl;
                std::cout << "Total Throughput: " << std::fixed << std::setprecision(2) << mbps << " MB/s (" 
                          << totalFrames << " frames/s)" << std::endl;
                
                std::cout << "Channel Sample Rates:" << std::endl;
                for (const auto& pair : channelSamples) {
                    std::cout << "  CH " << (int)pair.first << ": " << pair.second << " samples/s" << std::endl;
                }
                std::cout << "--------------" << std::endl;

                totalBytes = 0;
                totalFrames = 0;
                channelSamples.clear();
                lastReportTime = now;
            }
        }

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
