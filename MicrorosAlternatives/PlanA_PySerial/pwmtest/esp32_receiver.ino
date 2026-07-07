#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>

// --- JETSON -> ESP32 PROTOBUF INVERSION STRUCTURES ---
typedef struct {
    int32_t left_pwm;
    int32_t right_pwm;
} PwmCommand;

typedef struct {
    pb_size_t which_packet_type;
    union {
        PwmCommand pwm;
    } packet_type;
} JetsonToEspPacket;

#define JetsonToEspPacket_pwm_tag 1

const pb_field_t PwmCommand_fields = {
    PB_FIELD(1, INT32, REQUIRED, STATIC, FIRST, PwmCommand, left_pwm, left_pwm, 0),
    PB_FIELD(2, INT32, REQUIRED, STATIC, AFTER, PwmCommand, right_pwm, left_pwm, 0),
    PB_LAST_FIELD
};

const pb_field_t JetsonToEspPacket_fields = {
    PB_ONEOF_FIELD(packet_type, 1, MESSAGE, ONEOF, STATIC, FIRST, JetsonToEspPacket, pwm, pwm, &PwmCommand_fields),
    PB_LAST_FIELD
};

// --- ESP32 -> JETSON DATA BACK-STREAM DEFINITIONS ---
typedef struct {
    char text[80]; // Fixed safety buffer boundary for memory stability
} LogMessage;

typedef struct {
    pb_size_t which_packet_type;
    union {
        LogMessage log;
    } packet_type;
} EspToJetsonPacket;

#define EspToJetsonPacket_log_tag 1

const pb_field_t LogMessage_fields = {
    PB_FIELD(1, STRING, REQUIRED, STATIC, FIRST, LogMessage, text, text, 0),
    PB_LAST_FIELD
};

const pb_field_t EspToJetsonPacket_fields = {
    PB_ONEOF_FIELD(packet_type, 1, MESSAGE, ONEOF, STATIC, FIRST, EspToJetsonPacket, log, log, &LogMessage_fields),
    PB_LAST_FIELD
};

// --- SERIAL PROCESSING STORAGE BUFFERS ---
uint8_t serial_rx_buffer[128];
size_t rx_buffer_index = 0;

// --- COBS TRANSFORMATION DECODER ---
size_t decode_cobs_frame(const uint8_t* src, size_t src_len, uint8_t* dst) {
    size_t r_idx = 0, w_idx = 0;
    while (r_idx < src_len) {
        uint8_t code = src[r_idx++];
        for (uint8_t i = 1; i < code; i++) {
            dst[w_idx++] = src[r_idx++];
        }
        if (code < 0xFF && r_idx < src_len) {
            dst[w_idx++] = 0x00;
        }
    }
    return w_idx;
}

// --- COBS TRANSFORMATION ENCODER ---
size_t encode_cobs_frame(const uint8_t* src, size_t src_len, uint8_t* dst) {
    size_t read_index = 0, write_index = 1, code_index = 0;
    uint8_t code = 1;
    while (read_index < src_len) {
        if (src[read_index] == 0x00) {
            dst[code_index] = code; code = 1; code_index = write_index++; read_index++;
        } else {
            dst[write_index++] = src[read_index++]; code++;
            if (code == 0xFF) { dst[code_index] = code; code = 1; code_index = write_index++; }
        }
    }
    dst[code_index] = code; return write_index;
}

// --- ZERO-DELAY JETSON LOG FLUSHER ---
void send_instant_log_to_jetson(const char* text_msg) {
    uint8_t raw_buf[128];
    uint8_t cobs_buf[130];

    EspToJetsonPacket tx_packet = {0};
    tx_packet.which_packet_type = EspToJetsonPacket_log_tag;
    strncpy(tx_packet.packet_type.log.text, text_msg, sizeof(tx_packet.packet_type.log.text) - 1);

    pb_ostream_t stream = pb_ostream_from_buffer(raw_buf, sizeof(raw_buf));
    if (!pb_encode(&stream, EspToJetsonPacket_fields, &tx_packet)) return;

    size_t len = encode_cobs_frame(raw_buf, stream.bytes_written, cobs_buf);
    cobs_buf[len++] = 0x00; // Append structural frame boundary identifier

    Serial.write(cobs_buf, len);
    Serial.flush(); // Force immediate physical hardware pin shift execution
}

// --- PACKET DECONSTRUCTION MANAGER ---
void parse_incoming_buffer(uint8_t* buffer, size_t size) {
    uint8_t decoded_payload[128];
    size_t decoded_length = decode_cobs_frame(buffer, size, decoded_payload);

    JetsonToEspPacket rx_packet = {0};
    pb_istream_t stream = pb_istream_from_buffer(decoded_payload, decoded_length);

    if (pb_decode(&stream, JetsonToEspPacket_fields, &rx_packet)) {
        if (rx_packet.which_packet_type == JetsonToEspPacket_pwm_tag) {
            int32_t left = rx_packet.packet_type.pwm.left_pwm;
            int32_t right = rx_packet.packet_type.pwm.right_pwm;

            // Generate direct string response payload internally
            char response_msg[64];
            snprintf(response_msg, sizeof(response_msg), "ACK! Received PWM values -> Left: %d | Right: %d", left, right);
            
            // Dispatch confirmation instantly
            send_instant_log_to_jetson(response_msg);
        }
    }
}

void setup() {
    Serial.begin(115200);
}

void loop() {
    // Process input data stream on every single pass
    while (Serial.available() > 0) {
        uint8_t input_byte = Serial.read();

        if (input_byte == 0x00) {
            if (rx_buffer_index > 0) {
                parse_incoming_buffer(serial_rx_buffer, rx_buffer_index);
                rx_buffer_index = 0; // Clear index instantly
            }
        } else {
            if (rx_buffer_index < sizeof(serial_rx_buffer)) {
                serial_rx_buffer[rx_buffer_index++] = input_byte;
            }
        }
    }
}
