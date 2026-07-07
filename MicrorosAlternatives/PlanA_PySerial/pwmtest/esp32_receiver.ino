#include <Arduino.h>
#include <pb_encode.h>
#include <pb_decode.h>

// --- STRUCTURES ---
typedef struct {
    int32_t left_pwm;
    int32_t right_pwm;
} PwmCommand;

typedef struct {
    char text[64]; 
} LogMessage;

typedef struct {
    pb_size_t which_packet_type;
    union {
        PwmCommand pwm;
    } packet_type;
} JetsonToEspPacket;

typedef struct {
    pb_size_t which_packet_type;
    union {
        LogMessage log;
    } packet_type;
} EspToJetsonPacket;

// --- FIELD DESCRIPTIONS ---
const pb_field_t PwmCommand_fields[3] = {
    PB_FIELD(  1, INT32   , REQUIRED, STATIC  , FIRST, PwmCommand, left_pwm, left_pwm, 0),
    PB_FIELD(  2, INT32   , REQUIRED, STATIC  , OTHER, PwmCommand, right_pwm, left_pwm, 0),
    PB_LAST_FIELD
};

const pb_field_t LogMessage_fields[2] = {
    PB_FIELD(  1, STRING  , REQUIRED, STATIC  , FIRST, LogMessage, text, text, 0),
    PB_LAST_FIELD
};

const pb_field_t JetsonToEspPacket_fields[2] = {
    PB_ONEOF_FIELD(packet_type,   1, MESSAGE , ONEOF, STATIC  , FIRST, JetsonToEspPacket, pwm, pwm, &PwmCommand_fields),
    PB_LAST_FIELD
};

const pb_field_t EspToJetsonPacket_fields[2] = {
    PB_ONEOF_FIELD(packet_type,   1, MESSAGE , ONEOF, STATIC  , FIRST, EspToJetsonPacket, log, log, &LogMessage_fields),
    PB_LAST_FIELD
};

uint8_t serial_rx_buffer[128];
size_t rx_buffer_index = 0;

// Hardened COBS Decoder
size_t decode_cobs_frame(const uint8_t* src, size_t src_len, uint8_t* dst, size_t dst_max_len) {
    size_t r_idx = 0, w_idx = 0;
    while (r_idx < src_len) {
        uint8_t code = src[r_idx++];
        if (code == 0 || r_idx + code - 1 > src_len || w_idx + code - 1 > dst_max_len) {
            return 0; 
        }
        for (uint8_t i = 1; i < code; i++) {
            dst[w_idx++] = src[r_idx++];
        }
        if (code < 0xFF && r_idx < src_len) {
            if (w_idx >= dst_max_len) return 0;
            dst[w_idx++] = 0x00;
        }
    }
    return w_idx;
}

void parse_incoming_buffer(uint8_t* buffer, size_t size) {
    uint8_t decoded_payload[128];
    size_t decoded_length = decode_cobs_frame(buffer, size, decoded_payload, sizeof(decoded_payload));

    if (decoded_length == 0) return; 

    JetsonToEspPacket rx_packet = {0};
    pb_istream_t stream = pb_istream_from_buffer(decoded_payload, decoded_length);

    if (pb_decode(&stream, JetsonToEspPacket_fields, &rx_packet)) {
        // MATCH: checks against field tag 1 (pwm)
        if (rx_packet.which_packet_type == 1) { 
            int32_t left = rx_packet.packet_type.pwm.left_pwm;
            int32_t right = rx_packet.packet_type.pwm.right_pwm;
            
            // --- YOUR MOTOR CODE HERE ---
            // Example: analogWrite(LEFT_PIN, left);
        }
    }
}

void setup() {
    Serial.begin(115200);
    delay(500);
    while(Serial.available() > 0) { Serial.read(); } 
}

void loop() {
    while (Serial.available() > 0) {
        uint8_t input_byte = Serial.read();

        if (input_byte == 0x00) {
            if (rx_buffer_index > 0) {
                parse_incoming_buffer(serial_rx_buffer, rx_buffer_index);
                rx_buffer_index = 0; 
            }
        } else {
            if (rx_buffer_index < sizeof(serial_rx_buffer)) {
                serial_rx_buffer[rx_buffer_index++] = input_byte;
            } else {
                rx_buffer_index = 0; 
            }
        }
    }
}