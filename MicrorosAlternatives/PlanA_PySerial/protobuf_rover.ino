#include <Arduino.h>
#include <pb_decode.h>
#include <pb_encode.h>

// --- HARDWARE CONFIGURATION ---
#define LEFT_MOTOR_PWM_PIN  4
#define RIGHT_MOTOR_PWM_PIN 5
#define BATTERY_PIN         1
#define STATUS_LED_PIN      2

// --- NATIVE PROTOBUF STRUCTURE BLUEPRINTS ---
typedef struct {
    float linear_x;
    float angular_z;
} MotionCommand;

typedef struct {
    bool b_led_on;
} LedCommand;

typedef struct {
    pb_size_t which_packet_type;
    union {
        MotionCommand motion;
        LedCommand led;
    } packet_type;
} JetsonToEspPacket;

#define JetsonToEspPacket_motion_tag 1
#define JetsonToEspPacket_led_tag    2

typedef struct {
    int32_t left_ticks;
    int32_t right_ticks;
} EncoderTelemetry;

typedef struct {
    float voltage;
} BatteryTelemetry;

typedef struct {
    pb_size_t which_packet_type;
    union {
        EncoderTelemetry encoder;
        BatteryTelemetry battery;
    } packet_type;
} EspToJetsonPacket;

#define EspToJetsonPacket_encoder_tag 1
#define EspToJetsonPacket_battery_tag 2

// --- NANOPB FIELDS REFLECTION ARRAYS ---
const pb_field_t MotionCommand_fields[3] = {
    PB_FIELD(1, FLOAT   , REQUIRED, STATIC  , FIRST, MotionCommand, linear_x, linear_x, 0),
    PB_FIELD(2, FLOAT   , REQUIRED, STATIC  , AFTER, MotionCommand, angular_z, linear_x, 0),
    PB_LAST_FIELD
};

const pb_field_t LedCommand_fields[2] = {
    PB_FIELD(1, BOOL    , REQUIRED, STATIC  , FIRST, LedCommand, b_led_on, b_led_on, 0),
    PB_LAST_FIELD
};

const pb_field_t JetsonToEspPacket_fields[3] = {
    PB_ONEOF_FIELD(packet_type, 1, MESSAGE , ONEOF, STATIC  , FIRST, JetsonToEspPacket, motion, motion, &MotionCommand_fields),
    PB_ONEOF_FIELD(packet_type, 2, MESSAGE , ONEOF, STATIC  , UNION, JetsonToEspPacket, led, led, &LedCommand_fields),
    PB_LAST_FIELD
};

const pb_field_t EncoderTelemetry_fields[3] = {
    PB_FIELD(1, INT32   , REQUIRED, STATIC  , FIRST, EncoderTelemetry, left_ticks, left_ticks, 0),
    PB_FIELD(2, INT32   , REQUIRED, STATIC  , AFTER, EncoderTelemetry, right_ticks, left_ticks, 0),
    PB_LAST_FIELD
};

const pb_field_t BatteryTelemetry_fields[2] = {
    PB_FIELD(1, FLOAT   , REQUIRED, STATIC  , FIRST, BatteryTelemetry, voltage, voltage, 0),
    PB_LAST_FIELD
};

const pb_field_t EspToJetsonPacket_fields[3] = {
    PB_ONEOF_FIELD(packet_type, 1, MESSAGE , ONEOF, STATIC  , FIRST, EspToJetsonPacket, encoder, encoder, &EncoderTelemetry_fields),
    PB_ONEOF_FIELD(packet_type, 2, MESSAGE , ONEOF, STATIC  , UNION, EspToJetsonPacket, battery, battery, &BatteryTelemetry_fields),
    PB_LAST_FIELD
};

// --- GLOBAL BUFFER ALLOCATION ---
uint8_t rx_buffer[128];
size_t rx_index = 0;

// --- COBS ENCODER/DECODER ROUTINES ---
size_t cobs_encode(const uint8_t* src, size_t src_len, uint8_t* dst) {
    size_t read_index = 0, write_index = 1, code_index = 0;
    uint8_t code = 1;
    while (read_index < src_len) {
        if (src[read_index] == 0x00) {
            dst[code_index] = code;
            code = 1;
            code_index = write_index++;
            read_index++;
        } else {
            dst[write_index++] = src[read_index++];
            code++;
            if (code == 0xFF) {
                dst[code_index] = code;
                code = 1;
                code_index = write_index++;
            }
        }
    }
    dst[code_index] = code;
    return write_index;
}

size_t cobs_decode(const uint8_t* src, size_t src_len, uint8_t* dst) {
    size_t read_index = 0, write_index = 0;
    uint8_t code, i;
    while (read_index < src_len) {
        code = src[read_index++];
        for (i = 1; i < code; i++) {
            dst[write_index++] = src[read_index++];
        }
        if (code < 0xFF && read_index < src_len) {
            dst[write_index++] = 0x00;
        }
    }
    return write_index;
}

// --- HARDWARE CONTROL ACTUATORS ---
void setMotorSpeeds(float linear_x, float angular_z) {
    float left_speed  = constrain(linear_x - angular_z, -1.0, 1.0);
    float right_speed = constrain(linear_x + angular_z, -1.0, 1.0);

    uint32_t left_pwm  = abs((int)(left_speed * 255));
    uint32_t right_pwm = abs((int)(right_speed * 255));

    ledcWrite(LEFT_MOTOR_PWM_PIN, left_pwm);
    ledcWrite(RIGHT_MOTOR_PWM_PIN, right_pwm);
}

void sendProtobufTelemetry(EspToJetsonPacket* packet) {
    uint8_t raw_buffer[128];
    uint8_t cobs_buffer[130];
    
    pb_ostream_t stream = pb_ostream_from_buffer(raw_buffer, sizeof(raw_buffer));
    if (!pb_encode(&stream, EspToJetsonPacket_fields, packet)) return;
    
    size_t encoded_len = cobs_encode(raw_buffer, stream.bytes_written, cobs_buffer);
    cobs_buffer[encoded_len++] = 0x00;
    
    Serial.write(cobs_buffer, encoded_len);
}

void processIncomingProtobuf(uint8_t* buf, size_t len) {
    uint8_t decoded[128];
    size_t decoded_len = cobs_decode(buf, len, decoded);
    
    JetsonToEspPacket packet = {0};
    pb_istream_t stream = pb_istream_from_buffer(decoded, decoded_len);
    
    if (pb_decode(&stream, JetsonToEspPacket_fields, &packet)) {
        if (packet.which_packet_type == JetsonToEspPacket_motion_tag) {
            setMotorSpeeds(packet.packet_type.motion.linear_x, packet.packet_type.motion.angular_z);
        } 
        else if (packet.which_packet_type == JetsonToEspPacket_led_tag) {
            digitalWrite(STATUS_LED_PIN, packet.packet_type.led.b_led_on ? HIGH : LOW);
        }
    }
}

// --- SETUP AND LOOP ---
void setup() {
    Serial.begin(115200);
    pinMode(STATUS_LED_PIN, OUTPUT);
    analogSetAttenuation(ADC_11db);
    
    pinMode(LEFT_MOTOR_PWM_PIN, OUTPUT);
    pinMode(RIGHT_MOTOR_PWM_PIN, OUTPUT);
    ledcAttachChannel(LEFT_MOTOR_PWM_PIN, 5000, 8, 0);
    ledcAttachChannel(RIGHT_MOTOR_PWM_PIN, 5000, 8, 1);
}

void loop() {
    // 1. Process Stream Ingestion
    while (Serial.available()) {
        uint8_t c = Serial.read();
        if (c == 0x00) {
            if (rx_index > 0) {
                processIncomingProtobuf(rx_buffer, rx_index);
                rx_index = 0;
            }
        } else {
            if (rx_index < sizeof(rx_buffer)) {
                rx_buffer[rx_index++] = c;
            }
        }
    }

    // 2. Scheduled Protobuf Binary Telemetry (Every 50ms)
    static unsigned long last_telemetry = 0;
    if (millis() - last_telemetry > 50) {
        last_telemetry = millis();
        
        EspToJetsonPacket tx_packet = {0};
        tx_packet.which_packet_type = EspToJetsonPacket_battery_tag;
        tx_packet.packet_type.battery.voltage = analogRead(BATTERY_PIN) * (3.3 / 4095.0) * 4.0; 
        
        sendProtobufTelemetry(&tx_packet);
    }

    // 3. Asynchronous ASCII Diagnostic Output (Every 2000ms)
    static unsigned long last_debug = 0;
    if (millis() - last_debug > 2000) {
        last_debug = millis();
        Serial.println("System Status: OK. Protobuf Engine nominal.");
    }
}