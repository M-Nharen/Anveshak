*
 * load_sensor.ino
 * -----------------------------------------------------------------------
 * ESP32 (NodeMCU DevKit V1) firmware, 100% libraryless.
 *
 * Wire protocol (matches pwm_proto.proto):
 *
 *   JetsonToEspPacket {
 *     oneof packet_type { PwmCommand pwm_command = 1; }   // tag 0x0A (LEN)
 *   }
 *   PwmCommand {
 *     sint32 left_pwm  = 1;   // tag 0x08 (VARINT, ZigZag)
 *     sint32 right_pwm = 2;   // tag 0x10 (VARINT, ZigZag)
 *   }
 *
 *   EspToJetsonPacket {
 *     oneof packet_type { LogMessage log_message = 1; }   // tag 0x0A (LEN)
 *   }
 *   LogMessage {
 *     string text = 1;        // tag 0x0A (LEN)
 *   }
 *
 * Both directions share ONE UART. Every frame -- binary PWM commands
 * downstream, and text log packets upstream -- is COBS-encoded before
 * transmission and terminated with a single 0x00 byte. Because COBS
 * guarantees 0x00 never appears inside an encoded frame, the two kinds
 * of traffic can never desynchronize or corrupt one another on the wire.
 * -----------------------------------------------------------------------
 */

#include <Arduino.h>
#include <string.h>
#include <stdio.h>
#include <stdint.h>

// ---------------------------------------------------------------------
// Protobuf wire type constants
// ---------------------------------------------------------------------
#define WT_VARINT    0
#define WT_DELIMITED 2

// ---------------------------------------------------------------------
// Buffer sizing
// ---------------------------------------------------------------------
#define MAX_TEXT_LEN      96    // max characters in a single log message
#define MAX_RAW_PROTO_LEN 160   // max un-encoded protobuf bytes
#define MAX_COBS_LEN      (MAX_RAW_PROTO_LEN + (MAX_RAW_PROTO_LEN / 254) + 2)
#define MAX_RX_FRAME_LEN  128   // max COBS-encoded bytes we'll buffer from RX

static uint8_t rx_raw_buffer[MAX_RX_FRAME_LEN];
static size_t  rx_raw_len = 0;

// =======================================================================
// ZigZag helpers
// =======================================================================

// Encodes a signed 32-bit value into the ZigZag-mapped unsigned form
// that Protobuf's sint32 varint uses on the wire.
static inline uint32_t zigzag_from_int(int32_t n) {
  return (uint32_t)((n << 1) ^ (n >> 31));
}

// Decodes a ZigZag-mapped unsigned varint value back into a signed int32.
static inline int32_t zigzag_to_int(uint64_t n) {
  return (int32_t)(n >> 1) ^ -(int32_t)(n & 1);
}

// =======================================================================
// Protobuf varint encode / decode
// =======================================================================

// Writes `value` as a base-128 varint into buf. Returns bytes written.
// Caller must ensure buf has at least 10 bytes of headroom.
static size_t pb_encode_varint(uint8_t *buf, uint64_t value) {
  size_t i = 0;
  while (value >= 0x80) {
    buf[i++] = (uint8_t)((value & 0x7F) | 0x80);
    value >>= 7;
  }
  buf[i++] = (uint8_t)(value & 0x7F);
  return i;
}

// Reads a base-128 varint starting at buf[0]. `max_len` is the number of
// valid bytes available in buf. On success returns true, sets *value and
// *bytes_read. On malformed/truncated input returns false.
static bool pb_decode_varint(const uint8_t *buf, size_t max_len,
                              uint64_t *value, size_t *bytes_read) {
  uint64_t result = 0;
  size_t shift = 0;
  size_t i = 0;

  while (true) {
    if (i >= max_len || i >= 10) {
      return false; // truncated or malformed (too long for a varint)
    }
    uint8_t byte = buf[i];
    result |= (uint64_t)(byte & 0x7F) << shift;
    i++;
    if ((byte & 0x80) == 0) {
      break;
    }
    shift += 7;
  }

  *value = result;
  *bytes_read = i;
  return true;
}

// =======================================================================
// COBS encode / decode
// =======================================================================

// Encodes `length` bytes from `input` into `output` using standard COBS.
// Does NOT append the trailing 0x00 delimiter -- caller adds that.
// Returns the number of bytes written to output.
static size_t encode_cobs_frame(const uint8_t *input, size_t length,
                                 uint8_t *output) {
  size_t read_index = 0;
  size_t write_index = 1;
  size_t code_index = 0;
  uint8_t code = 1;

  while (read_index < length) {
    if (input[read_index] == 0x00) {
      output[code_index] = code;
      code = 1;
      code_index = write_index++;
      read_index++;
    } else {
      output[write_index++] = input[read_index++];
      code++;
      if (code == 0xFF) {
        output[code_index] = code;
        code = 1;
        code_index = write_index++;
      }
    }
  }
  output[code_index] = code;
  return write_index;
}

// Decodes a COBS frame (NOT including the trailing 0x00 delimiter) from
// `input` (length bytes) into `output`. Returns the number of decoded
// bytes, or 0 on a malformed frame.
static size_t decode_cobs_frame(const uint8_t *input, size_t length,
                                 uint8_t *output) {
  size_t read_index = 0;
  size_t write_index = 0;

  while (read_index < length) {
    uint8_t code = input[read_index];
    if (code == 0 || (read_index + code > length && code != 1)) {
      return 0; // malformed frame
    }
    read_index++;
    for (uint8_t i = 1; i < code; i++) {
      if (read_index >= length) {
        return 0;
      }
      output[write_index++] = input[read_index++];
    }
    if (code != 0xFF && read_index != length) {
      output[write_index++] = 0x00;
    }
  }
  return write_index;
}

// =======================================================================
// Upstream: build + send a LogMessage wrapped in an EspToJetsonPacket
// =======================================================================

// Manually bit-packs `text_msg` into:
//   EspToJetsonPacket { log_message: LogMessage { text: text_msg } }
// COBS-frames it, appends the 0x00 delimiter, and writes it to Serial.
void send_instant_log_to_jetson(const char *text_msg) {
  size_t text_len = strlen(text_msg);
  if (text_len > MAX_TEXT_LEN) {
    text_len = MAX_TEXT_LEN; // truncate defensively
  }

  uint8_t raw[MAX_RAW_PROTO_LEN];
  size_t pos = 0;

  // --- Inner LogMessage: field 1 (text), wire type LEN ---
  uint8_t inner[MAX_RAW_PROTO_LEN];
  size_t inner_pos = 0;
  inner[inner_pos++] = (uint8_t)((1 << 3) | WT_DELIMITED); // tag 0x0A
  inner_pos += pb_encode_varint(inner + inner_pos, text_len);
  memcpy(inner + inner_pos, text_msg, text_len);
  inner_pos += text_len;

  // --- Outer EspToJetsonPacket: field 1 (log_message), wire type LEN ---
  raw[pos++] = (uint8_t)((1 << 3) | WT_DELIMITED); // tag 0x0A
  pos += pb_encode_varint(raw + pos, inner_pos);
  memcpy(raw + pos, inner, inner_pos);
  pos += inner_pos;

  // --- COBS frame + delimiter, then push to the wire ---
  uint8_t encoded[MAX_COBS_LEN];
  size_t encoded_len = encode_cobs_frame(raw, pos, encoded);

  Serial.write(encoded, encoded_len);
  Serial.write((uint8_t)0x00);
  Serial.flush();
}

// =======================================================================
// Downstream: parse a decoded JetsonToEspPacket buffer
// =======================================================================

// Parses one field (tag + payload) starting at buf[offset]. On success,
// advances *offset past the field and returns true.
static bool read_field_header(const uint8_t *buf, size_t len, size_t *offset,
                               uint32_t *field_number, uint32_t *wire_type) {
  uint64_t tag_val;
  size_t bytes_read;
  if (!pb_decode_varint(buf + *offset, len - *offset, &tag_val, &bytes_read)) {
    return false;
  }
  *offset += bytes_read;
  *field_number = (uint32_t)(tag_val >> 3);
  *wire_type = (uint32_t)(tag_val & 0x07);
  return true;
}

// Reconstructs the COBS-decoded byte stream as a JetsonToEspPacket,
// extracts the top-level delimited PwmCommand (field_number == 1),
// pulls left_pwm / right_pwm out via ZigZag decoding, and immediately
// fires back a formatted ACK string over the same link.
void parse_incoming_buffer(uint8_t *buffer, size_t size) {
  size_t offset = 0;
  uint32_t field_number, wire_type;

  if (!read_field_header(buffer, size, &offset, &field_number, &wire_type)) {
    send_instant_log_to_jetson("ERR: malformed outer tag");
    return;
  }

  if (field_number != 1 || wire_type != WT_DELIMITED) {
    send_instant_log_to_jetson("ERR: unexpected outer field/type");
    return;
  }

  // Length-delimited payload = the embedded PwmCommand bytes.
  uint64_t inner_len;
  size_t bytes_read;
  if (!pb_decode_varint(buffer + offset, size - offset, &inner_len, &bytes_read)) {
    send_instant_log_to_jetson("ERR: malformed length varint");
    return;
  }
  offset += bytes_read;

  if (offset + inner_len > size) {
    send_instant_log_to_jetson("ERR: truncated PwmCommand payload");
    return;
  }

  size_t inner_end = offset + (size_t)inner_len;
  size_t inner_offset = offset;

  int32_t left_pwm = 0;
  int32_t right_pwm = 0;
  bool got_left = false;
  bool got_right = false;

  while (inner_offset < inner_end) {
    uint32_t f_num, f_wt;
    if (!read_field_header(buffer, inner_end, &inner_offset, &f_num, &f_wt)) {
      break;
    }

    if (f_wt != WT_VARINT) {
      // Not a field we understand -- bail rather than mis-parse.
      break;
    }

    uint64_t zz_val;
    size_t vread;
    if (!pb_decode_varint(buffer + inner_offset, inner_end - inner_offset,
                           &zz_val, &vread)) {
      break;
    }
    inner_offset += vread;

    if (f_num == 1) {
      left_pwm = zigzag_to_int(zz_val);
      got_left = true;
    } else if (f_num == 2) {
      right_pwm = zigzag_to_int(zz_val);
      got_right = true;
    }
    // Unknown field numbers are silently skipped (already consumed above
    // since we only support varint sub-fields in PwmCommand).
  }

  if (!got_left || !got_right) {
    send_instant_log_to_jetson("ERR: incomplete PwmCommand fields");
    return;
  }

  // --- Drive the motors here ---
  // apply_motor_pwm(left_pwm, right_pwm);  // hook up to your motor driver

  char ack[MAX_TEXT_LEN];
  snprintf(ack, sizeof(ack),
           "ACK Match! Received PWM values -> Left: %ld | Right: %ld",
           (long)left_pwm, (long)right_pwm);
  send_instant_log_to_jetson(ack);
}

// =======================================================================
// Arduino entry points
// =======================================================================

void setup() {
  Serial.begin(115200);
  // Give the USB-UART bridge a brief moment to enumerate before the
  // first write; harmless if already connected.
  delay(50);
  rx_raw_len = 0;
  send_instant_log_to_jetson("ESP32 online. Awaiting PWM commands.");
}

void loop() {
  while (Serial.available() > 0) {
    uint8_t incoming_byte = (uint8_t)Serial.read();

    if (incoming_byte == 0x00) {
      if (rx_raw_len > 0) {
        uint8_t decoded[MAX_RAW_PROTO_LEN];
        size_t decoded_len = decode_cobs_frame(rx_raw_buffer, rx_raw_len, decoded);

        if (decoded_len > 0) {
          parse_incoming_buffer(decoded, decoded_len);
        } else {
          send_instant_log_to_jetson("ERR: COBS decode failed");
        }
      }
      rx_raw_len = 0;
      continue;
    }

    if (rx_raw_len < MAX_RX_FRAME_LEN) {
      rx_raw_buffer[rx_raw_len++] = incoming_byte;
    } else {
      // Frame too long / stream desynced -- drop and resync on next 0x00.
      rx_raw_len = 0;
    }
  }
}