/*
  Dual UART Sniffer for Arduino Mega

  - Channel M (Motor/Controller): Serial2  (RX2 pin 17)
  - Channel L (LCD/Display):      Serial3  (RX3 pin 15)
  - PC Monitor:                   Serial   (USB)

  Commands from PC (Serial):
    B=9600        -> set baud rate for Serial2 & Serial3 (hot change)
    D=true        -> enable byte-by-byte debug
    D=false       -> disable byte-by-byte debug
*/

#include <Arduino.h>

// ---------------------- Config ----------------------
static uint32_t pcBaud = 115200;     // Serial (USB) baud
static uint32_t sniffBaud = 9600;    // Initial baud for Serial2/Serial3

// Frame segmentation by "silence gap":
static const uint16_t FRAME_GAP_MS = 25;  // if no bytes for this long -> flush buffer as a "frame"

// Buffers:
static const uint16_t BUFLEN = 128;       // increase if you expect longer frames

// Known packet detection (from original sketch):
static const uint8_t START_MOTOR = 0x43;
static const uint8_t LEN_MOTOR   = 9;
static const uint8_t START_LCD   = 0x59;
static const uint8_t LEN_LCD     = 7;

// ---------------------- State ----------------------
struct ChanState {
  uint8_t  buf[BUFLEN];
  uint16_t pos = 0;
  bool     overflow = false;
  uint32_t lastByteMs = 0;
};

ChanState chM; // Serial2
ChanState chL; // Serial3

bool debugBytes = false;

// ---------------------- Utils ----------------------
static void printHex2(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

static uint8_t chk8_add(uint8_t bval, uint8_t cval) {
  return (uint8_t)((bval + cval) & 0xFF);
}

static void bufClear(ChanState &ch) {
  ch.pos = 0;
  ch.overflow = false;
  // no need to zero the buffer for performance; optional:
  // memset(ch.buf, 0, BUFLEN);
}

static void bufAdd(ChanState &ch, uint8_t b) {
  if (ch.pos < BUFLEN) {
    ch.buf[ch.pos++] = b;
  } else {
    ch.overflow = true; // we still keep reading bytes but won't store more
  }
  ch.lastByteMs = millis();
}

static int16_t findPacket(const ChanState &ch, uint8_t start, uint8_t len) {
  if (len < 2 || ch.pos < len) return -1;

  for (uint16_t i = 0; i + len <= ch.pos; i++) {
    if (ch.buf[i] == start) {
      uint8_t chk = 0;
      for (uint8_t ii = 0; ii < (len - 1); ii++) {
        chk = chk8_add(ch.buf[i + ii], chk);
      }
      if (chk == ch.buf[i + len - 1]) {
        return (int16_t)i;
      }
    }
  }
  return -1;
}

static void printBufferFrame(const char *tag, const ChanState &ch) {
  Serial.print(tag);
  Serial.print(" frame (");
  Serial.print(ch.pos);
  Serial.print(" bytes");
  if (ch.overflow) Serial.print(", OVERFLOW");
  Serial.println("):");

  for (uint16_t i = 0; i < ch.pos; i++) {
    printHex2(ch.buf[i]);
    Serial.print(' ');
    // optional: newline every 16 bytes
    if ((i + 1) % 16 == 0) Serial.println();
  }
  Serial.println();
}

static void printKnownPacketsIfAny(const char *tag, const ChanState &ch) {
  // Motor known packet
  if (tag[0] == 'M') {
    int16_t p = findPacket(ch, START_MOTOR, LEN_MOTOR);
    if (p >= 0) {
      Serial.print("KNOWN Motor packet @");
      Serial.print(p);
      Serial.print(": ");
      for (uint8_t i = 0; i < LEN_MOTOR; i++) {
        printHex2(ch.buf[p + i]);
        Serial.print(' ');
      }
      Serial.println();
    }
  }

  // LCD known packet
  if (tag[0] == 'L') {
    int16_t p = findPacket(ch, START_LCD, LEN_LCD);
    if (p >= 0) {
      Serial.print("KNOWN LCD packet @");
      Serial.print(p);
      Serial.print(": ");
      for (uint8_t i = 0; i < LEN_LCD; i++) {
        printHex2(ch.buf[p + i]);
        Serial.print(' ');
      }
      Serial.println();
    }
  }
}

// ---------------------- Command parsing ----------------------
static void applySniffBaud(uint32_t newBaud) {
  sniffBaud = newBaud;

  // Restart Serial2 / Serial3
  Serial2.end();
  Serial3.end();
  delay(20);
  Serial2.begin(sniffBaud);
  Serial3.begin(sniffBaud);

  Serial.print("OK Baud set to ");
  Serial.println(sniffBaud);
}

static void setDebug(bool on) {
  debugBytes = on;
  Serial.print("OK Debug bytes = ");
  Serial.println(debugBytes ? "true" : "false");
}

static void handleCommandLine(String line) {
  line.trim();
  if (line.length() == 0) return;

  // Accept lowercase too
  String u = line;
  u.toUpperCase();

  // B=xxxx
  if (u.startsWith("B=")) {
    String num = line.substring(2);
    num.trim();
    long b = num.toInt();
    if (b <= 0) {
      Serial.println("ERR Invalid baud. Example: B=9600");
      return;
    }
    applySniffBaud((uint32_t)b);
    return;
  }

  // D=true / D=false (case-insensitive)
  if (u.startsWith("D=")) {
    String val = line.substring(2);
    val.trim();
    val.toLowerCase();
    if (val == "true" || val == "\"true\"" || val == "1" || val == "on") {
      setDebug(true);
    } else if (val == "false" || val == "\"false\"" || val == "0" || val == "off") {
      setDebug(false);
    } else {
      Serial.println("ERR Invalid D. Use D=true or D=false");
    }
    return;
  }

  Serial.print("ERR Unknown command: ");
  Serial.println(line);
  Serial.println("Commands: B=9600 | D=true | D=false");
}

static void pollPCCommands() {
  static String line;
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;
    if (c == '\n') {
      handleCommandLine(line);
      line = "";
    } else {
      // limit line length
      if (line.length() < 64) line += c;
    }
  }
}

// ---------------------- Sniffing ----------------------
static void drainChannel(HardwareSerial &port, ChanState &ch, const char *tag) {
  while (port.available()) {
    uint8_t b = (uint8_t)port.read();
    bufAdd(ch, b);

    if (debugBytes) {
      Serial.print(tag);
      Serial.print(" byte: ");
      printHex2(b);
      Serial.println();
    }
  }
}

static void maybeFlushOnGap(ChanState &ch, const char *tag) {
  if (ch.pos == 0) return;

  uint32_t now = millis();
  if ((uint32_t)(now - ch.lastByteMs) >= FRAME_GAP_MS) {
    // print full buffer as a frame
    printBufferFrame(tag, ch);

    // also try known packet extraction (optional helper)
    printKnownPacketsIfAny(tag, ch);

    // clear for next frame
    bufClear(ch);
  }
}

// ---------------------- Arduino entry ----------------------
void setup() {
  Serial.begin(pcBaud);
  while (!Serial) { /* for boards that need it */ }

  Serial2.begin(sniffBaud);
  Serial3.begin(sniffBaud);

  bufClear(chM);
  bufClear(chL);

  Serial.println("=== Dual UART Sniffer (Mega) ===");
  Serial.print("PC Baud: "); Serial.println(pcBaud);
  Serial.print("Sniff Baud (Serial2/3): "); Serial.println(sniffBaud);
  Serial.println("Commands: B=9600 | D=true | D=false");
  Serial.println("Debug bytes default: false");
}

void loop() {
  // 1) read commands from PC
  pollPCCommands();

  // 2) drain both channels (simultaneous hardware UART)
  drainChannel(Serial2, chM, "M"); // M = Motor/Controller line
  drainChannel(Serial3, chL, "L"); // L = LCD/Display line

  // 3) flush frames when silence gap detected
  maybeFlushOnGap(chM, "M");
  maybeFlushOnGap(chL, "L");
}