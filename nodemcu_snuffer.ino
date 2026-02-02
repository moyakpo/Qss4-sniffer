#include <Arduino.h>
#include <SoftwareSerial.h>

// ---------- NodeMCU v3 (ESP8266MOD) pins ----------
#define RX_MOTOR D5   // GPIO14  <- Conectar TX Controladora
#define RX_LCD   D6   // GPIO12  <- Conectar TX Pantalla
#define TX_DUMMY D7   // GPIO13  (NO conectar; requerido por SoftwareSerial)

// ---------- Config ----------
static const uint32_t PC_BAUD_DEFAULT = 115200;
static uint32_t sniffBaud = 9600;

// Frame segmentation by "silence gap"
static const uint16_t FRAME_GAP_MS = 25;

// Switch listener if idle (timeslice)
static const uint16_t SWITCH_TIMEOUT_MS = 50;

// Buffer size per channel
static const uint16_t BUFLEN = 128;

bool debugBytes = false;

// Two software serial ports (ESP8266: only one can listen at a time)
SoftwareSerial SoftM(RX_MOTOR, TX_DUMMY); // Motor/Controladora
SoftwareSerial SoftL(RX_LCD,   TX_DUMMY); // LCD/Pantalla

uint32_t lastSwitchMs = 0;

struct ChanState {
  uint8_t  buf[BUFLEN];
  uint16_t pos = 0;
  bool     overflow = false;
  uint32_t lastByteMs = 0;
};

ChanState chM;
ChanState chL;

static void printHex2(uint8_t b) {
  if (b < 0x10) Serial.print('0');
  Serial.print(b, HEX);
}

static void bufClear(ChanState &ch) {
  ch.pos = 0;
  ch.overflow = false;
}

static void bufAdd(ChanState &ch, uint8_t b) {
  if (ch.pos < BUFLEN) ch.buf[ch.pos++] = b;
  else ch.overflow = true;
  ch.lastByteMs = millis();
}

static void printFrame(const char *tag, ChanState &ch) {
  Serial.print(tag);
  Serial.print(" frame (");
  Serial.print(ch.pos);
  if (ch.overflow) Serial.print(", OVERFLOW");
  Serial.println("):");

  for (uint16_t i = 0; i < ch.pos; i++) {
    printHex2(ch.buf[i]);
    Serial.print(' ');
    if ((i + 1) % 16 == 0) Serial.println();
  }
  Serial.println();

  bufClear(ch);
}

// ----- Commands over USB Serial -----
static void restartSoftSerial() {
  SoftM.end();
  SoftL.end();
  delay(20);
  SoftM.begin(sniffBaud);
  SoftL.begin(sniffBaud);

  // restore current listener
  if (SoftM.isListening()) SoftM.listen();
  else SoftL.listen();
}

static void applySniffBaud(uint32_t b) {
  if (b == 0) {
    Serial.println("ERR Invalid baud. Example: B=9600");
    return;
  }
  sniffBaud = b;
  restartSoftSerial();
  Serial.print("OK Baud = ");
  Serial.println(sniffBaud);
}

static void setDebug(bool on) {
  debugBytes = on;
  Serial.print("OK Debug bytes = ");
  Serial.println(debugBytes ? "true" : "false");
}

static void handleCommandLine(String line) {
  line.trim();
  if (!line.length()) return;

  String u = line; u.toUpperCase();

  if (u.startsWith("B=")) {
    String num = line.substring(2);
    num.trim();
    applySniffBaud((uint32_t)num.toInt());
    return;
  }

  if (u.startsWith("D=")) {
    String val = line.substring(2);
    val.trim();
    val.toLowerCase();

    if (val == "true" || val == "\"true\"" || val == "1" || val == "on") setDebug(true);
    else if (val == "false" || val == "\"false\"" || val == "0" || val == "off") setDebug(false);
    else Serial.println("ERR Use D=true or D=false");
    return;
  }

  Serial.println("ERR Unknown command.");
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
      if (line.length() < 64) line += c;
    }
  }
}

// ----- Sniff logic -----
static void drain(SoftwareSerial &port, ChanState &ch, const char *tag) {
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

static void checkFlush(ChanState &ch, const char *tag) {
  if (ch.pos == 0) return;
  if ((millis() - ch.lastByteMs) >= FRAME_GAP_MS) {
    printFrame(tag, ch);
  }
}

static void switchListener() {
  if (SoftM.isListening()) SoftL.listen();
  else SoftM.listen();
  lastSwitchMs = millis();
}

void setup() {
  Serial.begin(PC_BAUD_DEFAULT);
  delay(50);

  SoftM.begin(sniffBaud);
  SoftL.begin(sniffBaud);

  // Start listening Motor by default
  SoftM.listen();
  lastSwitchMs = millis();

  bufClear(chM);
  bufClear(chL);

  Serial.println("=== ESP8266MOD Dual Sniffer (timesliced) ===");
  Serial.print("USB Serial Baud: "); Serial.println(PC_BAUD_DEFAULT);
  Serial.print("Sniff Baud: "); Serial.println(sniffBaud);
  Serial.println("RX pins: Motor=D5(GPIO14), LCD=D6(GPIO12)");
  Serial.println("Commands: B=9600 | D=true | D=false");
  Serial.println("Default debug: OFF");
}

void loop() {
  pollPCCommands();

  // Drain whichever is currently listening
  if (SoftM.isListening()) drain(SoftM, chM, "M");
  else drain(SoftL, chL, "L");

  // Flush frames on silence
  checkFlush(chM, "M");
  checkFlush(chL, "L");

  // Switch listener periodically so we can catch both directions
  if ((millis() - lastSwitchMs) > SWITCH_TIMEOUT_MS) {
    switchListener();
  }
}
