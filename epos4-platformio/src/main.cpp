#include <Arduino.h>
#include <SPI.h>
#include <mcp_can.h>

#define MCP_CS    4
#define MCP_SCK   18
#define MCP_MISO  19
#define MCP_MOSI  23

#define NODE_ID   1

#define MCP_CLOCK MCP_8MHZ
#define CAN_BAUD  CAN_500KBPS

#define COB_NMT      0x000
#define COB_SDO_TX   (0x600 + NODE_ID)
#define COB_SDO_RX   (0x580 + NODE_ID)

MCP_CAN CAN0(MCP_CS);

enum DriveMode {
  MODE_VELOCITY,
  MODE_POSITION
};

enum PositionCommandMode {
  POS_ABS,
  POS_REL
};

struct DriveConfig {
  DriveMode mode = MODE_VELOCITY;
  PositionCommandMode posMode = POS_ABS;

  int32_t rpm = 200;
  int32_t maxRpm = 7000;
  uint32_t acc = 10;
  uint32_t dec = 10;
  int32_t pos = 0;
  bool enabled = false;
};

DriveConfig cfg;
String serialLine;

// =========================
// LOG
// =========================
static void printAbort(const uint8_t data[8]) {
  uint32_t abortCode =
      ((uint32_t)data[4]) |
      ((uint32_t)data[5] << 8) |
      ((uint32_t)data[6] << 16) |
      ((uint32_t)data[7] << 24);

  Serial.print("ERR SDO_ABORT 0x");
  Serial.println(abortCode, HEX);
}

static void sendOk(const String &msg) {
  Serial.print("OK ");
  Serial.println(msg);
}

static void sendErr(const String &msg) {
  Serial.print("ERR ");
  Serial.println(msg);
}

// =========================
// CAN
// =========================
static bool canSendFrame(uint16_t id, uint8_t dlc, const uint8_t data[8]) {
  byte rc = CAN0.sendMsgBuf(id, 0, dlc, (uint8_t*)data);
  if (rc != CAN_OK) {
    Serial.print("ERR CAN_TX ");
    Serial.println(rc);
    return false;
  }
  return true;
}

static bool waitFrame(uint16_t expectedId, uint8_t *dlc, uint8_t data[8], uint32_t timeoutMs = 1000) {
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    if (CAN0.checkReceive() == CAN_MSGAVAIL) {
      unsigned long rxId = 0;
      CAN0.readMsgBuf(&rxId, dlc, data);

      if ((uint16_t)rxId == expectedId) {
        return true;
      }
    }
    delay(1);
  }

  Serial.print("ERR TIMEOUT 0x");
  Serial.println(expectedId, HEX);
  return false;
}

// =========================
// SDO WRITE
// =========================
static bool sdoWriteU8(uint16_t index, uint8_t subindex, uint8_t value) {
  uint8_t tx[8] = {
    0x2F,
    (uint8_t)(index & 0xFF),
    (uint8_t)((index >> 8) & 0xFF),
    subindex,
    value, 0, 0, 0
  };

  if (!canSendFrame(COB_SDO_TX, 8, tx)) return false;

  uint8_t rx[8] = {0};
  uint8_t dlc = 0;
  if (!waitFrame(COB_SDO_RX, &dlc, rx)) return false;

  if (rx[0] == 0x60) return true;
  if (rx[0] == 0x80) printAbort(rx);
  return false;
}

static bool sdoWriteU16(uint16_t index, uint8_t subindex, uint16_t value) {
  uint8_t tx[8] = {
    0x2B,
    (uint8_t)(index & 0xFF),
    (uint8_t)((index >> 8) & 0xFF),
    subindex,
    (uint8_t)(value & 0xFF),
    (uint8_t)((value >> 8) & 0xFF),
    0, 0
  };

  if (!canSendFrame(COB_SDO_TX, 8, tx)) return false;

  uint8_t rx[8] = {0};
  uint8_t dlc = 0;
  if (!waitFrame(COB_SDO_RX, &dlc, rx)) return false;

  if (rx[0] == 0x60) return true;
  if (rx[0] == 0x80) printAbort(rx);
  return false;
}

static bool sdoWriteU32(uint16_t index, uint8_t subindex, uint32_t value) {
  uint8_t tx[8] = {
    0x23,
    (uint8_t)(index & 0xFF),
    (uint8_t)((index >> 8) & 0xFF),
    subindex,
    (uint8_t)(value & 0xFF),
    (uint8_t)((value >> 8) & 0xFF),
    (uint8_t)((value >> 16) & 0xFF),
    (uint8_t)((value >> 24) & 0xFF)
  };

  if (!canSendFrame(COB_SDO_TX, 8, tx)) return false;

  uint8_t rx[8] = {0};
  uint8_t dlc = 0;
  if (!waitFrame(COB_SDO_RX, &dlc, rx)) return false;

  if (rx[0] == 0x60) return true;
  if (rx[0] == 0x80) printAbort(rx);
  return false;
}

static bool sdoWriteI32(uint16_t index, uint8_t subindex, int32_t value) {
  return sdoWriteU32(index, subindex, (uint32_t)value);
}

// =========================
// SDO READ
// =========================
static bool sdoReadU16(uint16_t index, uint8_t subindex, uint16_t &value) {
  uint8_t tx[8] = {
    0x40,
    (uint8_t)(index & 0xFF),
    (uint8_t)((index >> 8) & 0xFF),
    subindex,
    0, 0, 0, 0
  };

  if (!canSendFrame(COB_SDO_TX, 8, tx)) return false;

  uint8_t rx[8] = {0};
  uint8_t dlc = 0;
  if (!waitFrame(COB_SDO_RX, &dlc, rx)) return false;

  if (rx[0] == 0x4B) {
    value = (uint16_t)rx[4] | ((uint16_t)rx[5] << 8);
    return true;
  }

  if (rx[0] == 0x80) printAbort(rx);
  return false;
}

static bool sdoReadI32(uint16_t index, uint8_t subindex, int32_t &value) {
  uint8_t tx[8] = {
    0x40,
    (uint8_t)(index & 0xFF),
    (uint8_t)((index >> 8) & 0xFF),
    subindex,
    0, 0, 0, 0
  };

  if (!canSendFrame(COB_SDO_TX, 8, tx)) return false;

  uint8_t rx[8] = {0};
  uint8_t dlc = 0;
  if (!waitFrame(COB_SDO_RX, &dlc, rx)) return false;

  if (rx[0] == 0x43) {
    value =
      ((int32_t)rx[4]) |
      ((int32_t)rx[5] << 8) |
      ((int32_t)rx[6] << 16) |
      ((int32_t)rx[7] << 24);
    return true;
  }

  if (rx[0] == 0x80) printAbort(rx);
  return false;
}

// =========================
// EPOS / CiA402
// =========================
static bool sendNmtStart(uint8_t nodeId) {
  uint8_t data[8] = {0x01, nodeId, 0, 0, 0, 0, 0, 0};
  return canSendFrame(COB_NMT, 2, data);
}

static bool writeControlword(uint16_t cw) {
  return sdoWriteU16(0x6040, 0x00, cw);
}

static bool readStatusword(uint16_t &sw) {
  return sdoReadU16(0x6041, 0x00, sw);
}

static bool readActualVelocity(int32_t &vel) {
  return sdoReadI32(0x606C, 0x00, vel);
}

static bool readActualPosition(int32_t &pos) {
  return sdoReadI32(0x6064, 0x00, pos);
}

static bool isTargetReached(uint16_t sw) {
  return (sw & (1 << 10)) != 0;
}

static bool isSetpointAcknowledged(uint16_t sw) {
  return (sw & (1 << 12)) != 0;
}

static bool faultReset()       { return writeControlword(0x0080); }
static bool shutdownDrive()    { return writeControlword(0x0006); }
static bool switchOnDrive()    { return writeControlword(0x0007); }
static bool enableOperation()  { return writeControlword(0x000F); }
static bool disableOperation() { return writeControlword(0x0007); }
static bool disableVoltage()   { return writeControlword(0x0000); }

static bool enableDrive() {
  if (!faultReset()) return false;
  delay(100);
  if (!shutdownDrive()) return false;
  delay(100);
  if (!switchOnDrive()) return false;
  delay(100);
  if (!enableOperation()) return false;
  delay(100);
  cfg.enabled = true;
  return true;
}

static bool disableDrive() {
  if (!disableVoltage()) return false;
  cfg.enabled = false;
  return true;
}

// =========================
// TRYBY
// =========================
static bool applyVelocityMode() {
  if (!sdoWriteU8(0x6060, 0x00, 3)) return false;
  if (!sdoWriteU32(0x607F, 0x00, (uint32_t)cfg.maxRpm)) return false;
  if (!sdoWriteU32(0x6083, 0x00, cfg.acc)) return false;
  if (!sdoWriteU32(0x6084, 0x00, cfg.dec)) return false;
  if (!sdoWriteU16(0x6086, 0x00, 0)) return false;
  return true;
}

static bool applyPositionMode() {
  if (!sdoWriteU8(0x6060, 0x00, 1)) return false;
  if (!sdoWriteU32(0x607F, 0x00, (uint32_t)cfg.maxRpm)) return false;
  if (!sdoWriteU32(0x6081, 0x00, (uint32_t)cfg.rpm)) return false;
  if (!sdoWriteU32(0x6083, 0x00, cfg.acc)) return false;
  if (!sdoWriteU32(0x6084, 0x00, cfg.dec)) return false;
  if (!sdoWriteU16(0x6086, 0x00, 0)) return false;
  if (!sdoWriteU32(0x6067, 0x00, 50)) return false;
  if (!sdoWriteU16(0x6068, 0x00, 100)) return false;
  return true;
}

static bool applyConfig() {
  if (cfg.mode == MODE_VELOCITY) return applyVelocityMode();
  return applyPositionMode();
}

// =========================
// RUCH
// =========================
static bool runVelocity() {
  if (!sdoWriteI32(0x60FF, 0x00, cfg.rpm)) return false;
  if (!writeControlword(0x010F)) return false;
  delay(50);
  if (!writeControlword(0x000F)) return false;
  return true;
}

static bool stopVelocity() {
  if (!sdoWriteI32(0x60FF, 0x00, 0)) return false;

  if (!writeControlword(0x010F)) return false;
  delay(30);

  if (!writeControlword(0x000F)) return false;
  delay(30);

  if (!sdoWriteI32(0x60FF, 0x00, 0)) return false;
  if (!writeControlword(0x010F)) return false;
  delay(30);

  if (!writeControlword(0x000F)) return false;
  return true;
}

static bool waitForTargetReached(uint32_t timeoutMs = 20000) {
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    uint16_t sw = 0;
    if (readStatusword(sw) && isTargetReached(sw)) {
      return true;
    }
    delay(100);
  }

  return false;
}

static bool runPosition() {
  if (!sdoWriteI32(0x607A, 0x00, cfg.pos)) return false;

  uint16_t cwPrepare;
  uint16_t cwStart;

  if (cfg.posMode == POS_ABS) {
    cwPrepare = 0x002F;
    cwStart   = 0x003F;
  } else {
    cwPrepare = 0x006F;
    cwStart   = 0x007F;
  }

  if (!writeControlword(cwPrepare)) return false;
  delay(20);

  if (!writeControlword(cwStart)) return false;
  delay(20);

  if (!writeControlword(cwPrepare)) return false;

  if (!waitForTargetReached(20000)) {
    Serial.println("ERR TARGET_NOT_REACHED");
    return false;
  }

  Serial.println("OK TARGET_REACHED");
  return true;
}

static bool stopDrive() {
  if (cfg.mode == MODE_VELOCITY) return stopVelocity();

  if (!disableOperation()) return false;
  delay(30);
  if (!disableVoltage()) return false;
  cfg.enabled = false;
  return true;
}

// =========================
// STATUS / TELEM
// =========================
static void sendTelemetry() {
  int32_t vel = 0;
  int32_t pos = 0;

  bool velOk = readActualVelocity(vel);
  bool posOk = readActualPosition(pos);

  Serial.print("TELEM VEL=");
  if (velOk) Serial.print(vel); else Serial.print("ERR");

  Serial.print(" POS=");
  if (posOk) Serial.print(pos); else Serial.print("ERR");

  Serial.println();
}

static void sendStatus() {
  uint16_t sw = 0;
  int32_t vel = 0;
  int32_t pos = 0;

  bool swOk  = readStatusword(sw);
  bool velOk = readActualVelocity(vel);
  bool posOk = readActualPosition(pos);

  Serial.print("STAT MODE=");
  Serial.print(cfg.mode == MODE_VELOCITY ? "VEL" : "POS");

  Serial.print(" POSMODE=");
  Serial.print(cfg.posMode == POS_ABS ? "ABS" : "REL");

  Serial.print(" RPM=");
  Serial.print(cfg.rpm);

  Serial.print(" MAXRPM=");
  Serial.print(cfg.maxRpm);

  Serial.print(" ACC=");
  Serial.print(cfg.acc);

  Serial.print(" DEC=");
  Serial.print(cfg.dec);

  Serial.print(" POS_CMD=");
  Serial.print(cfg.pos);

  Serial.print(" EN=");
  Serial.print(cfg.enabled ? 1 : 0);

  Serial.print(" SW=");
  if (swOk) Serial.print(sw, HEX); else Serial.print("ERR");

  Serial.print(" VEL=");
  if (velOk) Serial.print(vel); else Serial.print("ERR");

  Serial.print(" POS_ACT=");
  if (posOk) Serial.print(pos); else Serial.print("ERR");

  Serial.print(" TR=");
  if (swOk) Serial.print(isTargetReached(sw) ? 1 : 0); else Serial.print("ERR");

  Serial.print(" ACK=");
  if (swOk) Serial.print(isSetpointAcknowledged(sw) ? 1 : 0); else Serial.print("ERR");

  Serial.println();
}

// =========================
// KOMENDY Z GUI
// =========================
static void handleCommand(String line) {
  line.trim();
  if (line.length() == 0) return;

  if (line == "PING") {
    sendOk("PONG");
    return;
  }

  if (line == "ENABLE") {
    if (enableDrive()) sendOk("ENABLE");
    else sendErr("ENABLE");
    return;
  }

  if (line == "DISABLE") {
    if (disableDrive()) sendOk("DISABLE");
    else sendErr("DISABLE");
    return;
  }

  if (line == "APPLY") {
    if (applyConfig()) sendOk("APPLY");
    else sendErr("APPLY");
    return;
  }

  if (line == "RUN") {
    bool ok = (cfg.mode == MODE_VELOCITY) ? runVelocity() : runPosition();
    if (ok) sendOk("RUN");
    else sendErr("RUN");
    return;
  }

  if (line == "STOP") {
    if (stopDrive()) sendOk("STOP");
    else sendErr("STOP");
    return;
  }

  if (line == "STATUS") {
    sendStatus();
    return;
  }

  if (line == "TELEM") {
    sendTelemetry();
    return;
  }

  if (line.startsWith("MODE ")) {
    String v = line.substring(5);
    v.trim();
    v.toUpperCase();

    if (v == "VEL") {
      cfg.mode = MODE_VELOCITY;
      sendOk("MODE VEL");
      return;
    }

    if (v == "POS") {
      cfg.mode = MODE_POSITION;
      sendOk("MODE POS");
      return;
    }

    sendErr("MODE");
    return;
  }

  if (line.startsWith("POSMODE ")) {
    String v = line.substring(8);
    v.trim();
    v.toUpperCase();

    if (v == "ABS") {
      cfg.posMode = POS_ABS;
      sendOk("POSMODE ABS");
      return;
    }

    if (v == "REL") {
      cfg.posMode = POS_REL;
      sendOk("POSMODE REL");
      return;
    }

    sendErr("POSMODE");
    return;
  }

  if (line.startsWith("SET ")) {
    int sep = line.indexOf(' ', 4);
    if (sep < 0) {
      sendErr("SET FORMAT");
      return;
    }

    String key = line.substring(4, sep);
    String val = line.substring(sep + 1);
    key.trim();
    val.trim();
    key.toUpperCase();

    long parsed = val.toInt();

    if (key == "RPM") {
      cfg.rpm = (int32_t)parsed;
      sendOk("SET RPM");
      return;
    }

    if (key == "MAXRPM") {
      cfg.maxRpm = (int32_t)parsed;
      sendOk("SET MAXRPM");
      return;
    }

    if (key == "ACC") {
      cfg.acc = (uint32_t)parsed;
      sendOk("SET ACC");
      return;
    }

    if (key == "DEC") {
      cfg.dec = (uint32_t)parsed;
      sendOk("SET DEC");
      return;
    }

    if (key == "POS") {
      cfg.pos = (int32_t)parsed;
      sendOk("SET POS");
      return;
    }

    sendErr("SET KEY");
    return;
  }

  sendErr("UNKNOWN");
}

static void readSerialCommands() {
  while (Serial.available()) {
    char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (serialLine.length() > 0) {
        handleCommand(serialLine);
        serialLine = "";
      }
    } else {
      serialLine += c;

      if (serialLine.length() > 120) {
        serialLine = "";
        sendErr("LINE_TOO_LONG");
      }
    }
  }
}

// =========================
// SETUP / LOOP
// =========================
void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println("BOOT ESP32 EPOS4 GUI BRIDGE");

  SPI.begin(MCP_SCK, MCP_MISO, MCP_MOSI, MCP_CS);

  while (CAN0.begin(MCP_ANY, CAN_BAUD, MCP_CLOCK) != CAN_OK) {
    Serial.println("ERR MCP2515_INIT");
    delay(1000);
  }

  CAN0.setMode(MCP_NORMAL);
  Serial.println("OK MCP2515_READY");

  if (!sendNmtStart(NODE_ID)) {
    Serial.println("ERR NMT_START");
  } else {
    Serial.println("OK NMT_START");
  }
}

void loop() {
  readSerialCommands();
}