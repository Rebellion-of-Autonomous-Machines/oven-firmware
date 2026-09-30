#include <Arduino.h>
#include <WiFi.h>
#include <Wire.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ModbusRTU.h>

// --- I2C relays ---
static const uint8_t I2C_SDA_PIN = 4;
static const uint8_t I2C_SCL_PIN = 15;
static const uint8_t PCF8574_RELAY_ADDR = 0x24;
static const uint8_t PCF8574_INPUT_ADDR = 0x22;
static const uint8_t DI5_BIT = 4; // P4 on input PCF8574
static const uint8_t DI6_BIT = 5; // P5 on input PCF8574

// Physical relay indexes (0-based on PCF8574)
static const uint8_t RELAY_MOTOR_A = 4; // Relay 5
static const uint8_t RELAY_MOTOR_B = 5; // Relay 6

// false = NC/NC stop, true = NO/NO stop
static const bool STOP_USE_NO_STATE = false;
static const uint16_t RELAY_DEADTIME_MS = 120;
static const uint32_t DOOR_TRAVEL_TIME_MS = 6000;

// --- WiFi AP ---
const char* ap_ssid = "oven";
const char* ap_password = "clickclick";

WebServer server(80);
DNSServer dnsServer;
static const uint16_t DNS_PORT = 53;

// --- Modbus RTU ---
HardwareSerial RS485Serial(2);
ModbusRTU mb;
static const uint8_t RS485_RX_PIN = 14;
static const uint8_t RS485_TX_PIN = 27;
static const uint32_t RS485_BAUDRATE = 9600;
static const uint8_t MODBUS_SLAVE_ID = 1;

enum ModbusReg : uint16_t {
  REG_COMMAND = 0,
  REG_DIRECTION = 1,
  REG_STATUS_BITS = 2,
  REG_COMMAND_RESULT = 3,
  REG_REG_COUNT = 16
};

enum ModbusCommand : uint16_t {
  CMD_NONE = 0,
  CMD_DOOR_OPEN = 1,
  CMD_DOOR_CLOSE = 2
};

enum ModbusCommandResult : uint16_t {
  CMD_RES_OK = 0,
  CMD_RES_UNKNOWN_CMD = 1,
  CMD_RES_BUSY = 2
};

enum MotorDirection : uint8_t {
  MOTOR_STOP = 0,
  MOTOR_OPEN = 1,
  MOTOR_CLOSE = 2
};

enum DoorState : uint8_t {
  DOOR_UNKNOWN = 0,
  DOOR_OPENING = 1,
  DOOR_OPEN = 2,
  DOOR_CLOSING = 3,
  DOOR_CLOSED = 4
};

enum ControlMode : uint8_t {
  MODE_AUTOMATIC,
  MODE_MANUAL_OPEN,
  MODE_MANUAL_CLOSE,
  MODE_INVALID
};

// PCF8574 relay modules are usually active-low: true means relay coil ON.
bool relayStates[6] = {false, false, false, false, false, false};
MotorDirection currentDirection = MOTOR_STOP;
DoorState doorState = DOOR_UNKNOWN;
uint32_t doorMovementDeadlineMs = 0;
bool di5State = true;
bool di6State = false;
bool inputStatesValid = false;
ControlMode controlMode = MODE_INVALID;
bool manualOverrideActive = false;
bool manualCommandPending = false;
MotorDirection manualTarget = MOTOR_STOP;

void initInputExpander() {
  // Writing ones keeps all PCF8574 pins in quasi-bidirectional input mode.
  Wire.beginTransmission(PCF8574_INPUT_ADDR);
  Wire.write(0xFF);
  Wire.endTransmission();
}

void updateInputStates() {
  Wire.requestFrom(PCF8574_INPUT_ADDR, static_cast<uint8_t>(1));
  if (Wire.available() < 1) {
    return;
  }

  uint8_t inputByte = Wire.read();
  di5State = bitRead(inputByte, DI5_BIT);
  di6State = bitRead(inputByte, DI6_BIT);
  inputStatesValid = true;
}

void writeRelays() {
  uint8_t value = 0xFF;
  for (uint8_t i = 0; i < 6; i++) {
    if (relayStates[i]) {
      bitClear(value, i);
    }
  }

  Wire.beginTransmission(PCF8574_RELAY_ADDR);
  Wire.write(value);
  Wire.endTransmission();
}

void setMotorRelaysRaw(bool relayA_no, bool relayB_no) {
  relayStates[RELAY_MOTOR_A] = relayA_no;
  relayStates[RELAY_MOTOR_B] = relayB_no;
  writeRelays();
}

void stopMotor() {
  setMotorRelaysRaw(STOP_USE_NO_STATE, STOP_USE_NO_STATE);
  currentDirection = MOTOR_STOP;
  doorMovementDeadlineMs = 0;
}

void startDoorMovement(MotorDirection target) {
  stopMotor();
  delay(RELAY_DEADTIME_MS);

  if (target == MOTOR_OPEN) {
    setMotorRelaysRaw(true, false);
    doorState = DOOR_OPENING;
  } else if (target == MOTOR_CLOSE) {
    setMotorRelaysRaw(false, true);
    doorState = DOOR_CLOSING;
  } else {
    return;
  }

  currentDirection = target;
  doorMovementDeadlineMs = millis() + DOOR_TRAVEL_TIME_MS;
}

bool doorMoving() {
  return doorState == DOOR_OPENING || doorState == DOOR_CLOSING;
}

bool canOpenDoor() {
  return doorState == DOOR_UNKNOWN || doorState == DOOR_CLOSED;
}

bool canCloseDoor() {
  return doorState == DOOR_UNKNOWN || doorState == DOOR_OPEN;
}

ControlMode readControlMode() {
  if (!inputStatesValid) return MODE_INVALID;
  if (di5State && !di6State) return MODE_AUTOMATIC;
  if (di5State && di6State) return MODE_MANUAL_OPEN;
  if (!di5State && di6State) return MODE_MANUAL_CLOSE;
  return MODE_INVALID;
}

bool remoteCommandsAllowed() {
  return controlMode == MODE_AUTOMATIC && !manualOverrideActive;
}

const char* controlModeText() {
  if (manualOverrideActive && controlMode == MODE_AUTOMATIC) {
    return "Ручной: выполняется последняя команда";
  }
  switch (controlMode) {
    case MODE_AUTOMATIC: return "Автоматический";
    case MODE_MANUAL_OPEN: return "Ручной: открыть";
    case MODE_MANUAL_CLOSE: return "Ручной: закрыть";
    default: return "Недопустимое состояние входов";
  }
}

bool requestDoorMovement(MotorDirection target) {
  if (target == MOTOR_OPEN && canOpenDoor()) {
    startDoorMovement(MOTOR_OPEN);
    return true;
  }
  if (target == MOTOR_CLOSE && canCloseDoor()) {
    startDoorMovement(MOTOR_CLOSE);
    return true;
  }
  return false;
}

void updateDoorState() {
  if (!doorMoving() || doorMovementDeadlineMs == 0) {
    return;
  }
  if (static_cast<int32_t>(millis() - doorMovementDeadlineMs) < 0) {
    return;
  }

  doorState = currentDirection == MOTOR_OPEN ? DOOR_OPEN : DOOR_CLOSED;
  doorMovementDeadlineMs = 0;
}

void updateControlMode() {
  controlMode = readControlMode();

  if (controlMode == MODE_MANUAL_OPEN || controlMode == MODE_MANUAL_CLOSE) {
    manualOverrideActive = true;
    manualCommandPending = true;
    manualTarget = controlMode == MODE_MANUAL_OPEN ? MOTOR_OPEN : MOTOR_CLOSE;
  }

  if (manualCommandPending && !doorMoving()) {
    bool targetReached =
        (manualTarget == MOTOR_OPEN && doorState == DOOR_OPEN) ||
        (manualTarget == MOTOR_CLOSE && doorState == DOOR_CLOSED);
    if (targetReached) {
      manualCommandPending = false;
    } else {
      startDoorMovement(manualTarget);
    }
  }

  if (controlMode == MODE_AUTOMATIC && !doorMoving() && !manualCommandPending) {
    manualOverrideActive = false;
    manualTarget = MOTOR_STOP;
  }
}

const char* doorStateText() {
  switch (doorState) {
    case DOOR_OPENING: return "Открывается";
    case DOOR_OPEN: return "Открыта";
    case DOOR_CLOSING: return "Закрывается";
    case DOOR_CLOSED: return "Закрыта";
    default: return "Положение неизвестно";
  }
}

String statusJson() {
  String json = "{";
  json += "\"direction\":" + String((uint16_t)currentDirection) + ",";
  json += "\"direction_text\":\"" + String(doorStateText()) + "\",";
  json += "\"moving\":" + String(doorMoving() ? "true" : "false") + ",";
  json += "\"control_mode\":\"" + String(controlModeText()) + "\",";
  json += "\"remote_enabled\":" + String(remoteCommandsAllowed() ? "true" : "false") + ",";
  json += "\"can_open\":" + String(remoteCommandsAllowed() && canOpenDoor() ? "true" : "false") + ",";
  json += "\"can_close\":" + String(remoteCommandsAllowed() && canCloseDoor() ? "true" : "false") + ",";
  json += "\"di5\":" + String(di5State ? 1 : 0) + ",";
  json += "\"di6\":" + String(di6State ? 1 : 0);
  json += "}";
  return json;
}

String generateHTML() {
  String html = R"rawliteral(
<!doctype html>
<html lang="ru">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <title>Дверь печи</title>
  <style>
    :root {
      --bg: #f4f6f8;
      --card: #ffffff;
      --text: #1f2937;
      --muted: #6b7280;
      --open: #1d4ed8;
      --close: #b91c1c;
    }
    * { box-sizing: border-box; }
    body { margin: 0; min-height: 100vh; display: grid; place-items: center; font-family: "Segoe UI", Tahoma, sans-serif; color: var(--text); background: linear-gradient(165deg, #e2e8f0 0%, var(--bg) 50%, #dbeafe 100%); padding: 16px; }
    .panel { width: min(440px, 100%); background: var(--card); border-radius: 14px; box-shadow: 0 14px 28px rgba(15, 23, 42, .14); padding: 20px; }
    h1 { margin: 0 0 8px; font-size: 24px; }
    .sub { margin: 0 0 16px; color: var(--muted); font-size: 14px; }
    .status { margin-bottom: 12px; border: 1px solid #cbd5e1; border-radius: 10px; padding: 10px 12px; background: #f8fafc; font-size: 15px; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 10px; }
    button { width: 100%; height: 54px; border: 0; border-radius: 10px; color: #fff; font-size: 18px; font-weight: 700; cursor: pointer; }
    .open { background: var(--open); }
    .close { background: var(--close); }
  </style>
</head>
<body>
  <main class="panel">
    <h1>Управление дверью</h1>
    <p class="sub">Противоположная команда становится доступна после завершения 6-секундного хода.</p>
    <div class="status">Состояние двери: <b id="direction">)rawliteral";
  html += String(doorStateText());
  html += R"rawliteral(</b></div>
    <div class="status">DI5: <b id="di5">)rawliteral";
  html += di5State ? "1" : "0";
  html += R"rawliteral(</b> &nbsp; DI6: <b id="di6">)rawliteral";
  html += di6State ? "1" : "0";
  html += R"rawliteral(</b></div>
    <div class="status">Режим: <b id="control-mode">)rawliteral";
  html += String(controlModeText());
  html += R"rawliteral(</b></div>
    <div class="row">
      <form action="/open" method="post"><button id="open-button" class="open" type="submit" )rawliteral";
  if (!remoteCommandsAllowed() || !canOpenDoor()) html += "disabled";
  html += R"rawliteral(>Открыть</button></form>
      <form action="/close" method="post"><button id="close-button" class="close" type="submit" )rawliteral";
  if (!remoteCommandsAllowed() || !canCloseDoor()) html += "disabled";
  html += R"rawliteral(>Закрыть</button></form>
    </div>
  </main>
  <script>
    async function refreshStatus() {
      try {
        const response = await fetch('/api/status', { cache: 'no-store' });
        if (!response.ok) return;
        const data = await response.json();
        document.getElementById('direction').textContent = data.direction_text;
        document.getElementById('di5').textContent = data.di5;
        document.getElementById('di6').textContent = data.di6;
        document.getElementById('control-mode').textContent = data.control_mode;
        document.getElementById('open-button').disabled = !data.can_open;
        document.getElementById('close-button').disabled = !data.can_close;
      } catch (e) {}
    }
    setInterval(refreshStatus, 500);
    refreshStatus();
  </script>
</body>
</html>
)rawliteral";
  return html;
}

void syncModbusRegistersFromState() {
  mb.Hreg(REG_DIRECTION, static_cast<uint16_t>(currentDirection));
  uint16_t statusBits = 0;
  if (doorMoving()) statusBits |= (1U << 0);
  if (doorState == DOOR_OPEN) statusBits |= (1U << 1);
  if (doorState == DOOR_CLOSED) statusBits |= (1U << 2);
  if (remoteCommandsAllowed()) statusBits |= (1U << 3);
  if (controlMode == MODE_MANUAL_OPEN) statusBits |= (1U << 4);
  if (controlMode == MODE_MANUAL_CLOSE) statusBits |= (1U << 5);
  if (controlMode == MODE_INVALID) statusBits |= (1U << 6);
  mb.Hreg(REG_STATUS_BITS, statusBits);
}

void handleModbusCommand() {
  uint16_t cmd = mb.Hreg(REG_COMMAND);
  if (cmd == CMD_NONE) return;

  uint16_t result = CMD_RES_OK;
  switch (cmd) {
    case CMD_DOOR_OPEN:
      if (!remoteCommandsAllowed() || !requestDoorMovement(MOTOR_OPEN)) result = CMD_RES_BUSY;
      break;
    case CMD_DOOR_CLOSE:
      if (!remoteCommandsAllowed() || !requestDoorMovement(MOTOR_CLOSE)) result = CMD_RES_BUSY;
      break;
    default: result = CMD_RES_UNKNOWN_CMD; break;
  }

  mb.Hreg(REG_COMMAND_RESULT, result);
  mb.Hreg(REG_COMMAND, CMD_NONE);
}

void handleRoot() { server.send(200, "text/html; charset=UTF-8", generateHTML()); }
void handleStatus() { server.send(200, "application/json; charset=UTF-8", statusJson()); }
void handleOpen() {
  if (remoteCommandsAllowed()) requestDoorMovement(MOTOR_OPEN);
  handleRoot();
}
void handleClose() {
  if (remoteCommandsAllowed()) requestDoorMovement(MOTOR_CLOSE);
  handleRoot();
}

void handleRedirectToRoot() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  initInputExpander();
  updateInputStates();
  stopMotor();

  RS485Serial.begin(RS485_BAUDRATE, SERIAL_8N1, RS485_RX_PIN, RS485_TX_PIN);
  mb.begin(&RS485Serial);
  mb.slave(MODBUS_SLAVE_ID);
  mb.addHreg(REG_COMMAND, 0, REG_REG_COUNT);
  mb.Hreg(REG_COMMAND_RESULT, CMD_RES_OK);
  syncModbusRegistersFromState();

  WiFi.softAP(ap_ssid, ap_password);
  delay(200);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/open", HTTP_POST, handleOpen);
  server.on("/close", HTTP_POST, handleClose);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/generate_204", HTTP_GET, handleRedirectToRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRedirectToRoot);
  server.on("/fwlink", HTTP_GET, handleRedirectToRoot);
  server.onNotFound(handleRedirectToRoot);

  server.begin();
}

void loop() {
  dnsServer.processNextRequest();
  updateInputStates();
  updateDoorState();
  updateControlMode();
  mb.task();
  handleModbusCommand();
  syncModbusRegistersFromState();
  server.handleClient();
}

