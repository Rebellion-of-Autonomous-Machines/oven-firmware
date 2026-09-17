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

// Physical relay indexes (0-based on PCF8574)
static const uint8_t RELAY_MOTOR_A = 4; // Relay 5
static const uint8_t RELAY_MOTOR_B = 5; // Relay 6

// false = NC/NC stop, true = NO/NO stop
static const bool STOP_USE_NO_STATE = false;
static const uint16_t RELAY_DEADTIME_MS = 120;
static const uint32_t MOTOR_RUN_TIMEOUT_MS = 30000;

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
  CMD_DOOR_CLOSE = 2,
  CMD_DOOR_STOP = 3
};

enum ModbusCommandResult : uint16_t {
  CMD_RES_OK = 0,
  CMD_RES_UNKNOWN_CMD = 1
};

enum MotorDirection : uint8_t {
  MOTOR_STOP = 0,
  MOTOR_OPEN = 1,
  MOTOR_CLOSE = 2
};

// PCF8574 relay modules are usually active-low: true means relay coil ON.
bool relayStates[6] = {false, false, false, false, false, false};
MotorDirection currentDirection = MOTOR_STOP;
uint32_t motorAutoStopDeadlineMs = 0;
uint32_t motorRunStartedMs = 0;
uint32_t lastRunDurationMs = 0;
MotorDirection lastRunDirection = MOTOR_STOP;
bool lastStopWasManual = false;
bool reverseCompensationPending = false;

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
  if (currentDirection != MOTOR_STOP && motorRunStartedMs != 0) {
    lastRunDurationMs = millis() - motorRunStartedMs;
    if (lastRunDurationMs > MOTOR_RUN_TIMEOUT_MS) {
      lastRunDurationMs = MOTOR_RUN_TIMEOUT_MS;
    }
    lastRunDirection = currentDirection;
  }
  setMotorRelaysRaw(STOP_USE_NO_STATE, STOP_USE_NO_STATE);
  currentDirection = MOTOR_STOP;
  motorAutoStopDeadlineMs = 0;
  motorRunStartedMs = 0;
}

void setMotorDirectionSafe(MotorDirection target) {
  if (currentDirection == target) {
    // Repeat command in same direction refreshes the 30s timer.
    motorAutoStopDeadlineMs = millis() + MOTOR_RUN_TIMEOUT_MS;
    motorRunStartedMs = millis();
    return;
  }

  stopMotor();
  delay(RELAY_DEADTIME_MS);

  if (target == MOTOR_OPEN) {
    setMotorRelaysRaw(true, false);
  } else if (target == MOTOR_CLOSE) {
    setMotorRelaysRaw(false, true);
  } else {
    stopMotor();
    return;
  }

  currentDirection = target;
  motorRunStartedMs = millis();
  bool useReverseCompensation = reverseCompensationPending &&
      lastStopWasManual &&
      lastRunDurationMs > 0 &&
      lastRunDirection != MOTOR_STOP &&
      target != lastRunDirection;
  if (useReverseCompensation) {
    motorAutoStopDeadlineMs = millis() + lastRunDurationMs;
  } else {
    motorAutoStopDeadlineMs = millis() + MOTOR_RUN_TIMEOUT_MS;
  }
  reverseCompensationPending = false;
  lastStopWasManual = false;
}

const char* directionText() {
  if (currentDirection == MOTOR_OPEN) {
    return "Открытие";
  }
  if (currentDirection == MOTOR_CLOSE) {
    return "Закрытие";
  }
  return "Стоп";
}

String statusJson() {
  String json = "{";
  json += "\"direction\":" + String((uint16_t)currentDirection) + ",";
  json += "\"direction_text\":\"" + String(directionText()) + "\",";
  json += "\"moving\":" + String(currentDirection == MOTOR_STOP ? "false" : "true");
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
      --stop: #0f766e;
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
    .stop { background: var(--stop); }
  </style>
</head>
<body>
  <main class="panel">
    <h1>Управление дверью</h1>
    <p class="sub">Концевики отключены. Движение останавливается только командой "Стоп".</p>
    <div class="status">Состояние двери: <b id="direction">)rawliteral";
  html += String(directionText());
  html += R"rawliteral(</b></div>
    <div class="row">
      <form action="/open" method="post"><button class="open" type="submit">Открыть</button></form>
      <form action="/close" method="post"><button class="close" type="submit">Закрыть</button></form>
    </div>
    <form action="/stop" method="post"><button class="stop" type="submit">Стоп</button></form>
  </main>
  <script>
    async function refreshStatus() {
      try {
        const response = await fetch('/api/status', { cache: 'no-store' });
        if (!response.ok) return;
        const data = await response.json();
        document.getElementById('direction').textContent = data.direction_text;
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
  if (currentDirection != MOTOR_STOP) statusBits |= (1U << 0);
  mb.Hreg(REG_STATUS_BITS, statusBits);
}

void handleModbusCommand() {
  uint16_t cmd = mb.Hreg(REG_COMMAND);
  if (cmd == CMD_NONE) return;

  uint16_t result = CMD_RES_OK;
  switch (cmd) {
    case CMD_DOOR_OPEN: setMotorDirectionSafe(MOTOR_OPEN); break;
    case CMD_DOOR_CLOSE: setMotorDirectionSafe(MOTOR_CLOSE); break;
    case CMD_DOOR_STOP:
      lastStopWasManual = true;
      reverseCompensationPending = true;
      stopMotor();
      break;
    default: result = CMD_RES_UNKNOWN_CMD; break;
  }

  mb.Hreg(REG_COMMAND_RESULT, result);
  mb.Hreg(REG_COMMAND, CMD_NONE);
}

void handleRoot() { server.send(200, "text/html; charset=UTF-8", generateHTML()); }
void handleStatus() { server.send(200, "application/json; charset=UTF-8", statusJson()); }
void handleOpen() { setMotorDirectionSafe(MOTOR_OPEN); handleRoot(); }
void handleClose() { setMotorDirectionSafe(MOTOR_CLOSE); handleRoot(); }
void handleStop() {
  lastStopWasManual = true;
  reverseCompensationPending = true;
  stopMotor();
  handleRoot();
}

void handleRedirectToRoot() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  Serial.begin(115200);
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
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
  server.on("/stop", HTTP_POST, handleStop);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/generate_204", HTTP_GET, handleRedirectToRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRedirectToRoot);
  server.on("/fwlink", HTTP_GET, handleRedirectToRoot);
  server.onNotFound(handleRedirectToRoot);

  server.begin();
}

void loop() {
  dnsServer.processNextRequest();
  mb.task();
  handleModbusCommand();
  if (currentDirection != MOTOR_STOP &&
      motorAutoStopDeadlineMs != 0 &&
      static_cast<int32_t>(millis() - motorAutoStopDeadlineMs) >= 0) {
    lastStopWasManual = false;
    reverseCompensationPending = false;
    stopMotor();
  }
  syncModbusRegistersFromState();
  server.handleClient();
}

