#include <Arduino.h>
#include <WiFi.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>

// --- I2C relays ---
static const uint8_t I2C_SDA_PIN = 4;
static const uint8_t I2C_SCL_PIN = 15;
static const uint8_t PCF8574_RELAY_ADDR = 0x24;
static const uint8_t PCF8574_INPUT_ADDR = 0x22;
static const uint8_t DI5_BIT = 4; // P4 on input PCF8574
static const uint8_t DI6_BIT = 5; // P5 on input PCF8574
static const uint32_t INPUT_DEBOUNCE_MS = 80;
static const uint32_t MANUAL_POSITION_CONFIRM_MS = 350;

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

// --- Ethernet / Modbus TCP (W5500) ---
Preferences preferences;
static const uint8_t W5500_CS_PIN = 5;
static const uint16_t MODBUS_TCP_PORT = 502;
static const uint8_t MODBUS_UNIT_ID = 1;
byte ethernetMac[] = {0x02, 0xA0, 0xC9, 0x00, 0x00, 0x63};
IPAddress ethernetIp(192, 168, 1, 52);
IPAddress ethernetDns(192, 168, 1, 1);
IPAddress ethernetGateway(192, 168, 1, 1);
IPAddress ethernetSubnet(255, 255, 255, 0);
bool ethernetUseDhcp = false;

class Esp32EthernetServer : public EthernetServer {
 public:
  explicit Esp32EthernetServer(uint16_t port) : EthernetServer(port) {}

  void begin(uint16_t port = 0) override {
    (void)port;
    EthernetServer::begin();
  }
};

Esp32EthernetServer modbusServer(MODBUS_TCP_PORT);
EthernetClient modbusClient;
String networkSettingsMessage;
bool networkSettingsSaved = false;

enum HoldingReg : uint16_t {
  HREG_STAT_MODE = 0,       // 40001
  HREG_STAT_ERROR = 1,      // 40002
  HREG_MAN_CONTROL_C = 2,   // 40003
  HREG_MAN_CONTROL_X = 3,   // 40004
  HREG_MAN_CONTROL_Y = 4,   // 40005
  HREG_MAN_CONTROL_Z = 5,   // 40006
  HREG_COMMAND = 6,         // 40007, legacy high-level command
  HREG_COMMAND_RESULT = 7,  // 40008
  HREG_DIRECTION = 8,       // 40009
  HREG_STATUS_BITS = 9,     // 40010
  HREG_DI_BITS = 10,        // 40011
  REG_REG_COUNT = 16
};

enum InputReg : uint16_t {
  IREG_TIME_ALL = 0,        // 30001
  IREG_TIME_HEAT = 1,       // 30002
  IREG_END_OPEN = 20,       // 30021
  IREG_END_CLOSED = 21      // 30022
};

static const uint16_t INPUT_REG_COUNT = IREG_END_CLOSED + 1;
uint16_t holdingRegs[REG_REG_COUNT] = {0};
uint16_t inputRegs[INPUT_REG_COUNT] = {0};

enum ModbusCommand : uint16_t {
  CMD_NONE = 0,
  CMD_DOOR_OPEN = 1,
  CMD_DOOR_CLOSE = 2
};

enum ModbusCommandResult : uint16_t {
  CMD_RES_OK = 0,
  CMD_RES_UNKNOWN_CMD = 1,
  CMD_RES_BUSY = 2,
  CMD_RES_MODE_REQUIRED = 3,
  CMD_RES_INVALID_ARG = 4,
  CMD_RES_REMOTE_LOCKED = 5,
  CMD_RES_RELAY_CONTROL_FORBIDDEN = 6
};

enum AutomationState : uint16_t {
  STAT_INITIAL = 1,
  STAT_SCENARIO_ACTIVE = 11,
  STAT_SCENARIO_COMPLETE = 19,
  STAT_MANUAL = 20,
  STAT_EMERGENCY_STOP = 30,
  STAT_ERROR = 99
};

enum AutomationError : uint16_t {
  ERROR_NONE = 0,
  ERROR_MANUAL_MODE_REQUIRED = 100,
  ERROR_COMMAND_DURING_MOVEMENT = 101,
  ERROR_INVALID_COMMAND = 102,
  ERROR_REMOTE_CONTROL_LOCKED = 103,
  ERROR_DIRECT_RELAY_CONTROL_FORBIDDEN = 105
};

enum ManualControlCommand : uint16_t {
  MAN_CMD_NONE = 0,
  MAN_CMD_OPEN = 1,
  MAN_CMD_CLOSE = 2,
  MAN_CMD_FORBIDDEN_RELAY_CONTROL = 5
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
bool rawDi5State = true;
bool rawDi6State = false;
bool inputStatesValid = false;
uint32_t inputStateChangedMs = 0;
ControlMode controlMode = MODE_INVALID;
bool manualOverrideActive = false;
bool manualCommandPending = false;
MotorDirection manualTarget = MOTOR_STOP;
bool selectorReadyForManualCommand = false;
ControlMode manualCandidateMode = MODE_INVALID;
uint32_t manualCandidateStartedMs = 0;
uint16_t automationState = STAT_INITIAL;
uint16_t publishedAutomationState = STAT_INITIAL;
uint16_t automationError = ERROR_NONE;
uint16_t commandResult = CMD_RES_OK;
uint64_t totalRuntimeSeconds = 0;
uint64_t lastSavedRuntimeSeconds = 0;
uint32_t runtimeTickMs = 0;

static const uint32_t RUNTIME_SAVE_INTERVAL_SECONDS = 600;

void loadRuntimeTelemetry() {
  preferences.begin("telemetry", true);
  totalRuntimeSeconds = preferences.getULong64("runtime_s", 0);
  preferences.end();
  lastSavedRuntimeSeconds = totalRuntimeSeconds;
  runtimeTickMs = millis();
}

void saveRuntimeTelemetry() {
  preferences.begin("telemetry", false);
  preferences.putULong64("runtime_s", totalRuntimeSeconds);
  preferences.end();
  lastSavedRuntimeSeconds = totalRuntimeSeconds;
}

void updateRuntimeTelemetry() {
  uint32_t now = millis();
  uint32_t elapsedMs = now - runtimeTickMs;
  if (elapsedMs >= 1000) {
    uint32_t elapsedSeconds = elapsedMs / 1000;
    totalRuntimeSeconds += elapsedSeconds;
    runtimeTickMs += elapsedSeconds * 1000UL;
  }

  if ((totalRuntimeSeconds - lastSavedRuntimeSeconds) >= RUNTIME_SAVE_INTERVAL_SECONDS) {
    saveRuntimeTelemetry();
  }
}

bool parseIPv4(const String& text, IPAddress& address) {
  String value = text;
  value.trim();
  return address.fromString(value);
}

void loadNetworkSettings() {
  Preferences networkPreferences;
  if (!networkPreferences.begin("ethernet", true)) return;

  IPAddress address;
  if (parseIPv4(networkPreferences.getString("ip", ethernetIp.toString()), address)) ethernetIp = address;
  if (parseIPv4(networkPreferences.getString("subnet", ethernetSubnet.toString()), address)) ethernetSubnet = address;
  if (parseIPv4(networkPreferences.getString("gateway", ethernetGateway.toString()), address)) ethernetGateway = address;
  if (parseIPv4(networkPreferences.getString("dns", ethernetDns.toString()), address)) ethernetDns = address;
  ethernetUseDhcp = networkPreferences.getBool("dhcp", false);
  networkPreferences.end();
}

bool saveNetworkSettings() {
  Preferences networkPreferences;
  if (!networkPreferences.begin("ethernet", false)) return false;

  bool saved = networkPreferences.putString("ip", ethernetIp.toString()) > 0;
  saved &= networkPreferences.putString("subnet", ethernetSubnet.toString()) > 0;
  saved &= networkPreferences.putString("gateway", ethernetGateway.toString()) > 0;
  saved &= networkPreferences.putString("dns", ethernetDns.toString()) > 0;
  saved &= networkPreferences.putBool("dhcp", ethernetUseDhcp) > 0;
  networkPreferences.end();
  return saved;
}

void applyEthernetSettings() {
  if (modbusClient) modbusClient.stop();
  if (ethernetUseDhcp) {
    Ethernet.begin(ethernetMac, 10000, 2000);
  } else {
    Ethernet.begin(ethernetMac, ethernetIp, ethernetDns, ethernetGateway, ethernetSubnet);
  }
  delay(200);
  modbusServer.begin();
}

const char* ethernetConnectionText() {
  if (Ethernet.hardwareStatus() == EthernetNoHardware) return "W5500 не обнаружен";
  if (Ethernet.linkStatus() == LinkOFF) return "Сетевой кабель отключен";
  if (Ethernet.linkStatus() == Unknown) return "Состояние линии неизвестно";
  if (Ethernet.localIP() == IPAddress(0, 0, 0, 0)) {
    return ethernetUseDhcp ? "Адрес DHCP не получен" : "IP не назначен";
  }
  return "Подключено";
}

String modbusClientText() {
  if (!modbusClient || !modbusClient.connected()) return "Нет подключения";
  return modbusClient.remoteIP().toString() + ":" + String(modbusClient.remotePort());
}

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
  bool newDi5State = bitRead(inputByte, DI5_BIT);
  bool newDi6State = bitRead(inputByte, DI6_BIT);

  if (!inputStatesValid) {
    rawDi5State = newDi5State;
    rawDi6State = newDi6State;
    di5State = newDi5State;
    di6State = newDi6State;
    inputStateChangedMs = millis();
    inputStatesValid = true;
    return;
  }

  if (newDi5State != rawDi5State || newDi6State != rawDi6State) {
    rawDi5State = newDi5State;
    rawDi6State = newDi6State;
    inputStateChangedMs = millis();
    return;
  }

  if ((millis() - inputStateChangedMs) >= INPUT_DEBOUNCE_MS) {
    di5State = rawDi5State;
    di6State = rawDi6State;
  }
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
  selectorReadyForManualCommand = false;
  manualCandidateMode = MODE_INVALID;
  manualCandidateStartedMs = 0;
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
  automationState = STAT_SCENARIO_ACTIVE;
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

bool doorAtTarget(MotorDirection target) {
  return (target == MOTOR_OPEN && doorState == DOOR_OPEN) ||
      (target == MOTOR_CLOSE && doorState == DOOR_CLOSED);
}

ControlMode readControlMode() {
  if (!inputStatesValid) return MODE_INVALID;
  if (di5State && !di6State) return MODE_AUTOMATIC;
  if (di5State && di6State) return MODE_MANUAL_OPEN;
  if (!di5State && di6State) return MODE_MANUAL_CLOSE;
  return MODE_INVALID;
}

bool remoteCommandsAllowed() {
  return controlMode == MODE_AUTOMATIC &&
      !manualOverrideActive &&
      !doorMoving() &&
      automationState != STAT_EMERGENCY_STOP &&
      automationState != STAT_ERROR;
}

const char* controlModeText() {
  if (doorMoving()) {
    return "Движение: все команды заблокированы";
  }
  if (manualCommandPending) {
    return manualTarget == MOTOR_OPEN
        ? "Ручной: открыть, верните ключ в Автомат"
        : "Ручной: закрыть, верните ключ в Автомат";
  }
  if (selectorReadyForManualCommand &&
      (controlMode == MODE_MANUAL_OPEN || controlMode == MODE_MANUAL_CLOSE)) {
    return "Ожидание стабилизации ключа";
  }
  if (!selectorReadyForManualCommand && controlMode != MODE_AUTOMATIC) {
    return "Верните ключ в Автомат";
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

  DoorState completedState = currentDirection == MOTOR_OPEN ? DOOR_OPEN : DOOR_CLOSED;
  stopMotor();
  doorState = completedState;
  if (automationState != STAT_ERROR && automationState != STAT_EMERGENCY_STOP) {
    automationState = STAT_SCENARIO_COMPLETE;
  }
}

void updateControlMode() {
  controlMode = readControlMode();

  // Selector changes during travel are observed only for indication.
  // They never arm or queue a command.
  if (doorMoving()) {
    return;
  }

  if (controlMode == MODE_MANUAL_OPEN || controlMode == MODE_MANUAL_CLOSE) {
    manualOverrideActive = true;
    // 0/1 is an unambiguous CLOSE position. It may replace a previously
    // latched OPEN command, while transitional 1/1 can never replace CLOSE.
    if (controlMode == MODE_MANUAL_CLOSE &&
        (selectorReadyForManualCommand || manualCommandPending)) {
      manualCommandPending = true;
      manualTarget = MOTOR_CLOSE;
      selectorReadyForManualCommand = false;
      manualCandidateMode = MODE_INVALID;
      manualCandidateStartedMs = 0;
    } else if (controlMode == MODE_MANUAL_OPEN &&
               selectorReadyForManualCommand &&
               !manualCommandPending) {
      // 1/1 can occur while the selector travels toward CLOSE, therefore
      // OPEN requires an additional confirmation interval.
      if (manualCandidateMode != MODE_MANUAL_OPEN) {
        manualCandidateMode = controlMode;
        manualCandidateStartedMs = millis();
      } else if ((millis() - manualCandidateStartedMs) >= MANUAL_POSITION_CONFIRM_MS) {
        manualCommandPending = true;
        manualTarget = MOTOR_OPEN;
        selectorReadyForManualCommand = false;
        manualCandidateMode = MODE_INVALID;
        manualCandidateStartedMs = 0;
      }
    }
    return;
  }

  if (controlMode == MODE_AUTOMATIC && manualCommandPending) {
    bool targetReached =
        (manualTarget == MOTOR_OPEN && doorState == DOOR_OPEN) ||
        (manualTarget == MOTOR_CLOSE && doorState == DOOR_CLOSED);
    manualCommandPending = false;
    if (targetReached) {
      manualOverrideActive = false;
      manualTarget = MOTOR_STOP;
      selectorReadyForManualCommand = true;
    } else {
      startDoorMovement(manualTarget);
    }
    return;
  }

  if (controlMode == MODE_AUTOMATIC) {
    manualOverrideActive = false;
    manualTarget = MOTOR_STOP;
    selectorReadyForManualCommand = true;
    manualCandidateMode = MODE_INVALID;
    manualCandidateStartedMs = 0;
  } else if (controlMode == MODE_INVALID) {
    manualOverrideActive = true;
    if (!manualCommandPending) {
      manualTarget = MOTOR_STOP;
      selectorReadyForManualCommand = false;
      manualCandidateMode = MODE_INVALID;
      manualCandidateStartedMs = 0;
    }
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
  json += "\"di6\":" + String(di6State ? 1 : 0) + ",";
  json += "\"automation_state\":" + String(automationState) + ",";
  json += "\"automation_error\":" + String(automationError) + ",";
  json += "\"runtime_hours\":" + String(static_cast<uint32_t>(totalRuntimeSeconds / 3600ULL));
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
    .panel { width: min(560px, 100%); background: var(--card); border-radius: 14px; box-shadow: 0 14px 28px rgba(15, 23, 42, .14); padding: 20px; }
    h1 { margin: 0 0 8px; font-size: 24px; }
    .sub { margin: 0 0 16px; color: var(--muted); font-size: 14px; }
    .status { margin-bottom: 12px; border: 1px solid #cbd5e1; border-radius: 10px; padding: 10px 12px; background: #f8fafc; font-size: 15px; }
    .row { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-bottom: 10px; }
    button { width: 100%; height: 54px; border: 0; border-radius: 10px; color: #fff; font-size: 18px; font-weight: 700; cursor: pointer; }
    .open { background: var(--open); }
    .close { background: var(--close); }
    button:disabled, input:disabled { opacity: .5; cursor: not-allowed; }
    .network { margin-top: 18px; padding-top: 16px; border-top: 1px solid #cbd5e1; }
    .network h2 { margin: 0 0 10px; font-size: 19px; }
    .network-state { margin: 5px 0; color: var(--muted); font-size: 14px; }
    .network-form { display: grid; grid-template-columns: 1fr 1fr; gap: 10px; margin-top: 14px; }
    .field { display: grid; gap: 5px; color: var(--muted); font-size: 13px; }
    .field input, .field select { width: 100%; border: 1px solid #cbd5e1; border-radius: 8px; padding: 10px; font-size: 15px; background: #fff; }
    .network-form button { grid-column: 1 / -1; height: 46px; background: #0f766e; font-size: 16px; }
    .notice { margin: 10px 0; padding: 10px; border-radius: 8px; font-size: 14px; }
    .notice-ok { background: #e7f7ef; color: #08734d; }
    .notice-error { background: #fdecea; color: var(--close); }
    @media (max-width: 520px) { .network-form { grid-template-columns: 1fr; } .network-form button { grid-column: 1; } }
  </style>
</head>
<body>
  <main class="panel">
    <h1>Управление дверью</h1>
    <p class="sub">При включении дверь закрывается 6 секунд. Затем доступно только противоположное направление.</p>
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
    <section class="network">
      <h2>Ethernet / Modbus TCP</h2>
)rawliteral";
  if (networkSettingsMessage.length() > 0) {
    html += "<div class=\"notice ";
    html += networkSettingsSaved ? "notice-ok" : "notice-error";
    html += "\">" + networkSettingsMessage + "</div>";
  }
  html += R"rawliteral(
      <div class="network-state">Состояние W5500: <b>)rawliteral";
  html += ethernetConnectionText();
  html += R"rawliteral(</b></div>
      <div class="network-state">Текущий IP: <b>)rawliteral";
  html += Ethernet.localIP().toString();
  html += R"rawliteral(</b></div>
      <div class="network-state">Порт Modbus TCP: <b>502</b>, Unit ID: <b>1</b></div>
      <div class="network-state">Клиент: <b>)rawliteral";
  html += modbusClientText();
  html += R"rawliteral(</b></div>
      <form id="network-form" class="network-form" action="/network" method="post">
        <label class="field">Получение адреса
          <select id="network-mode" name="mode" onchange="updateNetworkMode()">
            <option value="static")rawliteral";
  if (!ethernetUseDhcp) html += " selected";
  html += R"rawliteral(>Статический IP</option>
            <option value="dhcp")rawliteral";
  if (ethernetUseDhcp) html += " selected";
  html += R"rawliteral(>DHCP</option>
          </select>
        </label>
        <label class="field">IP-адрес<input class="static-network-field" name="ip" value=")rawliteral";
  html += ethernetIp.toString();
  html += R"rawliteral("></label>
        <label class="field">Маска подсети<input class="static-network-field" name="subnet" value=")rawliteral";
  html += ethernetSubnet.toString();
  html += R"rawliteral("></label>
        <label class="field">Шлюз<input class="static-network-field" name="gateway" value=")rawliteral";
  html += ethernetGateway.toString();
  html += R"rawliteral("></label>
        <label class="field">DNS<input class="static-network-field" name="dns" value=")rawliteral";
  html += ethernetDns.toString();
  html += R"rawliteral("></label>
        <button type="submit">Сохранить и применить</button>
      </form>
    </section>
  </main>
  <script>
    function updateNetworkMode() {
      const disabled = document.getElementById('network-mode').value === 'dhcp';
      document.querySelectorAll('.static-network-field').forEach(function(field) { field.disabled = disabled; });
    }
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
    updateNetworkMode();
    refreshStatus();
  </script>
</body>
</html>
)rawliteral";
  return html;
}

void setAutomationError(uint16_t errorCode, uint16_t resultCode) {
  automationError = errorCode;
  automationState = STAT_ERROR;
  commandResult = resultCode;
}

void clearManualControlRegisters() {
  holdingRegs[HREG_MAN_CONTROL_C] = 0;
  holdingRegs[HREG_MAN_CONTROL_X] = 0;
  holdingRegs[HREG_MAN_CONTROL_Y] = 0;
  holdingRegs[HREG_MAN_CONTROL_Z] = 0;
}

void handleAutomationStateWrite() {
  uint16_t requestedState = holdingRegs[HREG_STAT_MODE];
  if (requestedState == publishedAutomationState) {
    return;
  }

  if (requestedState == STAT_EMERGENCY_STOP) {
    stopMotor();
    doorState = DOOR_UNKNOWN;
    automationState = STAT_EMERGENCY_STOP;
    automationError = ERROR_NONE;
    commandResult = CMD_RES_OK;
    return;
  }

  if (requestedState == STAT_SCENARIO_ACTIVE) {
    if (automationState != STAT_INITIAL) {
      setAutomationError(ERROR_INVALID_COMMAND, CMD_RES_INVALID_ARG);
      return;
    }
    if (!remoteCommandsAllowed()) {
      setAutomationError(ERROR_REMOTE_CONTROL_LOCKED, CMD_RES_REMOTE_LOCKED);
      return;
    }
    if (doorAtTarget(MOTOR_OPEN)) {
      automationState = STAT_SCENARIO_COMPLETE;
      automationError = ERROR_NONE;
      commandResult = CMD_RES_OK;
      return;
    }
    if (!requestDoorMovement(MOTOR_OPEN)) {
      setAutomationError(ERROR_COMMAND_DURING_MOVEMENT, CMD_RES_BUSY);
      return;
    }
    automationError = ERROR_NONE;
    commandResult = CMD_RES_OK;
    return;
  }

  if (requestedState < 1 || requestedState > 20) {
    setAutomationError(ERROR_INVALID_COMMAND, CMD_RES_INVALID_ARG);
    return;
  }

  if (doorMoving()) {
    setAutomationError(ERROR_COMMAND_DURING_MOVEMENT, CMD_RES_BUSY);
    return;
  }

  if (requestedState == STAT_MANUAL &&
      (controlMode != MODE_AUTOMATIC || manualOverrideActive)) {
    setAutomationError(ERROR_REMOTE_CONTROL_LOCKED, CMD_RES_REMOTE_LOCKED);
    return;
  }

  automationError = ERROR_NONE;
  automationState = requestedState;
  commandResult = CMD_RES_OK;
}

void handleManualControlCommand() {
  uint16_t command = holdingRegs[HREG_MAN_CONTROL_C];
  if (command == MAN_CMD_NONE) {
    return;
  }

  uint16_t x = holdingRegs[HREG_MAN_CONTROL_X];
  uint16_t y = holdingRegs[HREG_MAN_CONTROL_Y];
  uint16_t z = holdingRegs[HREG_MAN_CONTROL_Z];
  clearManualControlRegisters();

  if (command == MAN_CMD_FORBIDDEN_RELAY_CONTROL) {
    setAutomationError(ERROR_DIRECT_RELAY_CONTROL_FORBIDDEN, CMD_RES_RELAY_CONTROL_FORBIDDEN);
    return;
  }

  // Preserve the more specific error raised while processing Stat_Mode
  // from the same Write Multiple Registers transaction.
  if (automationState == STAT_ERROR && commandResult != CMD_RES_OK) {
    return;
  }

  if (automationState != STAT_MANUAL) {
    setAutomationError(ERROR_MANUAL_MODE_REQUIRED, CMD_RES_MODE_REQUIRED);
    return;
  }
  if (!remoteCommandsAllowed()) {
    uint16_t errorCode = doorMoving() ? ERROR_COMMAND_DURING_MOVEMENT : ERROR_REMOTE_CONTROL_LOCKED;
    uint16_t resultCode = doorMoving() ? CMD_RES_BUSY : CMD_RES_REMOTE_LOCKED;
    setAutomationError(errorCode, resultCode);
    return;
  }

  if (command == MAN_CMD_OPEN || command == MAN_CMD_CLOSE) {
    if (x != 0 || y != 0 || z != 0) {
      setAutomationError(ERROR_INVALID_COMMAND, CMD_RES_INVALID_ARG);
      return;
    }
    MotorDirection target = command == MAN_CMD_OPEN ? MOTOR_OPEN : MOTOR_CLOSE;
    if (doorAtTarget(target)) {
      commandResult = CMD_RES_OK;
      return;
    }
    if (!requestDoorMovement(target)) {
      setAutomationError(ERROR_COMMAND_DURING_MOVEMENT, CMD_RES_BUSY);
      return;
    }
    commandResult = CMD_RES_OK;
    return;
  }

  setAutomationError(ERROR_INVALID_COMMAND, CMD_RES_UNKNOWN_CMD);
}

void handleLegacyModbusCommand() {
  uint16_t cmd = holdingRegs[HREG_COMMAND];
  if (cmd == CMD_NONE) return;
  holdingRegs[HREG_COMMAND] = CMD_NONE;

  if (!remoteCommandsAllowed()) {
    setAutomationError(
        doorMoving() ? ERROR_COMMAND_DURING_MOVEMENT : ERROR_REMOTE_CONTROL_LOCKED,
        doorMoving() ? CMD_RES_BUSY : CMD_RES_REMOTE_LOCKED);
    return;
  }

  MotorDirection target;
  if (cmd == CMD_DOOR_OPEN) {
    target = MOTOR_OPEN;
  } else if (cmd == CMD_DOOR_CLOSE) {
    target = MOTOR_CLOSE;
  } else {
    setAutomationError(ERROR_INVALID_COMMAND, CMD_RES_UNKNOWN_CMD);
    return;
  }

  if (doorAtTarget(target)) {
    commandResult = CMD_RES_OK;
    return;
  }
  if (!requestDoorMovement(target)) {
    setAutomationError(ERROR_COMMAND_DURING_MOVEMENT, CMD_RES_BUSY);
    return;
  }
  commandResult = CMD_RES_OK;
}

void syncModbusRegistersFromState() {
  holdingRegs[HREG_STAT_MODE] = automationState;
  publishedAutomationState = automationState;
  holdingRegs[HREG_STAT_ERROR] = automationError;
  holdingRegs[HREG_DIRECTION] = static_cast<uint16_t>(currentDirection);
  uint16_t statusBits = 0;
  if (doorMoving()) statusBits |= (1U << 0);
  if (doorState == DOOR_OPEN) statusBits |= (1U << 1);
  if (doorState == DOOR_CLOSED) statusBits |= (1U << 2);
  if (remoteCommandsAllowed()) statusBits |= (1U << 3);
  if (controlMode == MODE_MANUAL_OPEN) statusBits |= (1U << 4);
  if (controlMode == MODE_MANUAL_CLOSE) statusBits |= (1U << 5);
  if (controlMode == MODE_INVALID) statusBits |= (1U << 6);
  if (manualCommandPending) statusBits |= (1U << 7);
  holdingRegs[HREG_STATUS_BITS] = statusBits;
  holdingRegs[HREG_COMMAND_RESULT] = commandResult;

  uint16_t diBits = 0;
  if (di5State) diBits |= (1U << 0);
  if (di6State) diBits |= (1U << 1);
  if (inputStatesValid) diBits |= (1U << 2);
  holdingRegs[HREG_DI_BITS] = diBits;

  uint64_t runtimeHours64 = totalRuntimeSeconds / 3600ULL;
  uint16_t runtimeHours = runtimeHours64 > 65535ULL
      ? 65535
      : static_cast<uint16_t>(runtimeHours64);
  inputRegs[IREG_TIME_ALL] = runtimeHours;
  inputRegs[IREG_TIME_HEAT] = runtimeHours;
  inputRegs[IREG_END_OPEN] = doorState == DOOR_OPEN ? 1 : 0;
  inputRegs[IREG_END_CLOSED] = doorState == DOOR_CLOSED ? 1 : 0;
}

void sendModbusTcpResponse(EthernetClient& client, const uint8_t* requestHeader,
                           const uint8_t* pdu, uint16_t pduLength) {
  uint8_t header[7] = {
      requestHeader[0], requestHeader[1], 0, 0,
      highByte(pduLength + 1), lowByte(pduLength + 1), requestHeader[6]};
  client.write(header, sizeof(header));
  client.write(pdu, pduLength);
}

void sendModbusException(EthernetClient& client, const uint8_t* header,
                         uint8_t functionCode, uint8_t exceptionCode) {
  uint8_t pdu[2] = {static_cast<uint8_t>(functionCode | 0x80), exceptionCode};
  sendModbusTcpResponse(client, header, pdu, sizeof(pdu));
}

bool waitForClientBytes(EthernetClient& client, int count, uint16_t timeoutMs = 50) {
  uint32_t started = millis();
  while (client.connected() && client.available() < count &&
         millis() - started < timeoutMs) {
    delay(1);
  }
  return client.available() >= count;
}

bool validInputRegisterRange(uint16_t startAddress, uint16_t quantity) {
  if (quantity == 0) return false;
  uint32_t endAddress = static_cast<uint32_t>(startAddress) + quantity - 1;
  return (startAddress >= IREG_TIME_ALL && endAddress <= IREG_TIME_HEAT) ||
      (startAddress >= IREG_END_OPEN && endAddress <= IREG_END_CLOSED);
}

bool writableHoldingRegister(uint16_t address) {
  return address <= HREG_COMMAND;
}

void processModbusTcpRequest(EthernetClient& client) {
  if (!waitForClientBytes(client, 7)) return;

  uint8_t header[7];
  client.read(header, sizeof(header));
  uint16_t protocolId = word(header[2], header[3]);
  uint16_t length = word(header[4], header[5]);
  if (protocolId != 0 || length < 2 || length > 253) {
    client.stop();
    return;
  }

  uint16_t pduLength = length - 1;
  if (!waitForClientBytes(client, pduLength)) {
    client.stop();
    return;
  }

  uint8_t pdu[253];
  client.read(pdu, pduLength);
  uint8_t functionCode = pdu[0];
  if (header[6] != MODBUS_UNIT_ID && header[6] != 0) return;
  syncModbusRegistersFromState();

  if (functionCode == 0x03 || functionCode == 0x04) {
    if (pduLength < 5) {
      sendModbusException(client, header, functionCode, 0x03);
      return;
    }
    uint16_t startAddress = word(pdu[1], pdu[2]);
    uint16_t quantity = word(pdu[3], pdu[4]);
    bool validRange = functionCode == 0x03
        ? quantity > 0 && quantity <= 125 &&
            static_cast<uint32_t>(startAddress) + quantity <= REG_REG_COUNT
        : quantity <= 125 && validInputRegisterRange(startAddress, quantity);
    if (!validRange) {
      sendModbusException(client, header, functionCode, 0x02);
      return;
    }

    uint8_t response[252];
    response[0] = functionCode;
    response[1] = quantity * 2;
    for (uint16_t i = 0; i < quantity; i++) {
      uint16_t value = functionCode == 0x03
          ? holdingRegs[startAddress + i]
          : inputRegs[startAddress + i];
      response[2 + i * 2] = highByte(value);
      response[3 + i * 2] = lowByte(value);
    }
    sendModbusTcpResponse(client, header, response, 2 + quantity * 2);
    return;
  }

  if (functionCode == 0x06) {
    if (pduLength < 5) {
      sendModbusException(client, header, functionCode, 0x03);
      return;
    }
    uint16_t address = word(pdu[1], pdu[2]);
    if (!writableHoldingRegister(address)) {
      sendModbusException(client, header, functionCode, 0x02);
      return;
    }
    holdingRegs[address] = word(pdu[3], pdu[4]);
    sendModbusTcpResponse(client, header, pdu, 5);
    return;
  }

  if (functionCode == 0x10) {
    if (pduLength < 6) {
      sendModbusException(client, header, functionCode, 0x03);
      return;
    }
    uint16_t startAddress = word(pdu[1], pdu[2]);
    uint16_t quantity = word(pdu[3], pdu[4]);
    uint8_t byteCount = pdu[5];
    bool validRange = quantity > 0 && quantity <= 123 &&
        byteCount == quantity * 2 && pduLength >= 6 + byteCount &&
        static_cast<uint32_t>(startAddress) + quantity <= REG_REG_COUNT;
    if (validRange) {
      for (uint16_t i = 0; i < quantity; i++) {
        if (!writableHoldingRegister(startAddress + i)) {
          validRange = false;
          break;
        }
      }
    }
    if (!validRange) {
      sendModbusException(client, header, functionCode, 0x02);
      return;
    }
    for (uint16_t i = 0; i < quantity; i++) {
      holdingRegs[startAddress + i] = word(pdu[6 + i * 2], pdu[7 + i * 2]);
    }
    uint8_t response[5] = {functionCode, pdu[1], pdu[2], pdu[3], pdu[4]};
    sendModbusTcpResponse(client, header, response, sizeof(response));
    return;
  }

  sendModbusException(client, header, functionCode, 0x01);
}

void handleModbusTcp() {
  if (!modbusClient || !modbusClient.connected()) {
    EthernetClient newClient = modbusServer.available();
    if (newClient) modbusClient = newClient;
  }
  if (modbusClient && modbusClient.connected() && modbusClient.available()) {
    processModbusTcpRequest(modbusClient);
  }
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

void handleNetworkSettings() {
  if (doorMoving()) {
    networkSettingsSaved = false;
    networkSettingsMessage = "Настройки сети можно менять только после остановки двери.";
    server.send(409, "text/html; charset=UTF-8", generateHTML());
    return;
  }
  if (!server.hasArg("mode") ||
      (server.arg("mode") != "static" && server.arg("mode") != "dhcp")) {
    networkSettingsSaved = false;
    networkSettingsMessage = "Выберите режим получения IP-адреса.";
    server.send(400, "text/html; charset=UTF-8", generateHTML());
    return;
  }

  bool newUseDhcp = server.arg("mode") == "dhcp";
  IPAddress newIp, newSubnet, newGateway, newDns;
  if (!newUseDhcp &&
      (!server.hasArg("ip") || !server.hasArg("subnet") ||
       !server.hasArg("gateway") || !server.hasArg("dns") ||
       !parseIPv4(server.arg("ip"), newIp) ||
       !parseIPv4(server.arg("subnet"), newSubnet) ||
       !parseIPv4(server.arg("gateway"), newGateway) ||
       !parseIPv4(server.arg("dns"), newDns))) {
    networkSettingsSaved = false;
    networkSettingsMessage = "Ошибка: проверьте формат IPv4-адресов.";
    server.send(400, "text/html; charset=UTF-8", generateHTML());
    return;
  }

  bool oldDhcp = ethernetUseDhcp;
  IPAddress oldIp = ethernetIp;
  IPAddress oldSubnet = ethernetSubnet;
  IPAddress oldGateway = ethernetGateway;
  IPAddress oldDns = ethernetDns;
  ethernetUseDhcp = newUseDhcp;
  if (!newUseDhcp) {
    ethernetIp = newIp;
    ethernetSubnet = newSubnet;
    ethernetGateway = newGateway;
    ethernetDns = newDns;
  }

  if (!saveNetworkSettings()) {
    ethernetUseDhcp = oldDhcp;
    ethernetIp = oldIp;
    ethernetSubnet = oldSubnet;
    ethernetGateway = oldGateway;
    ethernetDns = oldDns;
    networkSettingsSaved = false;
    networkSettingsMessage = "Не удалось сохранить настройки сети.";
    server.send(500, "text/html; charset=UTF-8", generateHTML());
    return;
  }

  applyEthernetSettings();
  networkSettingsSaved = true;
  networkSettingsMessage = "Настройки сохранены. Текущий IP W5500: " +
      Ethernet.localIP().toString();
  handleRoot();
}

void handleRedirectToRoot() {
  server.sendHeader("Location", String("http://") + WiFi.softAPIP().toString() + "/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  Serial.begin(115200);
  loadRuntimeTelemetry();
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  initInputExpander();
  updateInputStates();
  stopMotor();

  holdingRegs[HREG_COMMAND_RESULT] = CMD_RES_OK;
  clearManualControlRegisters();
  syncModbusRegistersFromState();

  SPI.begin();
  Ethernet.init(W5500_CS_PIN);
  loadNetworkSettings();
  applyEthernetSettings();

  WiFi.softAP(ap_ssid, ap_password);
  delay(200);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());

  server.on("/", HTTP_GET, handleRoot);
  server.on("/open", HTTP_POST, handleOpen);
  server.on("/close", HTTP_POST, handleClose);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/network", HTTP_POST, handleNetworkSettings);
  server.on("/generate_204", HTTP_GET, handleRedirectToRoot);
  server.on("/hotspot-detect.html", HTTP_GET, handleRedirectToRoot);
  server.on("/fwlink", HTTP_GET, handleRedirectToRoot);
  server.onNotFound(handleRedirectToRoot);

  server.begin();

  Serial.print("AP IP: ");
  Serial.println(WiFi.softAPIP());
  Serial.print("Ethernet IP: ");
  Serial.println(Ethernet.localIP());

  // Establish a known safe position after every controller restart.
  startDoorMovement(MOTOR_CLOSE);
  syncModbusRegistersFromState();
}

void loop() {
  dnsServer.processNextRequest();
  updateInputStates();
  updateRuntimeTelemetry();
  updateDoorState();
  updateControlMode();
  if (ethernetUseDhcp) Ethernet.maintain();
  handleModbusTcp();
  handleAutomationStateWrite();
  handleManualControlCommand();
  handleLegacyModbusCommand();
  syncModbusRegistersFromState();
  server.handleClient();
}

