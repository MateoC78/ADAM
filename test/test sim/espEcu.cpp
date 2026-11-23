// =============================================================================
// ECU_Simulator.ino
// Simulates the Freematics OBD-II UART Adapter V2.1 for bench testing.
//
// Wiring (Simulator ESP32 → Main ESP32):
//   Simulator GPIO17 (TX) → Main ESP32 GPIO16 (RX)
//   Simulator GPIO16 (RX) → Main ESP32 GPIO17 (TX)
//   Simulator GND         → Main ESP32 GND
//
// The simulator responds to ELM327 AT commands and OBD-II PID queries
// with realistic, smoothly animated values — no car required.
// =============================================================================

#include <HardwareSerial.h>
#include <Arduino.h>

// --- Serial ports ---
// USB serial for debugging the simulator itself
#define DEBUG_BAUD    115200

// UART2 talks to your main ESP32 (mimics the Freematics adapter)
#define OBD_BAUD      9600
#define OBD_RX_PIN    16
#define OBD_TX_PIN    17

HardwareSerial obdPort(2);

// =============================================================================
// Simulated vehicle state
// All values drift smoothly over time to mimic a real idling/driving engine.
// =============================================================================
struct VehicleState {
  float rpm;            // 800 – 6500
  float speed;          // 0 – 180 km/h
  float coolantTemp;    // 20 – 105 °C
  float throttle;       // 0 – 100 %
  float engineLoad;     // 0 – 100 %
  float fuelPressure;   // 0 – 765 kPa
  float intakeTemp;     // 0 – 66 °C
  float maf;            // 0 – 655.35 g/s
  float fuelLevel;      // 0 – 100 %
  float baroPressure;   // 0 – 255 kPa (typically ~101)
  float voltage;        // Battery voltage 11.0 – 14.8 V
  int   dtcCount;       // Number of DTCs present
} vehicle;

// Drive scenario cycles through states automatically
enum DriveScenario {
  SCENARIO_IDLE,
  SCENARIO_ACCELERATING,
  SCENARIO_CRUISING,
  SCENARIO_DECELERATING,
  SCENARIO_COLD_START
};

DriveScenario currentScenario = SCENARIO_COLD_START;
unsigned long scenarioStartMs = 0;
unsigned long scenarioDurationMs = 15000;

// =============================================================================
// Utility: smooth lerp toward a target with a given step
// =============================================================================
float lerp(float current, float target, float step) {
  if (abs(target - current) < step) return target;
  return current + (target > current ? step : -step);
}

// =============================================================================
// Update vehicle state based on current scenario
// =============================================================================
void updateVehicleState() {
  unsigned long now = millis();

  // Advance scenario on timer
  if (now - scenarioStartMs > scenarioDurationMs) {
    scenarioStartMs = now;
    // Cycle through scenarios
    switch (currentScenario) {
      case SCENARIO_COLD_START:
        currentScenario = SCENARIO_IDLE;
        scenarioDurationMs = 10000;
        break;
      case SCENARIO_IDLE:
        currentScenario = SCENARIO_ACCELERATING;
        scenarioDurationMs = 8000;
        break;
      case SCENARIO_ACCELERATING:
        currentScenario = SCENARIO_CRUISING;
        scenarioDurationMs = 12000;
        break;
      case SCENARIO_CRUISING:
        currentScenario = SCENARIO_DECELERATING;
        scenarioDurationMs = 6000;
        break;
      case SCENARIO_DECELERATING:
        currentScenario = SCENARIO_IDLE;
        scenarioDurationMs = 10000;
        break;
    }
    Serial.print("[SIM] Scenario → ");
    Serial.println(currentScenario);
  }

  // Target values per scenario
  float targetRPM, targetSpeed, targetThrottle, targetLoad;

  switch (currentScenario) {
    case SCENARIO_COLD_START:
      targetRPM      = 1400 + sin(now / 800.0) * 100; // High cold idle
      targetSpeed    = 0;
      targetThrottle = 0;
      targetLoad     = 25;
      break;
    case SCENARIO_IDLE:
      targetRPM      = 820 + sin(now / 1200.0) * 30;  // Gentle idle flutter
      targetSpeed    = 0;
      targetThrottle = 0;
      targetLoad     = 15;
      break;
    case SCENARIO_ACCELERATING:
      targetRPM      = 4500 + sin(now / 400.0) * 200;
      targetSpeed    = 80;
      targetThrottle = 75 + sin(now / 600.0) * 10;
      targetLoad     = 80;
      break;
    case SCENARIO_CRUISING:
      targetRPM      = 2200 + sin(now / 2000.0) * 80;
      targetSpeed    = 100 + sin(now / 3000.0) * 5;
      targetThrottle = 30 + sin(now / 1500.0) * 5;
      targetLoad     = 40;
      break;
    case SCENARIO_DECELERATING:
      targetRPM      = 1000;
      targetSpeed    = 10;
      targetThrottle = 0;
      targetLoad     = 10;
      break;
  }

  // Smoothly move toward targets
  vehicle.rpm         = lerp(vehicle.rpm,         targetRPM,      15.0f);
  vehicle.speed       = lerp(vehicle.speed,       targetSpeed,    0.5f);
  vehicle.throttle    = lerp(vehicle.throttle,    targetThrottle, 1.0f);
  vehicle.engineLoad  = lerp(vehicle.engineLoad,  targetLoad,     0.5f);

  // Coolant warms up from cold start, stays in range
  float targetCoolant = (currentScenario == SCENARIO_COLD_START) ? 40 : 90;
  vehicle.coolantTemp = lerp(vehicle.coolantTemp, targetCoolant, 0.02f);

  // Derived values
  vehicle.maf          = (vehicle.engineLoad / 100.0f) * 50.0f + sin(now / 500.0) * 0.5f;
  vehicle.intakeTemp   = 25 + (vehicle.engineLoad / 100.0f) * 15;
  vehicle.fuelPressure = 300 + (vehicle.engineLoad / 100.0f) * 100;
  vehicle.fuelLevel    = max(0.0f, vehicle.fuelLevel - 0.00001f); // slowly drains
  vehicle.baroPressure = 101.3 + sin(now / 60000.0) * 0.5;       // nearly constant
  vehicle.voltage      = (currentScenario == SCENARIO_COLD_START)
                           ? 12.4 + sin(now / 200.0) * 0.3        // voltage dip on crank
                           : 14.1 + sin(now / 5000.0) * 0.2;      // alternator charging
}

// =============================================================================
// ELM327 response helpers
// =============================================================================
void sendPrompt() {
  obdPort.print("\r\n>");
}

void sendOK() {
  obdPort.print("OK");
  sendPrompt();
}

void sendError() {
  obdPort.print("?");
  sendPrompt();
}

void sendNoData() {
  obdPort.print("NO DATA");
  sendPrompt();
}

// Format and send a 1-byte PID response
void sendPID1(uint8_t mode, uint8_t pid, uint8_t A) {
  obdPort.printf("%02X %02X %02X", 0x40 + mode, pid, A);
  sendPrompt();
}

// Format and send a 2-byte PID response
void sendPID2(uint8_t mode, uint8_t pid, uint8_t A, uint8_t B) {
  obdPort.printf("%02X %02X %02X %02X", 0x40 + mode, pid, A, B);
  sendPrompt();
}

// =============================================================================
// Handle Mode 01 (current data) PID queries
// =============================================================================
void handleMode01(uint8_t pid) {
  switch (pid) {

    case 0x00: { // Supported PIDs 01-20
      // Bits indicate support for PIDs 0x01–0x20
      // We support: 05,0C,0D,0F,10,11 → bits 4,11,12,14,15,16
      obdPort.print("41 00 BE 3F A8 13");
      sendPrompt();
      break;
    }

    case 0x01: { // Monitor status since DTC cleared
      uint8_t A = (vehicle.dtcCount > 0 ? 0x80 : 0x00) | (vehicle.dtcCount & 0x7F);
      sendPID1(1, pid, A);
      break;
    }

    case 0x04: { // Calculated engine load 0–100%
      uint8_t A = (uint8_t)((vehicle.engineLoad / 100.0f) * 255.0f);
      sendPID1(1, pid, A);
      break;
    }

    case 0x05: { // Coolant temperature -40 to 215°C → A = temp + 40
      uint8_t A = (uint8_t)(vehicle.coolantTemp + 40);
      sendPID1(1, pid, A);
      break;
    }

    case 0x0A: { // Fuel pressure (gauge) kPa → A = kPa/3
      uint8_t A = (uint8_t)(vehicle.fuelPressure / 3.0f);
      sendPID1(1, pid, A);
      break;
    }

    case 0x0B: { // Intake manifold pressure kPa → A = kPa
      uint8_t A = (uint8_t)vehicle.baroPressure;
      sendPID1(1, pid, A);
      break;
    }

    case 0x0C: { // RPM → ((A*256)+B)/4
      int val = (int)(vehicle.rpm * 4);
      val = constrain(val, 0, 65535);
      sendPID2(1, pid, (val >> 8) & 0xFF, val & 0xFF);
      break;
    }

    case 0x0D: { // Vehicle speed km/h → A = speed
      uint8_t A = (uint8_t)constrain((int)vehicle.speed, 0, 255);
      sendPID1(1, pid, A);
      break;
    }

    case 0x0F: { // Intake air temp → A = temp + 40
      uint8_t A = (uint8_t)(vehicle.intakeTemp + 40);
      sendPID1(1, pid, A);
      break;
    }

    case 0x10: { // MAF g/s → ((A*256)+B)/100
      int val = (int)(vehicle.maf * 100);
      val = constrain(val, 0, 65535);
      sendPID2(1, pid, (val >> 8) & 0xFF, val & 0xFF);
      break;
    }

    case 0x11: { // Throttle position 0–100% → A = (pct/100)*255
      uint8_t A = (uint8_t)((vehicle.throttle / 100.0f) * 255.0f);
      sendPID1(1, pid, A);
      break;
    }

    case 0x1F: { // Run time since engine start (seconds)
      uint16_t secs = (uint16_t)(millis() / 1000);
      sendPID2(1, pid, (secs >> 8) & 0xFF, secs & 0xFF);
      break;
    }

    case 0x2F: { // Fuel tank level 0–100% → A = (pct/100)*255
      uint8_t A = (uint8_t)((vehicle.fuelLevel / 100.0f) * 255.0f);
      sendPID1(1, pid, A);
      break;
    }

    case 0x33: { // Barometric pressure kPa → A = kPa
      uint8_t A = (uint8_t)vehicle.baroPressure;
      sendPID1(1, pid, A);
      break;
    }

    case 0x46: { // Ambient air temperature → A = temp + 40
      uint8_t A = (uint8_t)(25 + 40); // fixed 25°C ambient
      sendPID1(1, pid, A);
      break;
    }

    default:
      sendNoData();
      break;
  }
}

// =============================================================================
// Handle Mode 03 — request DTCs
// =============================================================================
void handleMode03() {
  if (vehicle.dtcCount == 0) {
    obdPort.print("43 00");
  } else {
    // Return a fake DTC: P0301 (cylinder 1 misfire)
    // DTC encoding: P0301 → 03 01
    obdPort.print("43 01 03 01 00 00 00 00");
  }
  sendPrompt();
}

// =============================================================================
// Handle Mode 04 — clear DTCs
// =============================================================================
void handleMode04() {
  vehicle.dtcCount = 0;
  obdPort.print("44");
  sendPrompt();
}

// =============================================================================
// Handle Mode 09 — vehicle info
// =============================================================================
void handleMode09(uint8_t pid) {
  switch (pid) {
    case 0x02: { // VIN (17 chars)
      obdPort.print("49 02 01 53 49 4D 55 4C 41 54 45 44 45 43 55 30 30 31 00");
      sendPrompt();
      break;
    }
    default:
      sendNoData();
      break;
  }
}

// =============================================================================
// AT command handler
// =============================================================================
bool echoOn = true;   // ATE1 = echo on (default), ATE0 = off
bool linefeedsOn = true;
bool spacesOn = true;
char obdProtocol = '0'; // ATSP value

void handleATCommand(String cmd) {
  cmd.toUpperCase();
  cmd.trim();

  Serial.print("[SIM] AT← ");
  Serial.println(cmd);

  if (cmd == "ATZ" || cmd == "AT Z") {
    echoOn = true;
    linefeedsOn = true;
    spacesOn = true;
    delay(500); // simulate reset time
    obdPort.print("\r\nELM327 v1.5");
    sendPrompt();
    return;
  }

  if (cmd == "ATI") {
    obdPort.print("ELM327 v1.5");
    sendPrompt();
    return;
  }

  if (cmd == "ATE0") { echoOn = false; sendOK(); return; }
  if (cmd == "ATE1") { echoOn = true;  sendOK(); return; }
  if (cmd == "ATL0") { linefeedsOn = false; sendOK(); return; }
  if (cmd == "ATL1") { linefeedsOn = true;  sendOK(); return; }
  if (cmd == "ATS0") { spacesOn = false; sendOK(); return; }
  if (cmd == "ATS1") { spacesOn = true;  sendOK(); return; }
  if (cmd == "ATH0" || cmd == "ATH1") { sendOK(); return; } // headers on/off
  if (cmd == "ATAT0" || cmd == "ATAT1" || cmd == "ATAT2") { sendOK(); return; } // adaptive timing
  if (cmd == "ATST00") { sendOK(); return; } // set timeout
  if (cmd.startsWith("ATSP"))    { obdProtocol = cmd.charAt(4); sendOK(); return; }
  if (cmd.startsWith("ATSH"))    { sendOK(); return; } // accept header set
  if (cmd.startsWith("ATFC"))    { sendOK(); return; } // accept flow control cmds
  if (cmd == "ATAT2")            { sendOK(); return; } // accept adaptive timing

  if (cmd.startsWith("ATSP")) { // set protocol
    obdProtocol = cmd.charAt(4);
    sendOK();
    return;
  }

  if (cmd == "ATDP") { // describe protocol
    obdPort.print("AUTO");
    sendPrompt();
    return;
  }

  if (cmd == "ATDPN") {
    obdPort.printf("A%c", obdProtocol);
    sendPrompt();
    return;
  }

  if (cmd == "ATRV") { // read voltage
    obdPort.printf("%.1fV", vehicle.voltage);
    sendPrompt();
    return;
  }

  if (cmd == "ATLP") { // low power mode — ACK and simulate sleep
    obdPort.print("OK");
    Serial.println("[SIM] Entered low power mode (simulated)");
    sendPrompt();
    return;
  }

  if (cmd == "AT@1") { // device description
    obdPort.print("OBDII Simulator");
    sendPrompt();
    return;
  }

  // Unknown AT command
  Serial.print("[SIM] Unknown AT: ");
  Serial.println(cmd);
  sendError();
}

// =============================================================================
// OBD PID command handler
// Parses hex mode+pid from a command like "010C" or "03"
// =============================================================================
void handleOBDCommand(String cmd) {
  cmd.trim();
  if (cmd.length() < 2) { sendError(); return; }

  // Parse mode (first 2 hex chars)
  uint8_t mode = (uint8_t)strtol(cmd.substring(0, 2).c_str(), nullptr, 16);

  Serial.printf("[SIM] OBD← mode=%02X", mode);

  switch (mode) {
    case 0x01: {
      if (cmd.length() < 4) { sendError(); return; }
      uint8_t pid = (uint8_t)strtol(cmd.substring(2, 4).c_str(), nullptr, 16);
      Serial.printf(" pid=%02X\n", pid);
      handleMode01(pid);
      break;
    }
    case 0x03:
      Serial.println();
      handleMode03();
      break;
    case 0x04:
      Serial.println();
      handleMode04();
      break;
    case 0x09: {
      if (cmd.length() < 4) { sendError(); return; }
      uint8_t pid = (uint8_t)strtol(cmd.substring(2, 4).c_str(), nullptr, 16);
      Serial.printf(" pid=%02X\n", pid);
      handleMode09(pid);
      break;
    }
    default:
      Serial.println();
      sendNoData();
      break;
  }
}

// =============================================================================
// Parse and route incoming command string
// =============================================================================
void processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    sendPrompt(); // empty line → just re-prompt
    return;
  }

  if (echoOn) {
    obdPort.print(cmd);
    obdPort.print("\r\n");
  }

  // Route by prefix
  if (cmd.startsWith("AT") || cmd.startsWith("at")) {
    handleATCommand(cmd);
  } else {
    handleOBDCommand(cmd);
  }
}

// =============================================================================
// Setup
// =============================================================================
void setup() {
  Serial.begin(DEBUG_BAUD);
  Serial.println("\n[SIM] ECU Simulator starting...");

  obdPort.begin(OBD_BAUD, SERIAL_8N1, OBD_RX_PIN, OBD_TX_PIN);

  // Initial vehicle state
  vehicle.rpm          = 0;
  vehicle.speed        = 0;
  vehicle.coolantTemp  = 20;   // cold engine
  vehicle.throttle     = 0;
  vehicle.engineLoad   = 0;
  vehicle.fuelPressure = 0;
  vehicle.intakeTemp   = 25;
  vehicle.maf          = 0;
  vehicle.fuelLevel    = 75;   // 3/4 tank
  vehicle.baroPressure = 101.3;
  vehicle.voltage      = 12.6; // resting battery
  vehicle.dtcCount     = 0;

  scenarioStartMs = millis();

  Serial.println("[SIM] Ready. Waiting for commands on UART2...");
  Serial.printf("[SIM] RX=GPIO%d  TX=GPIO%d  BAUD=%d\n",
                OBD_RX_PIN, OBD_TX_PIN, OBD_BAUD);
}

// =============================================================================
// Main loop — read commands and update vehicle state
// =============================================================================
String incomingCmd = "";
unsigned long lastStateUpdate = 0;
unsigned long lastDebugPrint  = 0;

void loop() {
  // Update simulated vehicle state every 50ms
  if (millis() - lastStateUpdate > 50) {
    lastStateUpdate = millis();
    updateVehicleState();
  }

  // Print state to USB serial every 2 seconds for monitoring
  if (millis() - lastDebugPrint > 2000) {
    lastDebugPrint = millis();
    Serial.printf("[SIM] RPM=%.0f SPD=%.0f COOL=%.1f°C THR=%.0f%% LOAD=%.0f%% VOLT=%.1fV\n",
                  vehicle.rpm, vehicle.speed, vehicle.coolantTemp,
                  vehicle.throttle, vehicle.engineLoad, vehicle.voltage);
  }

  // Read incoming bytes from main ESP32
  while (obdPort.available()) {
    char c = obdPort.read();

    if (c == '\r' || c == '\n') {
      if (incomingCmd.length() > 0) {
        processCommand(incomingCmd);
        incomingCmd = "";
      }
    } else {
      incomingCmd += c;
    }
  }
}