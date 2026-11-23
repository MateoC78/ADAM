#include <OBDHandler.h>
#include <HardwareSerial.h>
#include <Arduino.h>


void OBDHandler::begin(int baud, int rxPin, int txPin) {
  serial.begin(baud, SERIAL_8N1, rxPin, txPin);
}

bool OBDHandler::init() {
  bool ready = false;

  if (!sendCommand("ATZ", 2000))    return false;
  delay(500);
  if (!sendCommand("ATE0", 1000))    return false;
  if (!sendCommand("ATL0", 1000))    return false;
  //if (!sendCommand("ATS0", 500))    return false; // spaces off

  //headers off
  if (!sendCommand("ATH0", 1000))    return false;

  //set ISO 15765 CAN protocol
  if (!sendCommand("ATSP6", 2000))   return false;

  //functional broadcast header
  if (!sendCommand("ATSH7DF", 1000))    return false;

  //CAN flow control
  if (!sendCommand("ATFCSD300000", 500))    return false;
  if (!sendCommand("ATFCSM1", 500))    return false;

  //Adaptive timing
  if (!sendCommand("ATAT2", 500))    return false;

  bool busAlive = false;
  for (int attempt = 0; attempt < 3; attempt++) {
    String r = sendC("0100", 3000);
    Serial.print("0100 attempt "); Serial.print(attempt + 1);
    Serial.print(": "); Serial.println(r);

    if (r.indexOf("41 00") >= 0) {
      busAlive = true;
      break;
    }
    delay(500);
  }

  if (!busAlive) {
    Serial.println("Bus not responding");
    return false;
  }

  Serial.println("OBD init complete");
  ready = true;

  return true;
}

//debugging helpers
String OBDHandler::sendQ(char* q){
  return query(q);
}

String OBDHandler::sendC(char* cmd, int timeout){
  serial.print(cmd);
  serial.print("\r");
  return readResponse(timeout);
}

void OBDHandler::sleep() {
  serial.println("ATLP\r");
}

void OBDHandler::wake() {
  serial.write('\r');
  delay(500);
  init();
}

// Returns -1 on failure
int OBDHandler::getRPM() {
  String r = query("010C");
  if (r.length() == 0) return -1;
  int a, b;
  if (sscanf(r.c_str(), "41 0C %02X %02X", &a, &b) == 2)
    return ((a * 256) + b) / 4;
  return -1;
}

int OBDHandler::getSpeedMPH() {
  String r = query("010D");
  if (r.length() == 0) return -1;
  int a;
  if (sscanf(r.c_str(), "41 0D %02X", &a) == 1)
    return (int)(a * 0.621371f);
  return -1;
}

int OBDHandler::getCoolantTempF() {
  String r = query("0105");
  if (r.length() == 0) return -1;
  int a;
  if (sscanf(r.c_str(), "41 05 %02X", &a) == 1)
    return celsiusToF(a-40);
  return -1;
}

int OBDHandler::getThrottle() {
  String r = query("0111");
  if (r.length() == 0) return -1;
  int a;
  if (sscanf(r.c_str(), "41 11 %02X", &a) == 1)
    return (a * 100) / 255;
  return -1;
}

int OBDHandler::getEngLoad() {
  String r = query("0104");
  if (r.length() == 0) return -1;
  int a;
  if (sscanf(r.c_str(), "41 04 %02X", &a) == 1)
    return (int)((a / 255.0f) * 100.0f);
  return -1;
}

float OBDHandler::getMAF() {
  String r = query("0110");
  if (r.length() == 0) return -1;
  int a, b;
  if (sscanf(r.c_str(), "41 10 %02X %02X", &a, &b) == 2)
    return ((a * 256.0f) + b) / 100.0f;
  return -1;
}

int OBDHandler::getFuelLevel() {
  String r = query("012F");
  if (r.length() == 0) return -1;
  int a;
  if (sscanf(r.c_str(), "41 2F %02X", &a) == 1)
    return (int)((a / 255.0f) * 100.0f);
  return -1;
}

float OBDHandler::getInstantMPG() {
  float maf   = getMAF();
  int   speed = getSpeedMPH();

  if (maf <= 0.1f || speed <= 0)  return -1; // idle/stopped
  if (maf < 0)                    return -1;

  // MPG = (speed_mph × 7718) / (MAF_g_per_sec × 100)
  // 7718 constant = (air/fuel ratio 14.7) × (fuel density 6.17 lb/gal) × (g/lb 453.6) / 3600
  float mpg = (speed * 7718.0f) / (maf * 100.0f);
  return constrain(mpg, 0.0f, 99.9f);
}

// Send AT command, return true if "OK" or ELM prompt received
bool OBDHandler::sendCommand(const char* cmd, int timeout) {
  serial.print(cmd);
  serial.print("\r");
  String resp = readResponse(timeout);
  return (resp.indexOf("OK") >= 0 || resp.indexOf("ELM") >= 0);
}

// Send PID query, return the data line only
String OBDHandler::query(const char* pid) {
  serial.print(pid);
  serial.print("\r");
  String resp = readResponse(300);
  // Strip echo and prompt, find the 41 XX response line
  int idx = resp.indexOf("41");
  if (idx < 0) return "";
  int end = resp.indexOf('\r', idx);
  if (end < 0) end = resp.length();
  return resp.substring(idx, end);
}

String OBDHandler::readResponse(int timeout) {
  String result = "";
  unsigned long t = millis();
  while (millis() - t < timeout) {
    while (serial.available()) {
      char c = serial.read();
      result += c;
      if (c == '>') return result; // ELM327 prompt = done
    }
  }
  return result;
}

int OBDHandler::celsiusToF(int c) {
  return (int)((c * 9.0f / 5.0f) + 32.0f);
}