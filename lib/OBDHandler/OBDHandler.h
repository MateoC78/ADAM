#pragma once
#include <HardwareSerial.h>

class OBDHandler {
public:
  HardwareSerial& serial;
  
  OBDHandler(HardwareSerial& s) : serial(s) {}

  void begin(int baud, int rxPin, int txPin);
  bool init();

  //debugging helpers
  String sendQ(char* q);
  String sendC(char* cmd, int timeout = 500);

  void sleep();
  void wake();
  // Returns -1 on failure
  int getRPM();
  int getSpeedMPH();
  int getCoolantTempF();
  int getThrottle();
  int getEngLoad();
  float getMAF();
  int getFuelLevel();
  float getInstantMPG();

private:
  // Send AT command, return true if "OK" or ELM prompt received
  bool sendCommand(const char* cmd, int timeout = 500);

  // Send PID query, return the data line only
  String query(const char* pid);
  String readResponse(int timeout);
  int celsiusToF(int c);
};