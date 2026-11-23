#include <HardwareSerial.h>

HardwareSerial obdPort(2);

void setup() {
  Serial.begin(115200);
  obdPort.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("Echo test ready");
  obdPort.println("Echo test ready");
}

void loop() {
  // Echo anything received on UART2 back to both ports
  while (obdPort.available()) {
    char c = obdPort.read();
    obdPort.print(c);        // echo back to main ESP32
    Serial.print(c);         // also show on simulator's USB monitor

  }

  // Also forward USB input to UART2 — lets you test manually
  while (Serial.available()) {
    char c = Serial.read();
    obdPort.print(c);
    Serial.print("[sent: 0x");
    Serial.print(c, HEX);
    Serial.println("]");
  }
}