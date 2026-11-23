#include <Arduino.h>
#include <OBDHandler.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <DFRobotDFPlayerMini.h>

//UART2 pins from ESP32 -> DFPlayer
#define MP3rx 16
#define MP3tx 17

//UART1 pins from ESP32 -> OBD
#define OBDrx 26
#define OBDtx 27

LiquidCrystal_I2C lcd(0x27,16,2);

HardwareSerial MP3Serial(2);
HardwareSerial obdSerial(1);

DFRobotDFPlayerMini mp3;
OBDHandler obd(obdSerial);

bool obdReady = false;
int lastRPM = 1000;

void setup() {

  Serial.begin(115200);

  //lcd SDA-pin D21, SCL-pin D22
  lcd.init();
  lcd.clear();
  lcd.backlight();

  lcd.setCursor(2,0);
  lcd.print("Welcome!");

  MP3Serial.begin(9600, SERIAL_8N1, MP3rx, MP3tx);
  delay(1000);
  if (mp3.begin(MP3Serial)) {
    mp3.volume(20);
    mp3.play(1);

    lcd.setCursor(0,0);
    lcd.print("MP3 connected");
  }
  
  obd.begin(9600, OBDrx, OBDtx);

  delay(2000);

  if (!obd.init()) {
    lcd.setCursor(0,1);
    lcd.print("OBD Failed");
    obdReady = false;
  } else {
    lcd.setCursor(0,1);
    lcd.print("OBD connected");
    obdReady = true;
  }

}

void loop() {
  while (!obdReady) {
    obdReady = obd.init();
    delay(1000);
    if (obdReady){
      lcd.setCursor(0,1);
      lcd.print("OBD connectedd");
      break;
    }
  }

  int T  = obd.getThrottle();
  int rpm = obd.getRPM();
  int speed = obd.getSpeedMPH();
  int temp  = obd.getCoolantTempF();
  int L = obd.getEngLoad();
  int mpg = obd.getInstantMPG();

  lcd.clear();
  lcd.setCursor(9,1);
  lcd.print("Thr:"); 
  lcd.print(T);
  lcd.print("% ");

  lcd.setCursor(0,0);
  lcd.print("Load:"); 
  lcd.print(L);
  lcd.print("% ");

  lcd.setCursor(0, 1);
  lcd.print("MPG:"); 
  lcd.print(mpg);
  lcd.print(" ");

  if (abs(rpm-lastRPM) > 700){
    mp3.play(2);
  }
  lastRPM = rpm;
  delay(500);
}