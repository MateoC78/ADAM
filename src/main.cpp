#include <Arduino.h>
#include <OBDHandler.h>
#include <HardwareSerial.h>
#include <LiquidCrystal_I2C.h>
#include <DFRobotDFPlayerMini.h>

// UART2 pins from ESP32 -> DFPlayer
#define MP3rx 16
#define MP3tx 17

// UART1 pins from ESP32 -> OBD
#define OBDrx 26
#define OBDtx 27

// Alerts
#define RPM_JUMP 700
#define RPM_WINDOW_MS 500
#define ALERT_COOLDOWN_MS 1500

#define OBD_POLL_GAP_MS 50


LiquidCrystal_I2C lcd(0x27,16,2);

HardwareSerial MP3Serial(2);
HardwareSerial obdSerial(1);

DFRobotDFPlayerMini mp3;
OBDHandler obd(obdSerial);

// Shared vehicle data
struct vehicleData {
  int rpm;
  int speed;
  int temp;
  int load;
  int throttle;
  float mpg;
  bool connected;
  uint32_t seq;     // counter for new data, increments on publish
};

static vehicleData v = {-1,-1,-1,-1,-1,-1.0f,false,0};
static SemaphoreHandle_t vMutex;

static void publish(vehicleData d) {
  if (xSemaphoreTake(vMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
    d.seq = v.seq + 1;
    v = d;
    xSemaphoreGive(vMutex);
  }
}

static bool snapshot(vehicleData &out) {
  if (xSemaphoreTake(vMutex, pdMS_TO_TICKS(20)) != pdTRUE) return false;
  out = v;
  xSemaphoreGive(vMutex);
  return true;
}

// periodic obdII data handling task, passes to queue
struct rawSample {
  int rpm, speed, temp, load, throttle;
  float maf;
  uint32_t t;            // millis() when sampled
};

QueueHandle_t sampleQ;

void dataTask(void*) {
  TickType_t last = xTaskGetTickCount();
  while(1) {
    rawSample s;
    s.rpm = obd.getRPM();
    s.speed = obd.getSpeedMPH();
    s.temp = obd.getCoolantTempF();
    s.load = obd.getEngLoad();
    s.throttle = obd.getThrottle();
    s.maf = obd.getMAF();
    s.t = millis();

    xQueueSend(sampleQ, &s, 0);
    vTaskDelayUntil(&last, pdMS_TO_TICKS(200));
  }
}

void obdTask(void*) {
  rawSample s;
  while(1) {
    if (xQueueReceive(sampleQ, &s, portMAX_DELAY) == pdTRUE) {
      float mpg = (s.speed > 0 && s.maf > 0.1f)
                  ? (s.speed * 7718.0f) / (s.maf * 100.0f) : -1;
      // process and publish data
    }
  }
} 

void alertTask(void*) {
  uint32_t lastSeq = 0;
  int refRPM = -1;
  uint32_t refTime = 0;
  uint32_t lastAlert = 0;

  while(1) {
    vehicleData d;
    if (snapshot(d) && d.seq != lastSeq) {
      lastSeq = d.seq;
      uint32_t now = millis();

      if (!d.connected || d.rpm < 0) {
        refRPM = -1;
      } else if (now - refTime >= RPM_WINDOW_MS) {
        if (refRPM >= 0 && abs(d.rpm - refRPM) > RPM_JUMP && now - lastAlert > ALERT_COOLDOWN_MS) {
          mp3.play(2);
          lastAlert = now;
        }
        refRPM = d.rpm;
        refTime = now;
      }
    }
    vTaskDelay(pdMS_TO_TICKS(20));
  }
}

static void num(char *out, int v, int width) {
  if (v < 0) snprintf(out, 8, "%*s", width, "--");
  else       snprintf(out, 8, "%*d", width, v);
}

void uiTask(void*) {
  // LCD line buffers
  char l0[17];
  char l1[17];
  // Engine parameter value buffers
  char l[8], r[8], m[8], t[8];

  while(1) {
    vehicleData d;
    if (snapshot(d)) {
      if (!d.connected) {
        snprintf(l0, sizeof l0, "%-16s", "OBD connecting..");
        snprintf(l1, sizeof l1, "%-16s", "Ignition ON?");  
      } else {
        num(l, d.load, 3);
        num(r, d.rpm, 4);
        num(m, d.mpg >= 0 ? (int)d.mpg : -1, 2);
        num(t, d.throttle, 3);
        // Load into buffers
        snprintf(l0, sizeof l0, "Load:%s%% R:%s", l, r);
        snprintf(l1, sizeof l1, "MPG:%s  Thr:%s%%", m, t);
      }

      // Print buffers
      lcd.setCursor(0, 0);
      lcd.print(l0);
      lcd.setCursor(0, 1);
      lcd.print(l1);
    }
    vTaskDelay(pdMS_TO_TICKS(250));
  }
}

void setup() {

  Serial.begin(115200);

  //lcd SDA-pin D21, SCL-pin D22
  lcd.init();
  lcd.clear();
  lcd.backlight();
  lcd.setCursor(2, 0);
  lcd.print("Welcome!");

  MP3Serial.begin(9600, SERIAL_8N1, MP3rx, MP3tx);
  delay(1000);
  if (mp3.begin(MP3Serial)) {
    mp3.volume(20);
    mp3.play(1);

    lcd.setCursor(0, 1);
    lcd.print("MP3 connected");
  } else {
    lcd.setCursor(0, 1);
    lcd.print("MP3 Failed");
  }
  
  delay(1000);
  lcd.clear();

  obd.begin(9600, OBDrx, OBDtx);
  delay(2000);

  if (!obd.init()) {
    lcd.setCursor(0,1);
    lcd.print("OBD Failed");
  } else {
    lcd.setCursor(0,1);
    lcd.print("OBD connected");
  }

  vMutex = xSemaphoreCreateMutex();
  sampleQ = xQueueCreate(4, sizeof(rawSample));

  xTaskCreatePinnedToCore(dataTask,   "data",   6144, nullptr, 2, nullptr, 1);
  xTaskCreatePinnedToCore(obdTask, "proc", 4096, nullptr, 2, nullptr, 1);
  //xTaskCreatePinnedToCore(alertTask, "alert", 3072, nullptr, 2, nullptr, 1);
  //xTaskCreatePinnedToCore(uiTask,    "ui",    4096, nullptr, 1, nullptr, 1);
}

void loop() {
  while(1);
}