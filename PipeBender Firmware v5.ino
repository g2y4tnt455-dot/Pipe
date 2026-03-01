// ============================================================
//  PIPE BENDER PRO — FIRMWARE v5.0  (PRODUCTION SPEC)
//  ESP32 DevKit V1 + BNO055 + 1.3" OLED (SSD1306)
//
//  ── BUTTON LAYOUT ────────────────────────────────────────
//  GPIO 0   PWR         Left end cap (recessed)
//  GPIO 4   ZERO        Front face left
//  GPIO 16  MODE        Front face center
//  GPIO 17  CUSTOM      Front face right of screen
//  GPIO 18  PRESET ▲    Front face far right
//  GPIO 19  PRESET ▼    Front face far right
//
//  ── LED LAYOUT ───────────────────────────────────────────
//  Top face — 45° channels:
//  GPIO 32  Level LED 0  far left  yellow
//  GPIO 33  Level LED 1  left      yellow
//  GPIO 34  Level LED 2  center    GREEN
//  GPIO 35  Level LED 3  right     yellow
//  GPIO 36  Level LED 4  far right yellow
//
//  Top face — center 45° channel:
//  GPIO 13  RGB Status R  (between level and preset LEDs)
//  GPIO 12  RGB Status G
//  GPIO 14  RGB Status B
//
//  Top face — standard holes:
//  GPIO 21  Preset LED 0  10°
//  GPIO 22  Preset LED 1  22.5°
//  GPIO 23  Preset LED 2  30°
//  GPIO 25  Preset LED 3  45°
//  GPIO 26  Preset LED 4  90°
//
//  Front face — above MODE button:
//  GPIO 27  Mode LED R
//  GPIO 5   Mode LED G
//  GPIO 15  Mode LED B
//
//  ── I2C (shared bus) ─────────────────────────────────────
//  GPIO 21  SDA  →  BNO055 + OLED SSD1306
//  GPIO 22  SCL  →  BNO055 + OLED SSD1306
//  BNO055  address 0x28
//  SSD1306 address 0x3C
//
//  ── REQUIRED LIBRARIES ───────────────────────────────────
//  Adafruit BNO055
//  Adafruit Unified Sensor
//  Adafruit SSD1306
//  Adafruit GFX
//  ArduinoJson v6
//  ESP32 BLE Arduino (built in)
//  Preferences (built in)
//
//  ── MODES ────────────────────────────────────────────────
//  LEVEL MODE  : torpedo level — shows tilt live
//                Screen: LEVEL | tilt° | battery%
//                Mode LED: BLUE
//
//  BEND MODE   : angle bending — zero then chase preset
//                Screen: moved° | TARGET xx° | battery%
//                Mode LED: GOLD
//
//  ── BUTTON BEHAVIOR ──────────────────────────────────────
//  MODE  1 press  = LEVEL MODE
//  MODE  2 presses (within 400ms) = BEND MODE
//  ZERO  short press = set zero reference (Bend Mode)
//  ZERO  long 2s    = clear zero reference
//  CUSTOM press     = enter custom angle mode
//         then ▲▼ to adjust in 0.5° steps, ZERO to confirm
//         OR set from app via BLE
//  PRESET ▲▼  cycle presets in Bend Mode
//
//  ── BLE ──────────────────────────────────────────────────
//  Device name:  "PipeBender Pro"
//  Service UUID: 4fafc201-1fb5-459e-8fcc-c5c9c331914b
//  Angle char:   beb5483e-36e1-4688-b7f5-ea07361b26a8  (notify)
//  Cmd char:     beb5483e-36e1-4688-b7f5-ea07361b26a9  (write)
// ============================================================

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_GFX.h>
#include <ArduinoJson.h>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <Preferences.h>

// ── PINS ─────────────────────────────────────────────────
#define PIN_BTN_PWR         0
#define PIN_BTN_ZERO        4
#define PIN_BTN_MODE        16
#define PIN_BTN_CUSTOM      17
#define PIN_BTN_PRE_UP      18
#define PIN_BTN_PRE_DN      19

#define PIN_LVL_0           32
#define PIN_LVL_1           33
#define PIN_LVL_2           34
#define PIN_LVL_3           35
#define PIN_LVL_4           36
const int LVL_PINS[5] = {PIN_LVL_0,PIN_LVL_1,PIN_LVL_2,PIN_LVL_3,PIN_LVL_4};

#define PIN_RGB_R           13
#define PIN_RGB_G           12
#define PIN_RGB_B           14

#define PIN_PRE_0           21
#define PIN_PRE_1           22
#define PIN_PRE_2           23
#define PIN_PRE_3           25
#define PIN_PRE_4           26
const int PRE_PINS[5] = {PIN_PRE_0,PIN_PRE_1,PIN_PRE_2,PIN_PRE_3,PIN_PRE_4};

#define PIN_MODE_R          27
#define PIN_MODE_G          5
#define PIN_MODE_B          15

#define PIN_BUZZER          2

// ── OLED ─────────────────────────────────────────────────
#define SCREEN_WIDTH        128
#define SCREEN_HEIGHT       64
#define OLED_ADDR           0x3C
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// ── SENSOR ───────────────────────────────────────────────
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

// ── STORAGE ──────────────────────────────────────────────
Preferences prefs;

// ── BLE ──────────────────────────────────────────────────
#define BLE_NAME            "PipeBender Pro"
#define SVC_UUID            "4fafc201-1fb5-459e-8fcc-c5c9c331914b"
#define CHR_ANGLE_UUID      "beb5483e-36e1-4688-b7f5-ea07361b26a8"
#define CHR_CMD_UUID        "beb5483e-36e1-4688-b7f5-ea07361b26a9"
BLEServer*         pServer    = nullptr;
BLECharacteristic* pAngleChr  = nullptr;
bool               bleConn    = false;

// ── STATE ─────────────────────────────────────────────────
#define MODE_LEVEL  0
#define MODE_BEND   1

int   deviceMode     = MODE_LEVEL;
int   presetIdx      = 0;
float zeroRef        = 0.0f;
bool  isZeroed       = false;
float currentAngle   = 0.0f;
float tiltAngle      = 0.0f;   // level mode: side tilt
float movedAngle     = 0.0f;   // bend mode: degrees moved from zero

// Custom angle entry
bool  customMode     = false;
float customAngle    = 45.0f;
bool  useCustom      = false;

// Screen dim timer
unsigned long lastActivity = 0;
bool screenDimmed = false;
#define DIM_TIMEOUT_MS  30000

// Battery (simulated — real impl needs ADC on GPIO 35 via voltage divider)
int battPct = 85;

const float PRESETS[5] = {10.0f, 22.5f, 30.0f, 45.0f, 90.0f};

// Bend thresholds
#define T_FAR   0.40f
#define T_NEAR  0.75f
#define T_HIT   0.98f
#define OVERBEND_DEG 10.0f  // degrees past target allowed for springback

// Button timing
#define DEBOUNCE        50
#define LONG_PRESS      2000
#define DBL_PRESS_WIN   400
#define BUZZ_MS         200

// Button state tracking
struct BtnState {
  bool     prev      = HIGH;
  unsigned long down = 0;
  bool     held      = false;
};
BtnState bZero, bMode, bCustom, bPreUp, bPreDn;
int      modePressCount = 0;
unsigned long lastModePress = 0;

bool buzzing = false;
unsigned long buzzStart = 0;

// ── BLE CALLBACKS ─────────────────────────────────────────
class SrvCB : public BLEServerCallbacks {
  void onConnect(BLEServer*)    { bleConn = true; }
  void onDisconnect(BLEServer*) { bleConn = false; BLEDevice::startAdvertising(); }
};

class CmdCB : public BLECharacteristicCallbacks {
  void onWrite(BLECharacteristic* c) {
    String v = c->getValue().c_str();
    StaticJsonDocument<128> d;
    if (deserializeJson(d, v) != DeserializationError::Ok) return;
    const char* cmd = d["cmd"];
    if (!cmd) return;

    wakeScreen();

    if (!strcmp(cmd,"zero")) {
      zeroRef = currentAngle; isZeroed = true;
      deviceMode = MODE_BEND; updateModeLED(); shortBeep();
    }
    else if (!strcmp(cmd,"mode_level")) {
      deviceMode = MODE_LEVEL; isZeroed = false;
      customMode = false; updateModeLED(); updatePresetLEDs();
    }
    else if (!strcmp(cmd,"mode_bend")) {
      deviceMode = MODE_BEND; updateModeLED();
    }
    else if (!strcmp(cmd,"preset")) {
      int i = d["index"] | 0;
      if (i>=0 && i<5) { presetIdx=i; useCustom=false; updatePresetLEDs(); }
    }
    else if (!strcmp(cmd,"custom_angle")) {
      float a = d["angle"] | 45.0f;
      a = constrain(a, 0.0f, 180.0f);
      customAngle = a; useCustom = true;
      customMode  = false;
      prefs.putFloat("customAngle", customAngle);
      prefs.putBool("useCustom", true);
      updatePresetLEDs(); shortBeep();
    }
    else if (!strcmp(cmd,"preset_up")) {
      useCustom=false; presetIdx=(presetIdx+1)%5; updatePresetLEDs(); shortBeep();
    }
    else if (!strcmp(cmd,"preset_dn")) {
      useCustom=false; presetIdx=(presetIdx-1+5)%5; updatePresetLEDs(); shortBeep();
    }
  }
};

// ── SETUP ─────────────────────────────────────────────────
void setup() {
  Serial.begin(115200);

  // Buttons
  for (int p : {PIN_BTN_PWR,PIN_BTN_ZERO,PIN_BTN_MODE,PIN_BTN_CUSTOM,PIN_BTN_PRE_UP,PIN_BTN_PRE_DN})
    pinMode(p, INPUT_PULLUP);

  // LEDs
  for (int p : {PIN_LVL_0,PIN_LVL_1,PIN_LVL_2,PIN_LVL_3,PIN_LVL_4,
                PIN_PRE_0,PIN_PRE_1,PIN_PRE_2,PIN_PRE_3,PIN_PRE_4,
                PIN_RGB_R,PIN_RGB_G,PIN_RGB_B,
                PIN_MODE_R,PIN_MODE_G,PIN_MODE_B,PIN_BUZZER})
    { pinMode(p, OUTPUT); digitalWrite(p, LOW); }

  prefs.begin("pbpro", false);
  presetIdx   = prefs.getInt("presetIdx", 0);
  customAngle = prefs.getFloat("customAngle", 45.0f);
  useCustom   = prefs.getBool("useCustom", false);

  // OLED
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed");
  } else {
    display.clearDisplay();
    display.setTextColor(SSD1306_WHITE);
    display.display();
  }

  // BNO055
  if (!bno.begin()) {
    Serial.println("BNO055 not found!");
    showOLEDError("SENSOR ERROR\nCHECK WIRING");
  }
  bno.setExtCrystalUse(true);

  setupBLE();
  startupAnimation();

  deviceMode = MODE_LEVEL;
  updateModeLED();
  updatePresetLEDs();
  lastActivity = millis();
  Serial.println("Pipe Bender Pro v5.0 ready");
}

void setupBLE() {
  BLEDevice::init(BLE_NAME);
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new SrvCB());
  BLEService* svc = pServer->createService(SVC_UUID);
  pAngleChr = svc->createCharacteristic(CHR_ANGLE_UUID,
    BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
  pAngleChr->addDescriptor(new BLE2902());
  BLECharacteristic* pCmdChr = svc->createCharacteristic(CHR_CMD_UUID,
    BLECharacteristic::PROPERTY_WRITE);
  pCmdChr->setCallbacks(new CmdCB());
  svc->start();
  BLEDevice::getAdvertising()->addServiceUUID(SVC_UUID);
  BLEDevice::getAdvertising()->setScanResponse(true);
  BLEDevice::startAdvertising();
}

// ── LOOP ──────────────────────────────────────────────────
unsigned long lastSensor = 0, lastBLE = 0, lastDisplay = 0;

void loop() {
  unsigned long now = millis();
  if (now - lastSensor  >= 20)  { lastSensor=now;  readSensor(); updateLEDs(); }
  if (now - lastBLE     >= 50)  { lastBLE=now;     broadcastBLE(); }
  if (now - lastDisplay >= 100) { lastDisplay=now; updateOLED(); }
  handleButtons();
  handleBuzzer();
  handleScreenDim();
}

// ── SENSOR ────────────────────────────────────────────────
void readSensor() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  // Roll = rotation around the long axis of the device
  currentAngle = euler.z();
  tiltAngle    = euler.z();  // for level mode
  if (deviceMode == MODE_BEND && isZeroed) {
    movedAngle = currentAngle - zeroRef;
    while (movedAngle >  180.0f) movedAngle -= 360.0f;
    while (movedAngle < -180.0f) movedAngle += 360.0f;
  }
}

float getTarget() {
  if (useCustom) return customAngle;
  return PRESETS[presetIdx];
}

float getProgress() {
  float t = getTarget();
  if (!isZeroed || t < 0.1f) return 0.0f;
  float maxProg = 1.0f + OVERBEND_DEG / max(t, 1.0f);
  return constrain(fabsf(movedAngle) / t, 0.0f, maxProg);
}

// ── LEDs ──────────────────────────────────────────────────
void updateLEDs() {
  for (int i=0;i<5;i++) digitalWrite(LVL_PINS[i], LOW);

  if (deviceMode == MODE_LEVEL) {
    // Standard torpedo level behavior
    float t = tiltAngle;
    if      (fabsf(t) < 1.0f)  digitalWrite(PIN_LVL_2, HIGH);
    else if (t < -3.0f)         digitalWrite(PIN_LVL_0, HIGH);
    else if (t < -1.0f)         digitalWrite(PIN_LVL_1, HIGH);
    else if (t >  3.0f)         digitalWrite(PIN_LVL_4, HIGH);
    else                        digitalWrite(PIN_LVL_3, HIGH);
    // RGB: green=level, red=tilted
    float at = fabsf(t);
    setRGB(at < 1.0f ? 0:255, at < 1.0f ? 255:0, 0);
  } else {
    // Bend mode: LEDs close inward as progress increases
    if (isZeroed) {
      float p = getProgress();
      if      (p > 1.0f)   { digitalWrite(PIN_LVL_2, HIGH); }  // PURPLE = overbend zone
      else if (p >= T_HIT)  { digitalWrite(PIN_LVL_2, HIGH); }  // GREEN = on target
      else if (p >= T_NEAR) { digitalWrite(PIN_LVL_1, HIGH); digitalWrite(PIN_LVL_3, HIGH); }
      else if (p >= T_FAR)  { digitalWrite(PIN_LVL_0, HIGH); digitalWrite(PIN_LVL_4, HIGH); }
      // RGB status + buzzer
      if      (p > 1.0f)   { setRGB(155,0,255); }              // Purple = overbend
      else if (p >= T_HIT)  { setRGB(0,255,0); startBuzz(); }  // Green = on target
      else if (p >= T_NEAR) { setRGB(255,80,0); }
      else                  { setRGB(255,0,0); }
    } else {
      // Waiting for zero — dim blue
      setRGB(0,0,30);
    }
  }
}

void setRGB(int r, int g, int b) {
  digitalWrite(PIN_RGB_R, r>127?HIGH:LOW);
  digitalWrite(PIN_RGB_G, g>127?HIGH:LOW);
  digitalWrite(PIN_RGB_B, b>127?HIGH:LOW);
}

void updateModeLED() {
  if (deviceMode == MODE_LEVEL) {
    // BLUE
    digitalWrite(PIN_MODE_R, LOW); digitalWrite(PIN_MODE_G, LOW); digitalWrite(PIN_MODE_B, HIGH);
  } else {
    // GOLD (red + green)
    digitalWrite(PIN_MODE_R, HIGH); digitalWrite(PIN_MODE_G, HIGH); digitalWrite(PIN_MODE_B, LOW);
  }
}

void updatePresetLEDs() {
  for (int i=0;i<5;i++)
    digitalWrite(PRE_PINS[i], (!useCustom && i==presetIdx && deviceMode==MODE_BEND) ? HIGH:LOW);
  prefs.putInt("presetIdx", presetIdx);
}

// ── OLED ──────────────────────────────────────────────────
void updateOLED() {
  if (screenDimmed) return;
  display.clearDisplay();
  display.setTextSize(1);

  if (customMode) {
    // Custom angle entry screen
    display.setCursor(0,0);
    display.println("CUSTOM ANGLE");
    display.println("USE UP/DN TO SET");
    display.println("ZERO TO CONFIRM");
    display.setTextSize(3);
    display.setCursor(10,30);
    char buf[16]; sprintf(buf,"%.1f", customAngle);
    display.print(buf); display.print((char)247); // degree symbol
  } else if (deviceMode == MODE_LEVEL) {
    // ── LEVEL MODE ──
    // Line 1: mode label
    display.setCursor(0,0);
    display.setTextSize(1);
    display.println("-- LEVEL MODE --");
    // Line 2: tilt angle big
    display.setTextSize(3);
    display.setCursor(8,16);
    char buf[12];
    sprintf(buf,"%.1f%c", fabsf(tiltAngle), (char)247);
    display.print(buf);
    // Line 3: status + battery
    display.setTextSize(1);
    display.setCursor(0,54);
    const char* lvlTxt = fabsf(tiltAngle)<1.0f ? "LEVEL" :
                         tiltAngle<0 ? "TILT LEFT" : "TILT RIGHT";
    display.print(lvlTxt);
    display.setCursor(90,54);
    char batt[8]; sprintf(batt,"%d%%",battPct);
    display.print(batt);
  } else {
    // ── BEND MODE ──
    float target = getTarget();
    float moved  = fabsf(movedAngle);
    float prog   = getProgress();

    if (!isZeroed) {
      // Not zeroed yet — show live level tilt like Level Mode
      display.setTextSize(1);
      display.setCursor(0,0);
      display.println("BEND MODE");
      display.println("Press ZERO to lock ref");
      display.setTextSize(3);
      display.setCursor(8,24);
      char buf[12]; sprintf(buf,"%.1f%c", fabsf(tiltAngle), (char)247);
      display.print(buf);
      display.setTextSize(1);
      display.setCursor(0,54);
      display.print(fabsf(tiltAngle)<1.0f?"LEVEL":tiltAngle<0?"TILT LEFT":"TILT RIGHT");
    } else {
      // Zeroed — show bend progress
      display.setTextSize(1);
      display.setCursor(0,0);
      char tline[24]; sprintf(tline,"BEND  TGT:%.1f%c", target, (char)247);
      display.print(tline);
      display.setTextSize(3);
      display.setCursor(4,14);
      char mline[12]; sprintf(mline,"%.1f%c", moved, (char)247);
      display.print(mline);
      display.setTextSize(1);
      display.setCursor(0,42);
      display.print("PROG:");
      int barW = (int)(min(1.0f,prog) * 70.0f);
      display.drawRect(30,42,70,8,SSD1306_WHITE);
      display.fillRect(30,42,barW,8,SSD1306_WHITE);
      display.setCursor(0,54);
      if (prog > 1.0f) {
        float past = fabsf(movedAngle) - target;
        char s[22]; sprintf(s,"OVER %.1f%c-springback", past, (char)247);
        display.print(s);
      } else if (prog >= T_HIT) {
        display.print("ON TARGET-KEEP BENDING");
      } else {
        float rem = max(0.0f, target-moved);
        char s[20]; sprintf(s,"NEED %.1f%c MORE", rem, (char)247);
        display.print(s);
      }
    }
    display.setCursor(110,54);
    char batt[6]; sprintf(batt,"%d%%",battPct);
    display.print(batt);
  }

  display.display();
}

void showOLEDError(const char* msg) {
  display.clearDisplay();
  display.setTextSize(1);
  display.setCursor(0,0);
  display.println(msg);
  display.display();
  delay(2000);
}

// ── SCREEN DIM ────────────────────────────────────────────
void wakeScreen() {
  lastActivity  = millis();
  screenDimmed  = false;
  display.ssd1306_command(SSD1306_DISPLAYON);
}

void handleScreenDim() {
  if (!screenDimmed && millis() - lastActivity > DIM_TIMEOUT_MS) {
    screenDimmed = true;
    display.ssd1306_command(SSD1306_DISPLAYOFF);
  }
}

// ── BUTTONS ───────────────────────────────────────────────
bool btnFell(int pin, BtnState& s) {
  bool cur = digitalRead(pin);
  bool fell = (s.prev==HIGH && cur==LOW);
  s.prev = cur; return fell;
}
bool btnRose(int pin, BtnState& s) {
  bool cur = digitalRead(pin);
  bool rose = (s.prev==LOW && cur==HIGH);
  s.prev = cur; return rose;
}

void handleButtons() {
  unsigned long now = millis();
  wakeScreen();  // any button wakes screen

  // ── MODE BUTTON ──
  bool modeCur = digitalRead(PIN_BTN_MODE);
  if (bMode.prev==HIGH && modeCur==LOW) {
    modePressCount = (now-lastModePress < DBL_PRESS_WIN) ? modePressCount+1 : 1;
    lastModePress  = now;
  }
  bMode.prev = modeCur;
  if (modePressCount>0 && now-lastModePress > DBL_PRESS_WIN) {
    customMode = false;
    if (modePressCount==1) {
      deviceMode=MODE_LEVEL; isZeroed=false;
      Serial.println("LEVEL MODE");
      shortBeep();
    } else {
      deviceMode=MODE_BEND;
      Serial.println("BEND MODE");
      shortBeep(); delay(80); shortBeep();
    }
    modePressCount=0; updateModeLED(); updatePresetLEDs();
  }

  // ── ZERO BUTTON ──
  bool zeroCur = digitalRead(PIN_BTN_ZERO);
  if (bZero.prev==HIGH && zeroCur==LOW) { bZero.down=now; bZero.held=false; }
  if (zeroCur==LOW && !bZero.held && now-bZero.down>=LONG_PRESS) {
    // Long press — clear zero
    isZeroed=false; zeroRef=0; useCustom=false; customMode=false;
    prefs.putBool("useCustom",false);
    shortBeep(); delay(120); shortBeep(); delay(120); shortBeep();
    bZero.held=true;
  }
  if (bZero.prev==LOW && zeroCur==HIGH) {
    if (!bZero.held && now-bZero.down>=DEBOUNCE) {
      if (customMode) {
        // Confirm custom angle
        useCustom=true; customMode=false;
        prefs.putFloat("customAngle",customAngle);
        prefs.putBool("useCustom",true);
        updatePresetLEDs(); shortBeep();
      } else if (deviceMode==MODE_BEND) {
        zeroRef=currentAngle; isZeroed=true;
        Serial.print("Zero set: "); Serial.println(zeroRef);
        shortBeep();
      }
    }
    bZero.prev=HIGH; bZero.down=0;
  }
  bZero.prev = zeroCur;

  // ── CUSTOM BUTTON ──
  bool custCur = digitalRead(PIN_BTN_CUSTOM);
  if (bCustom.prev==HIGH && custCur==LOW) {
    if (deviceMode==MODE_BEND) {
      customMode = !customMode;
      if (customMode) {
        // Start from current preset angle (or existing custom angle)
        if (!useCustom) customAngle = PRESETS[presetIdx];
        shortBeep();
      }
    }
  }
  bCustom.prev = custCur;

  // ── PRESET UP ──
  bool puCur = digitalRead(PIN_BTN_PRE_UP);
  if (bPreUp.prev==HIGH && puCur==LOW) {
    if (customMode) {
      customAngle = constrain(customAngle + 1.0f, 0.0f, 180.0f);
      useCustom = true;
      prefs.putFloat("customAngle", customAngle);
      prefs.putBool("useCustom", true);
      updatePresetLEDs();
    } else if (deviceMode==MODE_BEND) {
      useCustom=false; presetIdx=(presetIdx+1)%5;
      customAngle = PRESETS[presetIdx];
      updatePresetLEDs(); shortBeep();
    }
  }
  bPreUp.prev = puCur;

  // ── PRESET DOWN ──
  bool pdCur = digitalRead(PIN_BTN_PRE_DN);
  if (bPreDn.prev==HIGH && pdCur==LOW) {
    if (customMode) {
      customAngle = constrain(customAngle - 1.0f, 0.0f, 180.0f);
      useCustom = true;
      prefs.putFloat("customAngle", customAngle);
      prefs.putBool("useCustom", true);
      updatePresetLEDs();
    } else if (deviceMode==MODE_BEND) {
      useCustom=false; presetIdx=(presetIdx-1+5)%5;
      customAngle = PRESETS[presetIdx];
      updatePresetLEDs(); shortBeep();
    }
  }
  bPreDn.prev = pdCur;
}

// ── BUZZER ────────────────────────────────────────────────
void startBuzz() {
  if (buzzing) return;
  buzzing=true; buzzStart=millis();
  digitalWrite(PIN_BUZZER, HIGH);
}
void handleBuzzer() {
  if (buzzing && millis()-buzzStart>=BUZZ_MS) {
    buzzing=false; digitalWrite(PIN_BUZZER,LOW);
  }
}
void shortBeep() { digitalWrite(PIN_BUZZER,HIGH); delay(80); digitalWrite(PIN_BUZZER,LOW); }

// ── BLE BROADCAST ─────────────────────────────────────────
void broadcastBLE() {
  if (!bleConn) return;
  StaticJsonDocument<256> doc;
  doc["mode"]      = deviceMode==MODE_LEVEL ? "level":"bend";
  doc["angle"]     = round(currentAngle*10)/10.0;
  doc["tilt"]      = round(tiltAngle*10)/10.0;
  doc["moved"]     = round(movedAngle*10)/10.0;
  doc["target"]    = getTarget();
  doc["preset"]    = presetIdx;
  doc["useCustom"] = useCustom;
  doc["customAng"] = customAngle;
  doc["progress"]  = round(getProgress()*1000)/1000.0;
  doc["onTarget"]  = getProgress()>=T_HIT;
  doc["zeroed"]    = isZeroed;
  doc["batt"]      = battPct;
  doc["customMode"]= customMode;
  char buf[256]; serializeJson(doc,buf);
  pAngleChr->setValue(buf); pAngleChr->notify();
}

// ── STARTUP ANIMATION ─────────────────────────────────────
void startupAnimation() {
  // OLED splash
  display.clearDisplay();
  display.setTextSize(2);
  display.setCursor(4,8);  display.println("PIPE");
  display.setCursor(4,28); display.println("BENDER");
  display.setTextSize(1);
  display.setCursor(20,52); display.println("PRO  v5.0");
  display.display(); delay(1200);

  // LED sweep
  for (int i=0;i<5;i++) { digitalWrite(LVL_PINS[i],HIGH); delay(70); digitalWrite(LVL_PINS[i],LOW); }
  for (int i=0;i<5;i++) { digitalWrite(PRE_PINS[i],HIGH); delay(55); }
  delay(120);
  for (int i=0;i<5;i++) digitalWrite(PRE_PINS[i],LOW);

  // Mode LED flash blue
  for (int i=0;i<3;i++) { digitalWrite(PIN_MODE_B,HIGH); delay(90); digitalWrite(PIN_MODE_B,LOW); delay(70); }

  // RGB cycle
  setRGB(255,0,0); delay(100); setRGB(255,80,0); delay(100); setRGB(0,255,0); delay(100); setRGB(0,0,0);

  // 3 beeps
  shortBeep(); delay(90); shortBeep(); delay(90); shortBeep();

  display.clearDisplay(); display.display();
}
