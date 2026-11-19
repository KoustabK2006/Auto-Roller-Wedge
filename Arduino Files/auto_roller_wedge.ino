#include "ArduinoGraphics.h"
#include "Arduino_LED_Matrix.h"
#include <Servo.h>
#include <WiFiS3.h>
#include <Wire.h>
#include <U8g2lib.h>
#include "arduino_secrets.h"

const int greenLedPin = 2;
const int redLedPin   = 3;
const int touchPin    = 8;
const int servo1Pin   = 9;
const int servo2Pin   = 10;
const int trigPin     = 11;
const int echoPin     = 12;
const int buzzPin     = 13;

const int  maxAngle          = 60;
const int  slipTolerance     = 25;
const int  NO_ECHO           = 10000;
const unsigned long autoStepMs   = 1000;
const unsigned long returnStepMs = 25;
const unsigned long placeBlockMs = 3000;
const unsigned long minPressMs   = 50;

enum Phase {
  PHASE_RESETTING,
  PHASE_PLACE_BLOCK,
  PHASE_RUNNING,
  PHASE_SLIPPED
};

Phase currentPhase = PHASE_RESETTING;

int  angle          = 0;
int  lastWritten    = -1;
int  shown          = -1;
int  currentDist    = NO_ECHO;
int  initialDist    = NO_ECHO;
bool slipped        = false;
int  slipAngle      = -1;

unsigned long lastAutoStepTime    = 0;
unsigned long lastReturnStepTime  = 0;
unsigned long placeBlockStartTime = 0;
unsigned long buzzerStartTime     = 0;
bool buzzerActive                 = false;

bool lastTouch = false;
unsigned long pressStart = 0;

WiFiServer server(80);
unsigned long lastWifiCheck = 0;
const unsigned long wifiRetryInterval = 5000;
unsigned long lastIpLogTime = 0;
const unsigned long ipLogInterval = 10000;

ArduinoLEDMatrix matrix;
Servo servo1;
Servo servo2;

U8G2_SH1106_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

int oledPhase = -1;
int oledAngle = -1;

void serviceNetwork();
void startReset();
int  getDistance();
int  baselineDistance();
void showAngle();
void writeServos();
void beepOnce(int ms);
void checkTouchSensor();
void handleStateMachine();
void checkWiFiConnection();
void oledUpdate();

void showText(const char *s, int x, bool degree) {
  matrix.beginDraw();
  matrix.clear();
  matrix.stroke(0xFFFFFFFF);
  matrix.textFont(Font_4x6);
  matrix.beginText(x, 1, 0xFFFFFFFF);
  matrix.print(s);
  matrix.endText();
  if (degree) {
    matrix.point(10, 0);
    matrix.point(9, 1);
    matrix.point(11, 1);
    matrix.point(10, 2);
  }
  matrix.endDraw();
}

void showAngle() {
  if (angle == shown) return;
  char buf[6];
  snprintf(buf, sizeof(buf), "%d", angle);
  showText(buf, angle < 10 ? 4 : 1, true);
  shown = angle;
}

void scrollMessage(const char *msg) {
  matrix.beginDraw();
  matrix.stroke(0xFFFFFFFF);
  matrix.textScrollSpeed(55);
  matrix.textFont(Font_5x7);
  matrix.beginText(0, 1, 0xFFFFFFFF);
  matrix.println(msg);
  matrix.endText(SCROLL_LEFT);
  matrix.endDraw();
}

void formatMu(char *buf, size_t n, int deg) {
  long v = (long)(tan(deg * DEG_TO_RAD) * 1000.0 + 0.5);
  snprintf(buf, n, "%ld.%03ld", v / 1000, v % 1000);
}

void drawCentered(const char *s, int y) {
  u8g2.drawStr((128 - u8g2.getStrWidth(s)) / 2, y, s);
}

void drawTheta(int x, int y) {
  u8g2.drawEllipse(x + 2, y + 3, 2, 3);
  u8g2.drawHLine(x + 1, y + 3, 3);
}

void oledDraw() {
  char mu[12];
  char line[24];

  u8g2.clearBuffer();

  switch (currentPhase) {
    case PHASE_RESETTING:
      u8g2.setFont(u8g2_font_helvB12_tr);
      drawCentered("Resetting...", 30);
      u8g2.setFont(u8g2_font_6x10_tr);
      drawCentered("Wedge returning to level", 48);
      break;

    case PHASE_PLACE_BLOCK:
      u8g2.setFont(u8g2_font_helvB12_tr);
      drawCentered("Place block", 30);
      u8g2.setFont(u8g2_font_6x10_tr);
      drawCentered("on the wedge", 48);
      break;

    case PHASE_RUNNING:
    case PHASE_SLIPPED: {
      bool done = (currentPhase == PHASE_SLIPPED);
      int  deg  = (done && slipAngle >= 0) ? slipAngle : angle;
      formatMu(mu, sizeof(mu), deg);

      u8g2.setFont(u8g2_font_6x10_tr);
      {
        int wl = u8g2.getStrWidth("u = tan(");
        int wr = u8g2.getStrWidth(")");
        int x0 = (128 - (wl + 6 + wr)) / 2;
        u8g2.drawStr(x0, 9, "u = tan(");
        drawTheta(x0 + wl, 2);
        u8g2.drawStr(x0 + wl + 6, 9, ")");
      }

      u8g2.setFont(u8g2_font_logisoso28_tn);
      drawCentered(mu, 46);

      u8g2.setFont(u8g2_font_6x10_tr);
      drawTheta(0, 54);
      snprintf(line, sizeof(line), "= %d", deg);
      u8g2.drawStr(8, 61, line);
      u8g2.drawCircle(8 + u8g2.getStrWidth(line) + 3, 54, 2);

      const char *status = done ? "SLIPPED" : "RUNNING";
      int sw = u8g2.getStrWidth(status);
      if (done) {
        u8g2.drawBox(128 - sw - 4, 51, sw + 4, 13);
        u8g2.setDrawColor(0);
        u8g2.drawStr(128 - sw - 2, 61, status);
        u8g2.setDrawColor(1);
      } else {
        u8g2.drawStr(128 - sw, 61, status);
      }
      break;
    }
  }

  u8g2.sendBuffer();
}

void oledUpdate() {
  int ph = (int)currentPhase;
  bool changed = (ph != oledPhase) ||
                 (currentPhase == PHASE_RUNNING && angle != oledAngle);
  if (!changed) return;

  oledDraw();
  oledPhase = ph;
  oledAngle = angle;
}

void writeServos() {
  if (angle == lastWritten) return;
  servo1.write(90 + angle);
  servo2.write(90 - angle);
  lastWritten = angle;
}

void beepOnce(int ms) {
  digitalWrite(buzzPin, HIGH);
  unsigned long start = millis();
  while (millis() - start < (unsigned long)ms) {
    serviceNetwork();
    delayMicroseconds(250);
  }
  digitalWrite(buzzPin, LOW);
}

int getDistance() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH, 30000UL);
  if (duration == 0) return NO_ECHO;
  return (int)(duration * 0.1715);
}

int baselineDistance() {
  long sum = 0;
  int n = 0;
  for (int i = 0; i < 8; i++) {
    int d = getDistance();
    if (d != NO_ECHO) {
      sum += d;
      n++;
    }
    unsigned long waitStart = millis();
    while (millis() - waitStart < 50) {
      serviceNetwork();
      delayMicroseconds(500);
    }
  }
  return n ? (int)(sum / n) : NO_ECHO;
}

void startReset() {
  currentPhase = PHASE_RESETTING;
  slipped = false;
  slipAngle = -1;
  buzzerActive = false;
  digitalWrite(buzzPin, LOW);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(redLedPin, LOW);
  lastReturnStepTime = millis();
  shown = -1;
  Serial.println(F("[BENCH] Reset triggered. Returning to 0 deg..."));
}

void handleStateMachine() {
  unsigned long now = millis();

  if (buzzerActive && (now - buzzerStartTime >= 500)) {
    digitalWrite(buzzPin, LOW);
    buzzerActive = false;
  }

  switch (currentPhase) {
    case PHASE_RESETTING: {
      if (angle > 0) {
        if (now - lastReturnStepTime >= returnStepMs) {
          angle--;
          writeServos();
          showAngle();
          lastReturnStepTime = now;
        }
      } else {
        writeServos();
        showAngle();
        unsigned long settleStart = millis();
        while (millis() - settleStart < 200) {
          serviceNetwork();
          delayMicroseconds(500);
        }
        beepOnce(100);
        currentPhase = PHASE_PLACE_BLOCK;
        placeBlockStartTime = millis();
        Serial.println(F("[BENCH] 0 deg reached. Phase -> PLACE_BLOCK (3s)"));
      }
      break;
    }

    case PHASE_PLACE_BLOCK: {
      showAngle();
      if (now - placeBlockStartTime >= placeBlockMs) {
        Serial.println(F("[BENCH] Taking acoustic baseline distance..."));
        initialDist = baselineDistance();
        currentDist = initialDist;
        Serial.print(F("[BENCH] Baseline established: "));
        if (initialDist == NO_ECHO) {
          Serial.println(F("NO_ECHO"));
        } else {
          Serial.print(initialDist);
          Serial.println(F(" mm"));
        }
        currentPhase = PHASE_RUNNING;
        digitalWrite(greenLedPin, HIGH);
        digitalWrite(redLedPin, LOW);
        lastAutoStepTime = millis();
        Serial.println(F("[BENCH] Phase -> RUNNING (Green LED D2 ON)"));
      }
      break;
    }

    case PHASE_RUNNING: {
      digitalWrite(greenLedPin, HIGH);
      digitalWrite(redLedPin, LOW);

      currentDist = getDistance();

      if (currentDist != NO_ECHO && initialDist != NO_ECHO) {
        if (abs(currentDist - initialDist) >= slipTolerance) {
          slipped = true;
          slipAngle = angle;
          currentPhase = PHASE_SLIPPED;
          digitalWrite(greenLedPin, LOW);
          digitalWrite(redLedPin, HIGH);
          buzzerActive = true;
          buzzerStartTime = millis();
          digitalWrite(buzzPin, HIGH);

          Serial.print(F("[BENCH] *** SLIP DETECTED at "));
          Serial.print(slipAngle);
          Serial.print(F(" deg! Delta: "));
          Serial.print(abs(currentDist - initialDist));
          Serial.println(F(" mm ***"));
          return;
        }
      }

      if (angle < maxAngle && (now - lastAutoStepTime >= autoStepMs)) {
        angle++;
        lastAutoStepTime = now;
        writeServos();
        showAngle();
      }
      break;
    }

    case PHASE_SLIPPED: {
      digitalWrite(greenLedPin, LOW);
      digitalWrite(redLedPin, HIGH);
      currentDist = getDistance();
      showAngle();
      break;
    }
  }
}

void checkTouchSensor() {
  bool touch = (digitalRead(touchPin) == HIGH);

  if (touch && !lastTouch) {
    pressStart = millis();
  }

  if (!touch && lastTouch) {
    if (millis() - pressStart >= minPressMs) {
      if (currentPhase != PHASE_RESETTING && currentPhase != PHASE_PLACE_BLOCK) {
        Serial.println(F("[BENCH] Touch sensor pressed -> Resetting"));
        startReset();
      } else {
        Serial.println(F("[BENCH] Touch sensor ignored (reset already in progress)"));
      }
    }
  }

  lastTouch = touch;
}

void serviceNetwork() {
  WiFiClient client = server.available();
  if (!client) return;

  String reqLine = "";
  unsigned long timeout = millis() + 500;
  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      char c = client.read();
      if (c == '\n') break;
      if (c != '\r') reqLine += c;
    }
  }

  int firstSpace = reqLine.indexOf(' ');
  int secondSpace = reqLine.indexOf(' ', firstSpace + 1);
  String method = "";
  String path = "";
  if (firstSpace > 0) {
    method = reqLine.substring(0, firstSpace);
    if (secondSpace > firstSpace) {
      path = reqLine.substring(firstSpace + 1, secondSpace);
    } else {
      path = reqLine.substring(firstSpace + 1);
    }
  }

  while (client.connected() && millis() < timeout) {
    if (client.available()) {
      String line = client.readStringUntil('\n');
      if (line == "\r" || line.length() == 0) break;
    }
  }

  if (method == "OPTIONS") {
    client.println("HTTP/1.1 204 No Content");
    client.println("Access-Control-Allow-Origin: *");
    client.println("Access-Control-Allow-Methods: GET, POST, OPTIONS");
    client.println("Access-Control-Allow-Headers: Content-Type");
    client.println("Connection: close");
    client.println();
    client.stop();
    return;
  }

  #define SEND_CORS_HEADERS(statusLine) \
    client.println(statusLine); \
    client.println("Access-Control-Allow-Origin: *"); \
    client.println("Access-Control-Allow-Methods: GET, POST, OPTIONS"); \
    client.println("Access-Control-Allow-Headers: Content-Type"); \
    client.println("Content-Type: application/json"); \
    client.println("Connection: close"); \
    client.println();

  if (method == "GET" && (path == "/status" || path == "/status/")) {
    SEND_CORS_HEADERS("HTTP/1.1 200 OK");

    const char* phaseStr = "RUNNING";
    switch (currentPhase) {
      case PHASE_RESETTING:   phaseStr = "RESETTING";   break;
      case PHASE_PLACE_BLOCK: phaseStr = "PLACE_BLOCK"; break;
      case PHASE_RUNNING:     phaseStr = "RUNNING";     break;
      case PHASE_SLIPPED:     phaseStr = "SLIPPED";     break;
    }

    client.print("{\"angle\":");
    client.print(angle);

    client.print(",\"distance\":");
    if (currentDist == NO_ECHO) client.print("null");
    else client.print(currentDist);

    client.print(",\"initialDist\":");
    if (initialDist == NO_ECHO) client.print("null");
    else client.print(initialDist);

    client.print(",\"slipped\":");
    client.print(slipped ? "true" : "false");

    client.print(",\"slipAngle\":");
    if (slipAngle < 0) client.print("null");
    else client.print(slipAngle);

    client.print(",\"phase\":\"");
    client.print(phaseStr);
    client.print("\"");

    client.print(",\"maxAngle\":");
    client.print(maxAngle);

    client.print(",\"slipTolerance\":");
    client.print(slipTolerance);

    client.print(",\"uptimeMs\":");
    client.print(millis());

    client.println("}");
    client.stop();
    return;
  }

  if ((method == "POST" || method == "GET") && (path == "/reset" || path == "/reset/")) {
    if (currentPhase == PHASE_RESETTING || currentPhase == PHASE_PLACE_BLOCK) {
      SEND_CORS_HEADERS("HTTP/1.1 200 OK");
      client.println("{\"ok\":false,\"reason\":\"busy\"}");
    } else {
      startReset();
      SEND_CORS_HEADERS("HTTP/1.1 200 OK");
      client.println("{\"ok\":true}");
    }
    client.stop();
    return;
  }

  SEND_CORS_HEADERS("HTTP/1.1 404 Not Found");
  client.println("{\"error\":\"not found\"}");
  client.stop();
}

void checkWiFiConnection() {
  if (WiFi.status() != WL_CONNECTED) {
    unsigned long now = millis();
    if (now - lastWifiCheck >= wifiRetryInterval) {
      lastWifiCheck = now;
      Serial.println(F("[WIFI] Link down. Attempting background reconnect..."));
      WiFi.begin(SECRET_SSID, SECRET_PASS);
    }
  }
}

void setup() {
  Serial.begin(115200);
  delay(100);
  Serial.println(F("\n============================================="));
  Serial.println(F("  Auto-Roller Wedge Rig - UNO R4 WiFi Online  "));
  Serial.println(F("============================================="));

  pinMode(greenLedPin, OUTPUT);
  pinMode(redLedPin, OUTPUT);
  digitalWrite(greenLedPin, LOW);
  digitalWrite(redLedPin, LOW);
  pinMode(touchPin, INPUT);
  pinMode(buzzPin, OUTPUT);
  digitalWrite(buzzPin, LOW);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  matrix.begin();

  u8g2.begin();
  u8g2.setBusClock(400000);
  u8g2.clearBuffer();
  u8g2.setFont(u8g2_font_7x13B_tr);
  drawCentered("Auto Roller Wedge", 28);
  u8g2.setFont(u8g2_font_6x10_tr);
  drawCentered("Starting...", 46);
  u8g2.sendBuffer();

  servo1.attach(servo1Pin);
  servo2.attach(servo2Pin);

  if (strcmp(SECRET_SSID, "your-wifi-ssid") == 0) {
    Serial.println(F("\n[!] ========================================================="));
    Serial.println(F("[!] WARNING: Default placeholder credentials detected!"));
    Serial.println(F("[!] Copy arduino_secrets.h.example to arduino_secrets.h"));
    Serial.println(F("[!] and set your 2.4 GHz WiFi SSID and Password, then re-upload."));
    Serial.println(F("[!] =========================================================\n"));
    scrollMessage(" SET SSID IN SECRETS.H ");
  }

  Serial.print(F("[WIFI] Connecting to SSID: "));
  Serial.println(SECRET_SSID);

  int wifiAttempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifiAttempts < 20) {
    WiFi.begin(SECRET_SSID, SECRET_PASS);
    delay(500);
    Serial.print(".");
    wifiAttempts++;
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    IPAddress ip = WiFi.localIP();
    Serial.println(F("\n**************************************************"));
    Serial.print(F(">>> ARDUINO IP ADDRESS: "));
    Serial.println(ip);
    Serial.print(F(">>> Web Dashboard Target: http://"));
    Serial.println(ip);
    Serial.println(F("**************************************************\n"));

    server.begin();
    Serial.println(F("[HTTP] Server listening on port 80"));

    String ipStr = " IP: " + String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]) + " ";
    scrollMessage(ipStr.c_str());
  } else {
    Serial.println(F("[WIFI] Connection timed out. Running in standalone mode."));
    Serial.println(F("[WIFI] Tip: Ensure your WiFi is 2.4 GHz and credentials in arduino_secrets.h match."));
  }

  startReset();
}

void loop() {
  serviceNetwork();
  checkTouchSensor();
  handleStateMachine();
  oledUpdate();
  checkWiFiConnection();

  if (Serial.available()) {
    while (Serial.available()) Serial.read();
    if (WiFi.status() == WL_CONNECTED) {
      IPAddress ip = WiFi.localIP();
      Serial.println(F("\n-----------------------------------------"));
      Serial.print(F(">>> ARDUINO IP ADDRESS: "));
      Serial.println(ip);
      Serial.print(F(">>> Enter this in Web Settings: http://"));
      Serial.println(ip);
      Serial.println(F("-----------------------------------------"));
      String ipStr = " IP: " + String(ip[0]) + "." + String(ip[1]) + "." + String(ip[2]) + "." + String(ip[3]) + " ";
      scrollMessage(ipStr.c_str());
    } else {
      Serial.print(F("[WIFI] Offline. Configured SSID: "));
      Serial.println(SECRET_SSID);
    }
  }

  if (WiFi.status() == WL_CONNECTED && millis() - lastIpLogTime >= ipLogInterval) {
    lastIpLogTime = millis();
    Serial.print(F("[LIVE LINK] Arduino IP: http://"));
    Serial.print(WiFi.localIP());
    Serial.print(F(" | Angle: "));
    Serial.print(angle);
    Serial.print(F(" deg | Phase: "));
    switch (currentPhase) {
      case PHASE_RESETTING:   Serial.println(F("RESETTING"));   break;
      case PHASE_PLACE_BLOCK: Serial.println(F("PLACE_BLOCK")); break;
      case PHASE_RUNNING:     Serial.println(F("RUNNING"));     break;
      case PHASE_SLIPPED:     Serial.println(F("SLIPPED"));     break;
    }
  }

  delay(20);
}
