// Nano 33 IoT on the planchette: serves the HTTP API the backend calls, shows characters on the round display, and
// relays the answer to the Uno on the Zumo as $poi(x,y) points to drive to.
//
//   POST /heyOuija   show "?" for a while
//   POST /answer     form field answer=<a-z0-9>|yes|no  -> 200 accepted, 400 bad answer, 409 still spelling the last one
//   POST /tryAgain   show a red "?" for 2 s             -> 200, or 409 while spelling
//   GET  /status     busy / queue / link counters, for debugging
//
// The Uno answers with $arrived(x,y) for every point it reaches and $smiley() after the last one. When the last point
// has been reached the bot counts as idle, and (if BACKEND_HOST is defined in secrets.h) the Nano tells the backend with
// POST /bot/idle so the next question is taken straight away.
#include <SPI.h>
#include <WiFiNINA.h>
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include "OuijaFont.h"
#include "secrets.h"  // SECRET_SSID, SECRET_PASS, optionally BACKEND_HOST and BACKEND_PORT (see secrets.h.example)

#define LCD_RES_PIN 7
#define LCD_DC_PIN 8
#define LCD_CS_PIN 9
// Note: SCL goes to SCK (D13), SDA goes to MOSI (D11)

#define RGB_TAN 0xDE50

#define SERIAL1_BAUDRATE 9600
#define POI_SPACING_MS 500        // gap between points sent to the Uno, so its serial buffer is never overrun
#define POI_ACK_TIMEOUT_MS 60000UL  // send the next point anyway if the Uno has not reached the last one by then
#define MAX_POINTS 24             // longest accepted answer
#define BUSY_TIMEOUT_MS 240000UL  // give up waiting for the Uno after this long, so a lost message cannot wedge the bot
#define HEY_OUIJA_SHOW_MS 10000
#define TRY_AGAIN_SHOW_MS 2000
#define CHARACTER_SHOW_MS 3000
#define REQUEST_TIMEOUT_MS 1500
#define BODY_MAX 160

#ifndef BACKEND_PORT
#define BACKEND_PORT 8080
#endif

Adafruit_GC9A01A tft(LCD_CS_PIN, LCD_DC_PIN, LCD_RES_PIN);

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;
WiFiServer server(80);
bool serverStarted = false;
uint32_t lastWifiTry = 0;

const char *poi_numbers[] = {
  // board goes 1..9,0
  "18,-6",
  "-18,-6",
  "-14,-6",
  "-10,-6",
  "-6,-6",
  "-2,-6",
  "2,-6",
  "6,-6",
  "10,-6",
  "14,-6"
};
const char *poi_letters[] = {
  "-24,7",
  "-20,8",
  "-16,9",
  "-12,9",
  "-8,10", //e
  "-4,10",
  "0,10",
  "4,10",
  "8,10",
  "12,9", //j
  "16,9",
  "20,8",
  "24,7",
  "-24,-1",
  "-20,1", //o
  "-16,2",
  "-12,2",
  "-8,3",
  "-4,3",
  "0,3", //t
  "4,3",
  "8,3",
  "12,2",
  "16,2",
  "20,1", //y
  "24,-1"
};
#define POI_YES 0
#define POI_NO 1
#define POI_REST 2
const char *poi_other[] = {
  "-18,18", "18,18", "0,-15"
};

// points still to be sent to the Uno
const char *pointQueue[MAX_POINTS];
uint8_t queueHead = 0;
uint8_t queueCount = 0;
uint8_t pointsSent = 0;
uint8_t pointsArrived = 0;
uint32_t nextPointAt = 0;

bool busy = false;
uint32_t busySince = 0;
bool notifyIdle = false;

uint32_t clearScreenAt = 0;  // 0 = nothing scheduled

// ---- display ----

void fillScreen() {
  tft.fillScreen(GC9A01A_BLACK);
  yield();
}

void drawStrCentered(String str, uint8_t textSize, uint16_t textColor) {
  int16_t x1, y1;
  int16_t x = 120, y = 190;
  switch (textSize) {
    case 1:
      y = 160; break;
    case 2:
      y = 190; break;
    case 4:
      y = 250; break;
  }

  uint16_t w, h;
  tft.setTextSize(textSize);
  tft.getTextBounds(str, tft.getCursorX(), tft.getCursorY(), &x1, &y1, &w, &h);
  fillScreen();
  tft.setCursor(x - w / 2, y - h / 2);
  tft.setTextColor(textColor);
  tft.println(str);
}

void drawStrCentered(String str, uint8_t textSize) {
  drawStrCentered(str, textSize, RGB_TAN);
}

// Two lines of text, stacked around the middle of the round screen. The first line sits a little above centre and the
// second below, close enough that both stay inside the circle.
void drawTwoLines(const char *line1, const char *line2, uint8_t textSize, uint16_t textColor) {
  const int16_t centers[2] = {92, 150};
  const char *lines[2] = {line1, line2};
  fillScreen();
  tft.setTextSize(textSize);
  tft.setTextColor(textColor);
  for (uint8_t i = 0; i < 2; i++) {
    int16_t x1, y1;
    uint16_t w, h;
    tft.getTextBounds(lines[i], 0, 0, &x1, &y1, &w, &h);
    tft.setCursor(120 - w / 2 - x1, centers[i] - h / 2 - y1);
    tft.print(lines[i]);
  }
}

// shows the character the bot has just reached, looked up from its coordinates
void drawPoint(const char *xy) {
  if (strcmp(xy, poi_other[POI_YES]) == 0) {
    drawStrCentered("yes", 2);
    return;
  }
  if (strcmp(xy, poi_other[POI_NO]) == 0) {
    drawStrCentered("no", 2);
    return;
  }
  if (strcmp(xy, poi_other[POI_REST]) == 0) {
    return;  // nothing to draw
  }

  for (uint8_t j = 0; j < 26; j++) {
    if (strcmp(xy, poi_letters[j]) == 0) {
      drawStrCentered(String((char) ('a' + j)), 4);
      return;
    }
  }
  for (uint8_t j = 0; j < 10; j++) {
    if (strcmp(xy, poi_numbers[j]) == 0) {
      drawStrCentered(String((char) ('0' + j)), 4);
      return;
    }
  }
}

void showFor(const String &str, uint8_t textSize, uint16_t color, uint32_t ms) {
  drawStrCentered(str, textSize, color);
  clearScreenAt = millis() + ms;
}

// ---- link to the Uno ----

void sendPoi(const char *xy) {
  Serial1.write("$poi(");
  Serial1.write(xy);
  Serial1.write(')');
}

void finishSpelling() {
  busy = false;
  notifyIdle = true;
  Serial.println(F("[d] spelling finished"));
}

// Every point sent has been reached, and none is still waiting to be sent.
bool allPointsReached() {
  return queueCount == 0 && pointsArrived >= pointsSent;
}

void handleUnoMessage(char *msg) {
  // msg is "$name(args" without the closing bracket
  char *open = strchr(msg, '(');
  if (open == nullptr) return;
  *open = '\0';
  const char *name = msg + 1;
  const char *args = open + 1;

  if (strcmp(name, "arrived") == 0) {
    pointsArrived++;
    drawPoint(args);
    clearScreenAt = millis() + CHARACTER_SHOW_MS;
    if (busy && allPointsReached()) finishSpelling();
  } else if (strcmp(name, "check") == 0) {
    showFor("check", 1, RGB_TAN, CHARACTER_SHOW_MS);
  } else if (strcmp(name, "orient") == 0) {
    // after the calibration spin: stays up until the Uno says it has recorded the zero
    drawTwoLines("spin me", "to 0", 1, RGB_TAN);
    clearScreenAt = 0;
  } else if (strcmp(name, "ready") == 0) {
    showFor("ready", 1, RGB_TAN, CHARACTER_SHOW_MS);
  } else if (strcmp(name, "smiley") == 0) {
    showFor(":)", 2, RGB_TAN, CHARACTER_SHOW_MS);
    // the Uno has emptied its queue; if this side has nothing left to send either, the bot is idle
    if (busy && queueCount == 0) finishSpelling();
  }
}

void pumpUno() {
  static char buf[24];
  static uint8_t len = 0;
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '$') len = 0;  // a new message starts here, whatever came before
    if (len < sizeof buf - 1) buf[len++] = c;
    if (c == ')') {
      buf[len - 1] = '\0';
      if (buf[0] == '$') handleUnoMessage(buf);
      len = 0;
    }
  }
}

// Sends the next queued point once the spacing has passed and the Uno has reached the previous one. The Uno's software
// serial cannot receive while it is sending ($arrived), so points sent ahead of it could collide with its replies and
// lose a letter. If a reply never comes, the next point goes anyway after POI_ACK_TIMEOUT_MS.
void pumpPoints() {
  if (queueCount == 0 || (int32_t) (millis() - nextPointAt) < 0) return;
  if (pointsSent > pointsArrived && (int32_t) (millis() - (nextPointAt + POI_ACK_TIMEOUT_MS)) < 0) return;
  sendPoi(pointQueue[queueHead]);
  queueHead = (queueHead + 1) % MAX_POINTS;
  queueCount--;
  pointsSent++;
  nextPointAt = millis() + POI_SPACING_MS;
}

// ---- backend ----

void tellBackendIdle() {
#ifdef BACKEND_HOST
  WiFiClient client;
  if (client.connect(BACKEND_HOST, BACKEND_PORT)) {
    client.print(F("POST /bot/idle HTTP/1.1\r\nHost: "));
    client.print(BACKEND_HOST);
    client.print(F("\r\nContent-Length: 0\r\nConnection: close\r\n\r\n"));
    client.flush();
    client.stop();
    Serial.println(F("[d] told the backend the bot is idle"));
  } else {
    Serial.println(F("[d] could not reach the backend"));
  }
#endif
}

// ---- HTTP ----

void respond(WiFiClient &client, int code, const char *reason, const String &body) {
  client.print(F("HTTP/1.1 "));
  client.print(code);
  client.print(' ');
  client.println(reason);
  client.println(F("Content-Type: text/plain"));
  client.println(F("Connection: close"));
  client.print(F("Content-Length: "));
  client.println(body.length());
  client.println();
  client.print(body);
  client.flush();
  client.stop();
}

// Reads one line into out (without the line end). Returns false on timeout.
bool readLine(WiFiClient &client, String &out, uint32_t deadline) {
  out = "";
  while ((int32_t) (millis() - deadline) < 0) {
    pumpUno();
    while (client.available() > 0) {
      char c = client.read();
      if (c == '\n') return true;
      if (c != '\r' && out.length() < 120) out += c;
    }
    if (!client.connected()) return false;
  }
  return false;
}

// Undoes form encoding: "+" is a space and %XX is a byte. A malformed %XX is dropped.
String urlDecode(const String &in) {
  String out;
  for (unsigned int i = 0; i < in.length(); i++) {
    char c = in[i];
    if (c == '+') {
      out += ' ';
    } else if (c == '%' && i + 2 < in.length() && isxdigit(in[i + 1]) && isxdigit(in[i + 2])) {
      char hex[3] = {in[i + 1], in[i + 2], '\0'};
      out += (char) strtol(hex, nullptr, 16);
      i += 2;
    } else if (c != '%') {
      out += c;
    }
  }
  return out;
}

// Pulls a form field's value out of "a=1&b=2".
String formField(const String &body, const String &name) {
  String key = name + "=";
  int start = body.startsWith(key) ? 0 : body.indexOf("&" + key);
  if (start < 0) return "";
  start += (start == 0 ? 0 : 1) + key.length();
  int end = body.indexOf('&', start);
  return urlDecode(end < 0 ? body.substring(start) : body.substring(start, end));
}

void handleHeyOuija(WiFiClient &client) {
  // while the bot is spelling, the display belongs to the characters
  if (!busy) showFor("?", 4, RGB_TAN, HEY_OUIJA_SHOW_MS);
  respond(client, 200, "OK", "ok");
}

void handleTryAgain(WiFiClient &client) {
  if (busy) {
    respond(client, 409, "Conflict", "busy");
    return;
  }
  showFor("?", 4, GC9A01A_RED, TRY_AGAIN_SHOW_MS);
  respond(client, 200, "OK", "ok");
}

void enqueue(const char *xy) {
  pointQueue[(queueHead + queueCount) % MAX_POINTS] = xy;
  queueCount++;
}

void handleAnswer(WiFiClient &client, const String &body) {
  String answer = formField(body, "answer");
  answer.toLowerCase();

  if (busy) {
    respond(client, 409, "Conflict", "busy");
    return;
  }

  // work out the points first, so a bad answer changes nothing
  const char *points[MAX_POINTS];
  uint8_t count = 0;
  if (answer == "yes") {
    points[count++] = poi_other[POI_YES];
  } else if (answer == "no") {
    points[count++] = poi_other[POI_NO];
  } else {
    for (unsigned int i = 0; i < answer.length() && count < MAX_POINTS; i++) {
      char c = answer[i];
      if (c >= '0' && c <= '9') points[count++] = poi_numbers[c - '0'];
      else if (c >= 'a' && c <= 'z') points[count++] = poi_letters[c - 'a'];
    }
  }
  if (count == 0) {
    respond(client, 400, "Bad Request", "no letters or digits in answer");
    return;
  }

  queueHead = 0;
  queueCount = 0;
  for (uint8_t i = 0; i < count; i++) enqueue(points[i]);
  pointsSent = 0;
  pointsArrived = 0;
  nextPointAt = millis();
  busy = true;
  busySince = millis();
  clearScreenAt = 0;
  fillScreen();
  Serial.print(F("[d] answer="));
  Serial.println(answer);
  respond(client, 200, "OK", "ok");
}

void handleStatus(WiFiClient &client) {
  String out;
  out += "busy ";
  out += busy ? 1 : 0;
  out += "\nqueued ";
  out += queueCount;
  out += "\nsent ";
  out += pointsSent;
  out += "\narrived ";
  out += pointsArrived;
  out += "\nuptime_ms ";
  out += millis();
  out += "\nrssi ";
  out += WiFi.RSSI();
  out += "\n";
  respond(client, 200, "OK", out);
}

void handleClient(WiFiClient &client) {
  uint32_t deadline = millis() + REQUEST_TIMEOUT_MS;
  String requestLine;
  if (!readLine(client, requestLine, deadline)) {
    client.stop();
    return;
  }

  int contentLength = 0;
  String header;
  while (readLine(client, header, deadline) && header.length() > 0) {
    header.toLowerCase();
    if (header.startsWith("content-length:")) contentLength = header.substring(15).toInt();
  }

  String body;
  if (contentLength > BODY_MAX) {
    respond(client, 400, "Bad Request", "body too large");
    return;
  }
  while ((int) body.length() < contentLength && (int32_t) (millis() - deadline) < 0) {
    pumpUno();
    while (client.available() > 0 && (int) body.length() < contentLength) body += (char) client.read();
  }

  if (requestLine.startsWith("POST /heyOuija")) handleHeyOuija(client);
  else if (requestLine.startsWith("POST /answer")) handleAnswer(client, body);
  else if (requestLine.startsWith("POST /tryAgain")) handleTryAgain(client);
  else if (requestLine.startsWith("GET /status")) handleStatus(client);
  else respond(client, 404, "Not Found", "not found");
}

// ---- setup / loop ----

void connectWifi() {
  Serial.print(F("Connecting to "));
  Serial.println(ssid);
  int status = WL_IDLE_STATUS;
  uint32_t start = millis();
  while (status != WL_CONNECTED && millis() - start < 20000) {
    status = WiFi.begin(ssid, pass);
    delay(1000);
  }
  if (status == WL_CONNECTED) {
    if (!serverStarted) {
      server.begin();
      serverStarted = true;
    }
    Serial.print(F("HTTP server on "));
    Serial.println(WiFi.localIP());
  } else {
    Serial.println(F("WiFi connect failed; will retry."));
  }
}

void setup() {
  Serial.begin(115200);
  uint32_t start = millis();
  while (!Serial && millis() - start < 1500) {}

  Serial1.begin(SERIAL1_BAUDRATE);

  SPI.begin();
  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(0x0000);
  tft.setFont(&CAPTH___24pt7b);

  drawStrCentered("hi", 2);
  connectWifi();
  delay(500);
  fillScreen();
}

void loop() {
  pumpUno();
  pumpPoints();

  WiFiClient client = server.available();
  if (client) handleClient(client);

  if (clearScreenAt != 0 && (int32_t) (millis() - clearScreenAt) >= 0) {
    clearScreenAt = 0;
    fillScreen();
  }

  if (busy && millis() - busySince > BUSY_TIMEOUT_MS) {
    Serial.println(F("[d] gave up waiting for the Uno"));
    queueCount = 0;
    finishSpelling();
  }

  if (notifyIdle) {
    notifyIdle = false;
    tellBackendIdle();
  }

  if (WiFi.status() != WL_CONNECTED && millis() - lastWifiTry > 15000) {
    lastWifiTry = millis();
    connectWifi();
  }
}
