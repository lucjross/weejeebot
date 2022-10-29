#include <SPI.h>
#include "Adafruit_GFX.h"
#include "Adafruit_GC9A01A.h"
#include "OuijaFont.h"
#include <arduino-timer.h>
#include <WiFiWebServer.h>
#include "secrets.h"

#define LCD_RES_PIN 7
#define LCD_DC_PIN 8
#define LCD_CS_PIN 9
/*
 * Note: SCL goes to SCK (D13), SDA goes to MOSI (D11)
 */

#define RGB_TAN 0xDE50

Adafruit_GC9A01A tft(LCD_CS_PIN, LCD_DC_PIN, LCD_RES_PIN);

#define SERIAL1_BAUDRATE 9600

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

int status = WL_IDLE_STATUS;
WiFiWebServer server(80);

auto timer = timer_create_default();

Timer<>::Task fillScreenTask;

/**
 * Handle POST /heyOuija
 */
void handle_heyOuija() {
  server.send(200);

//  String message = "handle_heyOuija";
//  message += ": " + server.args();
//  message += "\n";
//  for (uint8_t i = 0; i < server.args(); i++) {
//    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
//  }
//  Serial.println(message);

  drawStrCentered("?", 4);

  fillScreenTask = timer.in(10000, [](void *) -> bool {
    fillScreen();
  });
}

const char *poi_numbers[] PROGMEM = {
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
const char *poi_letters[] PROGMEM = {
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
const char *poi_other[] PROGMEM = {
  "-18,18", "18,18", "0,-15"
};

/**
 * Handle POST /tryAgain
 */
void handle_tryAgain() {
  server.send(200);

  timer.cancel(fillScreenTask);
  
  drawStrCentered("?", 4, GC9A01A_RED);
  timer.in(2000, [](void *) -> bool {
    fillScreen();
    return false;
  });
}

void drawPoint(char* xy) {
  const char **poi = poi_other;
  if (strcmp(xy, poi[POI_YES]) == 0) {
    drawStrCentered("yes", 2);
    return;
  }
  else if (strcmp(xy, poi[POI_NO]) == 0) {
    drawStrCentered("no", 2);
    return;
  }
  else if (strcmp(xy, poi[POI_REST]) == 0) {
    //nothing to draw
    return;
  }

  for (uint8_t j = 0; j < 26; j++) {
    if (strcmp(xy, poi_letters[j]) == 0) {
      drawStrCentered(String((const char) ('a' + j)), 4);
      return;
    }
  }

  for (uint8_t j = 0; j < 10; j++) {
    if (strcmp(xy, poi_numbers[j]) == 0) {
      drawStrCentered(String((const char) ('0' + j)), 4);
      return;
    }
  }
}

/**
 * Handle POST /answer
 */
void handle_answer() {
  if (server.args() != 2) {
    // 1 arg "answer=x" and 1 arg "plain: answer=x" (why?)
    server.send(400); return;
  }

  server.send(200);

  timer.cancel(fillScreenTask);
  fillScreen();

  const char* answer = server.arg(0).c_str();
  Serial.print("[d] answer="); Serial.println(answer);
  if (strcmp(answer, "yes") == 0) {
    sendAddPoi(poi_other[POI_YES]);
  }
  else if (strcmp(answer, "no") == 0) {
    sendAddPoi(poi_other[POI_NO]);
  }
  else {
    uint16_t write_delay_ms = 0;
    for (int i = 0; i < strlen(answer); i++) {
      const char *poi = nullptr;
      if (answer[i] >= '0' && answer[i] <= '9') {
        poi = poi_numbers[answer[i] - '0'];
      }
      else if (answer[i] >= 'a' && answer[i] <= 'z') {
        poi = poi_letters[answer[i] - 'a'];
      }

      if (poi != nullptr) {
        timer.in(write_delay_ms, [](void *_poi) -> bool {
          sendAddPoi((const char *) _poi);
        }, (void *) poi);

        write_delay_ms += 500;
      }
    }
  }
}

void sendAddPoi(const char* str) {
  Serial1.write("$addPoi(");
  Serial1.write(str);
  Serial1.write(')');
}

volatile char serial_read_stage[16];
volatile uint8_t serial_bytes_staged;
char arrived_cmd[] = "$arrived(";
uint8_t arrived_cmd_args_idx = 9;
char check_cmd[] = "$check(";
uint8_t check_cmd_args_idx = 7;
char smiley_cmd[] = "$smiley(";
uint8_t smiley_cmd_args_idx = 8;

bool cmdMatches(uint8_t len, char cmd[]) {
  for (uint8_t i = 0; i < len; i++) {
    if (serial_read_stage[i] != cmd[i]) {
      return false;
    }
  }

  return true;
}

bool readSerial1(void *) {
  while (Serial1.available() > 0) {
    uint8_t c = Serial1.read();
    Serial.print("[d] c="); Serial.println((char) c);
    
    serial_read_stage[serial_bytes_staged++] = (char) c;
    if (serial_bytes_staged >= 16) {
      serial_bytes_staged = 0;
    }
    
    if (c == ')') {
      serial_bytes_staged = 0;

      bool is_arrived = cmdMatches(arrived_cmd_args_idx, arrived_cmd);
      Serial.print("[d]is_arrived="); Serial.println(is_arrived);
      if (is_arrived) {
        strtok((char*) serial_read_stage, "()");
        char* xy = strtok(nullptr, "()");

        Serial.print("[d] arrived at x,y="); Serial.println(xy);

        drawPoint(xy);
        timer.in(3000, [](void *) -> bool {
          fillScreen();
        });

        return true;
      }

      bool is_check = cmdMatches(check_cmd_args_idx, check_cmd);
      Serial.print("[d]is_check="); Serial.println(is_check);
      if (is_check) {
        drawStrCentered("check", 1);
        timer.in(3000, [](void *) -> bool {
          fillScreen();
        });

        return true;
      }

      bool is_smiley = cmdMatches(smiley_cmd_args_idx, smiley_cmd);
      Serial.print("[d]is_smiley="); Serial.println(is_smiley);
      if (is_smiley) {
        drawStrCentered(":)", 2);
        timer.in(3000, [](void *) -> bool {
          fillScreen();
        });

        return true;
      }
    }
  }

  return true;
}

/**
 * Setup
 */
void setup() {
  Serial.begin(19200);
  while (!Serial && millis() < 5000) {}
  
  Serial.print(F("Attempting to connect to Network named: "));
  Serial.print(ssid);
  status = WiFi.begin(ssid, pass);
  while (status != WL_CONNECTED) {
    delay(1000);
    Serial.print('.');
    status = WiFi.begin(ssid, pass);
  }
  Serial.println(" connected");

  server.on(F("/heyOuija"), HTTP_POST, handle_heyOuija);
  server.on(F("/answer"), HTTP_POST, handle_answer);
  server.on(F("/tryAgain"), HTTP_POST, handle_tryAgain);
  server.begin();
  Serial.print(F("HTTP server started on "));
  Serial.println(WiFi.localIP());
  
  pinMode(LCD_RES_PIN, OUTPUT);
  pinMode(LCD_DC_PIN, OUTPUT);
  pinMode(LCD_CS_PIN, OUTPUT);

  SPI.begin();

  Serial1.begin(SERIAL1_BAUDRATE);

  timer.every(200, readSerial1);

  tft.begin();
  tft.setRotation(2);
  tft.fillScreen(0x0000);
  tft.setFont(&CAPTH___24pt7b);

  drawStrCentered("hi", 2);
  delay(500);
  fillScreen();
}

/**
 * loop
 */
void loop(void) {
  server.handleClient();

  timer.tick<void>();
}

void drawStrCentered(String str, uint8_t textSize) {
  drawStrCentered(str, textSize, RGB_TAN);
}

void drawStrCentered(String str, uint8_t textSize, uint16_t textColor) {
  int16_t x1, y1;
  int16_t x = 120, y;
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

void fillScreen() {
  tft.fillScreen(GC9A01A_BLACK);
  yield();
}
