// Bench test for the GC9A01 round display on the Nano 33 IoT. No WiFi, no Uno link.
// Sweeps red, green, blue, white and black, draws a ring and a cross-hair to show the visible circle and orientation,
// then shows "Screen OK" with a seconds counter. Pins match display_and_web.ino.
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_GC9A01A.h>

#define LCD_RES_PIN 7
#define LCD_DC_PIN 8
#define LCD_CS_PIN 9
// SCL goes to SCK (D13), SDA goes to MOSI (D11)

#define COLOR_BLACK 0x0000
#define COLOR_RED 0xF800
#define COLOR_GREEN 0x07E0
#define COLOR_BLUE 0x001F
#define COLOR_WHITE 0xFFFF
#define COLOR_YELLOW 0xFFE0

Adafruit_GC9A01A tft(LCD_CS_PIN, LCD_DC_PIN, LCD_RES_PIN);

uint32_t seconds = 0;

void setup() {
  Serial.begin(115200);
  SPI.begin();
  tft.begin();
  tft.setRotation(2);

  const uint16_t fills[] = {COLOR_RED, COLOR_GREEN, COLOR_BLUE, COLOR_WHITE, COLOR_BLACK};
  for (uint8_t i = 0; i < 5; i++) {
    tft.fillScreen(fills[i]);
    delay(700);
  }

  // outline of the full 240x240 area and the visible circle, plus a cross-hair; the "T" marks the top edge
  tft.drawRect(0, 0, 240, 240, COLOR_WHITE);
  tft.drawCircle(120, 120, 119, COLOR_YELLOW);
  tft.drawFastHLine(0, 120, 240, COLOR_GREEN);
  tft.drawFastVLine(120, 0, 240, COLOR_GREEN);
  tft.setTextColor(COLOR_RED);
  tft.setTextSize(3);
  tft.setCursor(108, 20);
  tft.print("T");
  delay(2500);

  tft.fillScreen(COLOR_BLACK);
  tft.setTextColor(COLOR_WHITE);
  tft.setTextSize(4);
  tft.setCursor(36, 85);
  tft.print("Screen OK");
  Serial.println(F("display_test running"));
}

void loop() {
  tft.fillRect(80, 135, 80, 30, COLOR_BLACK);
  tft.setTextColor(COLOR_YELLOW);
  tft.setTextSize(3);
  tft.setCursor(88, 138);
  tft.print(seconds++);
  delay(1000);
}
