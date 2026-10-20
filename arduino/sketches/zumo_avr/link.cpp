#include <NeoSWSerial.h>
#include "link.h"
#include "globals.h"

NeoSWSerial nss(SS_RX, SS_TX);

void linkBegin() {
  nss.begin(SS_BAUDRATE);
}

void sendArrived(point p) {
  nss.write("$arrived(");
  char x[4], y[4];
  itoa(p.x, x, 10);
  // board coordinate Y matches the POI Y (the screen should be over it)
  itoa(p.y + PLANCH_CENTER_TO_SCREEN_cm, y, 10);
  nss.write(x); nss.write(','); nss.write(y);
  nss.write(')');
}

void sendSmiley() {
  nss.write("$smiley()");
}

void sendOrient() {
  nss.write("$orient()");
  nss.flush();
}

void sendReady() {
  nss.write("$ready()");
  nss.flush();
}

void sendCheck() {
  nss.write("$check()");
  nss.flush();
}

char serial_read_stage[16];
uint8_t serial_bytes_staged;
char poi_cmd[] = "$poi(";
#define POI_CMD_X_IDX 5
volatile bool reading = false;

bool readNSS(void *) {
  if (reading) {
    return true;
  }

  reading = true;
  while (nss.available() > 0) {
    uint8_t c = nss.read();
    Serial.print(F("[d] c=")); Serial.println((char) c);
    serial_read_stage[serial_bytes_staged++] = (char) c;
    if (serial_bytes_staged >= 16) {
      serial_bytes_staged = 0;
    }
    
    if (c == ')') {
      serial_bytes_staged = 0;
      
      bool is_add_poi = true;
      for (uint8_t i = 0; i < POI_CMD_X_IDX; i++) {
        if (serial_read_stage[i] != poi_cmd[i]) {
          is_add_poi = false;
          break;
        }
      }
  
      if (is_add_poi) {
        point p;
        strtok(serial_read_stage, "(,");
        p.x = atoi(strtok(nullptr, "(,"));
        // board coordinate Y matches the POI Y (the screen should be over it)
        p.y = atoi(strtok(nullptr, "(,")) - PLANCH_CENTER_TO_SCREEN_cm;
  
        Serial.print(F("[d] enqing p.x=")); Serial.print(p.x);
        Serial.print(F(", p.y=")); Serial.println(p.y);
  
        global.poi_queue.enqueue(p);
      }
    }
  }

  reading = false;

  return true;
}
