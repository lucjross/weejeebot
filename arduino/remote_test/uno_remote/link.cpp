#include "link.h"
#include "config.h"
#include <NeoSWSerial.h>

namespace {

NeoSWSerial nss(LINK_RX_PIN, LINK_TX_PIN);
char line[LINE_MAX];
uint8_t len = 0;
bool overflow = false;

}  // namespace

namespace Link {

void begin() {
  nss.begin(LINK_BAUD);
}

bool readLine(char *buf) {
  while (nss.available() > 0) {
    char c = nss.read();
    if (c == '\r') continue;
    if (c == '\n') {
      bool ok = !overflow && len > 0;
      if (ok) {
        memcpy(buf, line, len);
        buf[len] = '\0';
      }
      len = 0;
      overflow = false;
      if (ok) return true;
      continue;
    }
    if (len < LINE_MAX - 1) {
      line[len++] = c;
    } else {
      overflow = true;
    }
  }
  return false;
}

void sendLine(const char *s) {
  nss.print(s);
  nss.write('\n');
}

}  // namespace Link
