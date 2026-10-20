// Nano 33 IoT side of the remote test harness. See ../README.md for the protocol.
//
// Joins WiFi and serves a tiny HTTP API. POST /cmd forwards a command line to the Uno over Serial1;
// lines coming back from the Uno are kept in a ring buffer that GET /log returns. No authentication.
#include <SPI.h>
#include <WiFiNINA.h>
#include "secrets.h"  // SECRET_SSID, SECRET_PASS (copy secrets.h.example)

#define LINK_BAUD 19200
#define LOG_LINES 40
#define LINE_LEN 128
#define CMD_MAX 100

char ssid[] = SECRET_SSID;
char pass[] = SECRET_PASS;

WiFiServer server(80);

char logText[LOG_LINES][LINE_LEN];
uint32_t nextSeq = 1;  // sequence number of the next line stored; line n lives in slot n % LOG_LINES

char rxLine[LINE_LEN];
uint8_t rxLen = 0;

uint32_t lastWifiTry = 0;
bool serverStarted = false;

void pushLine(const char *s) {
  strncpy(logText[nextSeq % LOG_LINES], s, LINE_LEN - 1);
  logText[nextSeq % LOG_LINES][LINE_LEN - 1] = '\0';
  nextSeq++;
}

// Moves any bytes the Uno has sent into the log. Call often so Serial1's small buffer never overflows.
void pumpSerial() {
  while (Serial1.available() > 0) {
    char c = Serial1.read();
    if (c == '\r') continue;
    if (c == '\n') {
      if (rxLen > 0) {
        rxLine[rxLen] = '\0';
        pushLine(rxLine);
      }
      rxLen = 0;
    } else if (rxLen < LINE_LEN - 1) {
      rxLine[rxLen++] = c;
    } else {
      rxLen = 0;  // over-long line: drop it
    }
  }
}

void connectWifi() {
  Serial.print(F("Connecting to "));
  Serial.println(ssid);
  int status = WL_IDLE_STATUS;
  uint32_t start = millis();
  while (status != WL_CONNECTED && millis() - start < 20000) {
    status = WiFi.begin(ssid, pass);
    delay(1000);
    Serial.print('.');
  }
  Serial.println();
  if (status == WL_CONNECTED) {
    if (!serverStarted) {
      server.begin();
      serverStarted = true;
    }
    Serial.print(F("Connected. IP address: "));
    Serial.println(WiFi.localIP());
    char note[48];
    snprintf(note, sizeof note, "N wifi up %d.%d.%d.%d", WiFi.localIP()[0], WiFi.localIP()[1],
             WiFi.localIP()[2], WiFi.localIP()[3]);
    pushLine(note);
  } else {
    Serial.println(F("WiFi connect failed; will retry."));
  }
}

String urlDecode(const String &s) {
  String out;
  for (unsigned i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '+') {
      out += ' ';
    } else if (c == '%' && i + 2 < s.length() + 0 && isxdigit(s[i + 1]) && isxdigit(s[i + 2])) {
      out += (char) strtol(s.substring(i + 1, i + 3).c_str(), nullptr, 16);
      i += 2;
    } else {
      out += c;
    }
  }
  return out;
}

// Finds name in a "a=1&b=2" string. Returns true and fills value if present.
bool getParam(const String &params, const char *name, String &value) {
  String key = String(name) + "=";
  int start = 0;
  while (start <= (int) params.length()) {
    int end = params.indexOf('&', start);
    if (end < 0) end = params.length();
    String pair = params.substring(start, end);
    if (pair.startsWith(key)) {
      value = urlDecode(pair.substring(key.length()));
      return true;
    }
    start = end + 1;
  }
  return false;
}

// Reads one line (up to '\n') from the client without blocking the serial pump. Returns false on timeout.
bool readClientLine(WiFiClient &client, String &line, uint32_t timeoutMs) {
  line = "";
  uint32_t deadline = millis() + timeoutMs;
  while (millis() < deadline) {
    pumpSerial();
    while (client.available() > 0) {
      char c = client.read();
      if (c == '\n') return true;
      if (c != '\r') line += c;
    }
    if (!client.connected() && client.available() == 0) break;
  }
  return false;
}

void respond(WiFiClient &client, int code, const String &body) {
  client.print(F("HTTP/1.1 "));
  client.print(code);
  client.println(code == 200 ? F(" OK") : F(" Bad Request"));
  client.println(F("Content-Type: text/plain"));
  client.println(F("Connection: close"));
  client.print(F("Content-Length: "));
  client.println(body.length());
  client.println();
  client.print(body);
  client.flush();
  client.stop();
}

String buildLog(uint32_t since) {
  uint32_t newest = nextSeq - 1;  // 0 if nothing yet
  uint32_t oldest = (nextSeq > LOG_LINES) ? nextSeq - LOG_LINES : 1;
  uint32_t missed = (since + 1 < oldest) ? oldest - (since + 1) : 0;
  String out = "next ";
  out += newest;
  out += " dropped ";
  out += missed;
  out += "\n";
  for (uint32_t seq = max(oldest, since + 1); seq <= newest; seq++) {
    out += seq;
    out += ' ';
    out += logText[seq % LOG_LINES];
    out += '\n';
  }
  return out;
}

void handleClient() {
  WiFiClient client = server.available();
  if (!client) return;

  String requestLine;
  if (!readClientLine(client, requestLine, 500)) {
    client.stop();
    return;
  }

  int contentLength = 0;
  String header;
  while (readClientLine(client, header, 500)) {
    if (header.length() == 0) break;
    header.toLowerCase();
    if (header.startsWith("content-length:")) {
      contentLength = header.substring(15).toInt();
    }
  }

  // "METHOD /path?query HTTP/1.1"
  int sp1 = requestLine.indexOf(' ');
  int sp2 = requestLine.indexOf(' ', sp1 + 1);
  if (sp1 < 0 || sp2 < 0) {
    respond(client, 400, "bad request\n");
    return;
  }
  String target = requestLine.substring(sp1 + 1, sp2);
  String path = target;
  String query = "";
  int q = target.indexOf('?');
  if (q >= 0) {
    path = target.substring(0, q);
    query = target.substring(q + 1);
  }

  String body = "";
  uint32_t deadline = millis() + 500;
  while ((int) body.length() < contentLength && millis() < deadline) {
    pumpSerial();
    while (client.available() > 0 && (int) body.length() < contentLength) {
      body += (char) client.read();
    }
  }

  if (path == "/cmd") {
    String cmd;
    if (!getParam(body, "cmd", cmd) && !getParam(query, "cmd", cmd)) {
      respond(client, 400, "missing cmd\n");
      return;
    }
    cmd.replace("\r", "");
    cmd.replace("\n", "");
    if (cmd.length() == 0 || cmd.length() > CMD_MAX) {
      respond(client, 400, "bad cmd length\n");
      return;
    }
    Serial1.print(cmd);
    Serial1.write('\n');
    respond(client, 200, "queued\n");
  } else if (path == "/log") {
    String sinceText;
    uint32_t since = getParam(query, "since", sinceText) ? (uint32_t) sinceText.toInt() : 0;
    respond(client, 200, buildLog(since));
  } else if (path == "/status") {
    String out = "uptime_ms ";
    out += millis();
    out += "\nrssi ";
    out += WiFi.RSSI();
    out += "\nnext_seq ";
    out += nextSeq;
    out += "\n";
    respond(client, 200, out);
  } else {
    respond(client, 200, "remote test relay: POST /cmd (cmd=...), GET /log?since=N, GET /status\n");
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.begin(LINK_BAUD);
  pushLine("N boot");
  connectWifi();
}

void loop() {
  pumpSerial();

  if (WiFi.status() != WL_CONNECTED) {
    if (millis() - lastWifiTry > 10000) {
      lastWifiTry = millis();
      connectWifi();
    }
    return;
  }

  handleClient();
}
