/****************************************************
 * EE303 ESP32 Server Integration + TEST STUBS (OPTIMIZED)
 ****************************************************/

#include <WiFi.h>

// ====== DEBUG SWITCH ======
#define DEBUG 1
#if DEBUG
  #define DBG_PRINT(x)   Serial.print(x)
  #define DBG_PRINTLN(x) Serial.println(x)
#else
  #define DBG_PRINT(x)   do{}while(0)
  #define DBG_PRINTLN(x) do{}while(0)
#endif

// ===== WiFi credentials =====
static const char* ssid     = "iot";
static const char* password = "launchable72unstrict";

// ===== Server config =====
static const char* TEAM_ID = "etgf6754";
static const char* SERVER_HOST = "3.250.38.184";
static const uint16_t SERVER_PORT = 8000;

// ===== Group number =====
static const int GROUP_NUMBER = 18;

// ----------------------------------------------------
// normalizeNode(): KEEPING your mapping exactly: 5 -> 7
// ----------------------------------------------------
int normalizeNode(int n) {
  if (n == 5) return 7;
  return n;
}

// ----------------------------------------------------
// WiFi connect
// ----------------------------------------------------
void wifiConnectBlocking() {
#if DEBUG
  Serial.print("Connecting to WiFi");
#endif
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(300);
#if DEBUG
    Serial.print(".");
#endif
  }
#if DEBUG
  Serial.println("\nWiFi connected.");
  Serial.print("IP: ");
  Serial.println(WiFi.localIP());
#endif
}

// ----------------------------------------------------
// Minimal HTTP POST using WiFiClient
// ----------------------------------------------------
int postArrivedAndGetNextTarget(int group, int position) {
  if (WiFi.status() != WL_CONNECTED) return -1;

  position = normalizeNode(position);

  WiFiClient client;
  if (!client.connect(SERVER_HOST, SERVER_PORT)) {
    DBG_PRINTLN("connect() failed");
    return -1;
  }

  char path[64];
  snprintf(path, sizeof(path), "/api/arrived/%s", TEAM_ID);

  char body[48];
  int bodyLen = snprintf(body, sizeof(body), "group=%d&position=%d", group, position);

  // Send HTTP request
  client.print("POST ");
  client.print(path);
  client.print(" HTTP/1.1\r\n");
  client.print("Host: ");
  client.print(SERVER_HOST);
  client.print(":");
  client.print(SERVER_PORT);
  client.print("\r\n");
  client.print("Content-Type: application/x-www-form-urlencoded\r\n");
  client.print("Connection: close\r\n");
  client.print("Content-Length: ");
  client.print(bodyLen);
  client.print("\r\n\r\n");
  client.print(body);

#if DEBUG
  Serial.println();
  Serial.print("POST "); Serial.print(path);
  Serial.print(" body="); Serial.println(body);
#endif

  // Wait until something arrives
  unsigned long start = millis();
  while (!client.available()) {
    if (!client.connected()) { client.stop(); return -1; }
    if (millis() - start > 5000) { client.stop(); return -1; }
    delay(1);
  }

  // Read status line (up to '\n')
  char statusLine[64] = {0};
  size_t i = 0;
  while (client.available() && i < sizeof(statusLine) - 1) {
    char c = client.read();
    if (c == '\n') break;
    if (c != '\r') statusLine[i++] = c;
  }

#if DEBUG
  Serial.print("Status: "); Serial.println(statusLine);
#endif

  // Parse status code safely from "HTTP/1.1 200 OK"
  int status = -1;
  // find first space, then atoi after it
  char* sp = strchr(statusLine, ' ');
  if (sp && *(sp + 1)) status = atoi(sp + 1);

  // ----- FIX #1: Robustly skip headers even if server closes early -----
  // Consume until we see \r\n\r\n
  uint8_t state = 0;
  unsigned long hdrStart = millis();
  while (state < 4) {
    if (client.available()) {
      char c = client.read();
      if      (state == 0 && c == '\r') state = 1;
      else if (state == 1 && c == '\n') state = 2;
      else if (state == 2 && c == '\r') state = 3;
      else if (state == 3 && c == '\n') state = 4;
      else state = (c == '\r') ? 1 : 0;
    } else {
      // If nothing is available, either wait briefly or bail on timeout.
      if (!client.connected()) break; // connection closed; no more bytes coming
      if (millis() - hdrStart > 5000) { client.stop(); return -1; }
      delay(1);
    }
  }

  // Read body (drain what’s available; body may be empty on 400)
  char resp[128];
  size_t r = 0;
  unsigned long bodyStart = millis();
  while (client.available() || client.connected()) {
    if (client.available()) {
      char c = client.read();
      // keep it compact/readable
      if (c == '\r' || c == '\n') continue;
      if (r < sizeof(resp) - 1) resp[r++] = c;
    } else {
      if (millis() - bodyStart > 1500) break;
      delay(1);
    }
  }
  resp[r] = '\0';
  client.stop();

#if DEBUG
  Serial.print("Body: "); Serial.println(resp);
#endif

  // ----- FIX #2: reject non-200 -----
  if (status != 200) {
    return -1;
  }

  // Handle terminal response
  const char* finished = "Already Finished";
  if (strncmp(resp, finished, 16) == 0) {
    return -2;
  }

  // Parse integer response (digits only)
  if (resp[0] == '\0') return -1;
  for (size_t k = 0; resp[k]; k++) {
    if (resp[k] < '0' || resp[k] > '9') return -1;
  }

  int next = atoi(resp);
  return normalizeNode(next);
}

/****************************************************
 * TEST STUBS (replace later with real robot code)
 ****************************************************/
int sourceJunction = 0;
int destinationJunction = 0;

int routeNum = 0;
int routeIndex = 0;

void clearRoutes() { DBG_PRINTLN("clearRoutes()"); }
void findRoutes(int src, int dst) {
#if DEBUG
  Serial.print("findRoutes(): "); Serial.print(src);
  Serial.print(" -> "); Serial.println(dst);
#endif
}
int chooseRoute() { DBG_PRINTLN("chooseRoute() -> 0"); return 0; }

/****************************************************
 * Arduino setup/loop
 ****************************************************/
void setup() {
  // Keep baud at 9600 as requested
  Serial.begin(9600);
  wifiConnectBlocking();
}

void loop() {
  static unsigned long lastMs = 0;
  const unsigned long INTERVAL_MS = 3000;
  unsigned long now = millis();
  if (now - lastMs < INTERVAL_MS) return;
  lastMs = now;

  int next = postArrivedAndGetNextTarget(GROUP_NUMBER, sourceJunction);

  // TERMINATE on "Already Finished"
  if (next == -2) {
#if DEBUG
    Serial.println("PROGRAM TERMINATED BY SERVER (Already Finished)");
#endif
    pinMode(LED_BUILTIN, OUTPUT);
    digitalWrite(LED_BUILTIN, HIGH);
    while (true) delay(1000);
  }

  if (next >= 0) {
    destinationJunction = next;

    clearRoutes();
    findRoutes(sourceJunction, destinationJunction);
    routeNum = chooseRoute();
    routeIndex = 0;

    // teleport for test
    sourceJunction = destinationJunction;
  }
}
// DISTANCE SENSOR CODE
float sensor_distance() {
  int raw = analogRead(sensor);                 // ESP32 default: 0..4095
  float volts = (raw * 3.3f) / 4095.0f;         // convert to volts (assuming 3.3V ADC range)

  if (volts <= 0.001f) {
    return -1.0f;                               // invalid / avoid pow(0, -1)
  }

  float distance = 13.0f * powf(volts, -1.0f);  // from datasheet fit

  delay(50);

  if (distance <= 30.0f) {
    Serial.println(distance);
  }

  return distance;
}
