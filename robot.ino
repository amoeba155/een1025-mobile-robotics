#include <WiFi.h>

// debug
#define DEBUG 1
#if DEBUG
#define DBG_PRINT(x) Serial.print(x)
#define DBG_PRINTLN(x) Serial.println(x)
#else
#define DBG_PRINT(x) \
  do { \
  } while (0)
#define DBG_PRINTLN(x) \
  do { \
  } while (0)
#endif

// wifi credentials
static const char* ssid = "iot";
static const char* password = "shipmatish73hyperdoricism";

// server config
static const char* TEAM_ID = "etgf6754";
static const char* SERVER_HOST = "3.250.38.184";
static const uint16_t SERVER_PORT = 8000;
static const int GROUP_NO = 18;

int normalizeNode(int n) {
  if (n == 5) return 7;
  return n;
}

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

int postArrivedAndGetNextTarget(int group, int position) {
  if (WiFi.status() != WL_CONNECTED) return -1;

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
    Serial.print("POST ");
    Serial.print(path);
    Serial.print(" body=");
    Serial.println(body);
  #endif

  // Wait until something arrives
  unsigned long start = millis();
  while (!client.available()) {
    if (!client.connected()) {
      client.stop();
      return -1;
    }
    if (millis() - start > 5000) {
      client.stop();
      return -1;
    }
    delay(1);
  }

  // Read status line (up to '\n')
  char statusLine[64] = { 0 };
  size_t i = 0;
  while (client.available() && i < sizeof(statusLine) - 1) {
    char c = client.read();
    if (c == '\n') break;
    if (c != '\r') statusLine[i++] = c;
  }

  #if DEBUG
    Serial.print("Status: ");
    Serial.println(statusLine);
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
      if (state == 0 && c == '\r') state = 1;
      else if (state == 1 && c == '\n') state = 2;
      else if (state == 2 && c == '\r') state = 3;
      else if (state == 3 && c == '\n') state = 4;
      else state = (c == '\r') ? 1 : 0;
    } else {
      // If nothing is available, either wait briefly or bail on timeout.
      if (!client.connected()) break;  // connection closed; no more bytes coming
      if (millis() - hdrStart > 5000) {
        client.stop();
        return -1;
      }
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
    Serial.print("Body: ");
    Serial.println(resp);
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

// pin initialization
int leftMotorPhase = 20;
int leftMotorPWM = 39;
int rightMotorPhase = 38;
int rightMotorPWM = 37;

int osLinePins[3] = { 7, 6, 5 };
int osJunctionPins[2] = { 15, 4 };

int dsTriggerPin = 35;
int dsEchoPin = 45;

// optical sensor values
int lineAnalogValues[3] = { 0, 0, 0 };
int junctionAnalogValues[2] = { 0, 0 };

// physical dimensions
int wheelbase = 136;  // mm
float distance = 0;
int spinTime = 0;
float pwmScale = 1.9;
float weight, sum, average = 0;

// PID control/speed values
// 145, 220, 10, 0
int desiredPWM = 185;  // 0-255
float Kp = 220.0;      // 220
float Kd = 10.0;       // 25
float Ki = 0.5;
double derivative, integral = 1.0;
double dt = 0;
unsigned long now, lastTime = 0;
float error, lastError = 0.0;

// map, index is junction, clockwise, counterclockwise, third (if there is a 3rd adjacent)
int junctions[7][3] = { { 4, 6, -1 }, { 5, 6, -1 }, { 6, 3, -1 }, { 2, 5, -1 }, { 5, 0, -1 }, { 3, 4, 1 }, { 0, 2, 1 } };
int routes[4][8] = {  // maximum routes, maximum stops in a route. last reserved for length
  { -1, -1, -1, -1, -1, -1, -1, 0 },
  { -1, -1, -1, -1, -1, -1, -1, 0 },
  { -1, -1, -1, -1, -1, -1, -1, 0 },
  { -1, -1, -1, -1, -1, -1, -1, 0 }
};
int routeNum = 0;
int routeIndex = 0;
int cwccw = 1;  // clockwise/counter clockwise

// route calculation
bool visited[7];
int path[7];
int path_len = 0;
int shortest = 10;

// junction tracking
int sourceJunction = -1;
int destinationJunction = 0;
int endFlag = 0;

int state = 256;

// move forward/back
void mobotDrive(int direction, int speed) {
  digitalWrite(leftMotorPhase, direction);
  analogWrite(leftMotorPWM, speed);

  digitalWrite(rightMotorPhase, !direction);
  analogWrite(rightMotorPWM, speed);
}

// stop
void mobotStop() {
  digitalWrite(leftMotorPhase, 0);
  analogWrite(leftMotorPWM, 0);

  digitalWrite(rightMotorPhase, 0);
  analogWrite(rightMotorPWM, 0);
}

// turn
void mobotTurn(int direction, int outsidePWM, int insidePWM) {
  digitalWrite(leftMotorPhase, 1);
  digitalWrite(rightMotorPhase, 0);
  if (direction == 0) {
    analogWrite(leftMotorPWM, insidePWM);
    analogWrite(rightMotorPWM, outsidePWM);
  } else {
    analogWrite(leftMotorPWM, outsidePWM);
    analogWrite(rightMotorPWM, insidePWM);
  }
}

// spin in place
void mobotSpin(int degrees, int pwm, int direction) {
  distance = 3.141 * 0.25 * wheelbase;
  spinTime = int((distance / (pwm * pwmScale)) * 875);
  spinTime *= (degrees / 90);

  digitalWrite(leftMotorPhase, direction);
  analogWrite(leftMotorPWM, pwm);
  digitalWrite(rightMotorPhase, direction);
  analogWrite(rightMotorPWM, pwm);

  delay(spinTime);
}

// junction logic
void junction() {
  mobotDrive(1, desiredPWM);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(80);
  mobotStop();
  //delay(800);
  // start (facing 0 from 4)
  if (sourceJunction == -1) {
    sourceJunction = 0;
    destinationJunction = postArrivedAndGetNextTarget(GROUP_NO, 0);
    bestRoute(sourceJunction, destinationJunction);
  } else {
    // increment along route, update sourceJunction to new junction
    routeIndex += 1;
    sourceJunction = routes[routeNum][routeIndex];

    if (sourceJunction == destinationJunction) {  // path complete
      if (endFlag) {
        if (sourceJunction == 6) {
          routeIndex = 0;
          destinationJunction = 1;
          bestRoute(sourceJunction, destinationJunction);
        } else if (sourceJunction == 1) {
          Serial.println("ending");
          endSequence();
        }
      } else {
        // communicate with server, get new destination
        routeIndex = 0;
        destinationJunction = postArrivedAndGetNextTarget(GROUP_NO, sourceJunction);
        if (sourceJunction == destinationJunction) {
          destinationJunction = postArrivedAndGetNextTarget(GROUP_NO, sourceJunction);
        }

        if (destinationJunction == 7) {
          destinationJunction = 6;
          endFlag = 1;
        }

        bestRoute(sourceJunction, destinationJunction);
      }
    }
  }

  // turning to/from 1
  if (routes[routeNum][routeIndex + 1] == 1) {  // to
    mobotTurn(!cwccw, desiredPWM, 0);
    while (analogRead(osJunctionPins[!cwccw]) >= 400);
    while (analogRead(osLinePins[1]) > 700);
    cwccw = 6 - sourceJunction;
    mobotStop();
  } else if ((sourceJunction == 5 || sourceJunction == 6) && routes[routeNum][routeIndex - 1] == 1) {  // from
    cwccw = (routes[routeNum][routeIndex + 1] == junctions[sourceJunction][1]);
    mobotSpin(90, desiredPWM, !cwccw);
    while (analogRead(osJunctionPins[!cwccw]) <= 400);
    while (analogRead(osLinePins[1]) > 700);
    mobotStop();
  }

  // flip direction if facing wrong way
  if (routes[routeNum][routeIndex + 1] == junctions[sourceJunction][!cwccw]) {
    mobotSpin(180, desiredPWM, cwccw);
    while (analogRead(osJunctionPins[cwccw]) > 700);
    mobotStop();
    cwccw = !cwccw;
  }

  Serial.print(cwccw);
  Serial.print("...");
  Serial.print(sourceJunction);
  Serial.print(",");
  Serial.print(routes[routeNum][routeIndex + 1]);
  Serial.print("...");
  Serial.println(routes[routeNum][routeIndex + 2]);
  // drive and mark lastTime for PID
  mobotDrive(1, desiredPWM);
  digitalWrite(LED_BUILTIN, LOW);
  lastTime = millis();
}

// find all routes from source to destination
void findRoutes(int current, int destination) {
  visited[current] = true;
  path[path_len++] = current;

  if (current == destination) {  // route found
    for (int i = 0; i < path_len; i++) {
      routes[routeNum][i] = path[i];
      routes[routeNum][7] += 1;
    }
    routeNum += 1;
  } else {
    for (int i = 0; i < 3; i++) {
      int neighbor = junctions[current][i];
      if (neighbor == -1) continue;

      if (!visited[neighbor]) {
        findRoutes(neighbor, destination);
      }
    }
  }
  // backtrack
  path_len--;
  visited[current] = false;
}

void bestRoute(int src, int dest) {
  routeNum = 0;
  findRoutes(src, dest);
  shortest = 10;
  for (int i = 0; i < 4; i++) {
    if ((routes[i][7] < shortest) && (routes[i][7] > 0)) {
      shortest = routes[i][7];
      routeNum = i;
    }
    routes[i][7] = 0;
  }
}


// determine action
int lineFollowPoll() {
  // correct readings (white = high, black = low)
  lineAnalogValues[0] = 4095 - analogRead(osLinePins[0]);
  lineAnalogValues[1] = 4095 - analogRead(osLinePins[1]);
  lineAnalogValues[2] = 4095 - analogRead(osLinePins[2]);
  junctionAnalogValues[0] = 4095 - analogRead(osJunctionPins[0]);
  junctionAnalogValues[1] = 4095 - analogRead(osJunctionPins[1]);

  sum = junctionAnalogValues[0] + lineAnalogValues[0] + lineAnalogValues[1] + lineAnalogValues[2] + junctionAnalogValues[1] + 1;
  average = (sum - lineAnalogValues[1]) / 4;  // average of side optical sensors

  // decision. may need to adjust comparison values dependent on light
  if ((average >= 2800) && (lineAnalogValues[1] > 3700)) {  // at junction
    state = 3000;
  } else if (lineAnalogValues[1] > 3600) {  // middle sensor on line, drive forward
    state = 3001;
  } else {  // PID
    weight = (-2.5 * junctionAnalogValues[0] - 1.0 * lineAnalogValues[0] + 0.0 * lineAnalogValues[1] + 1.0 * lineAnalogValues[2] + 2.5 * junctionAnalogValues[1]) / sum;
    error = 0.0 - weight;
    now = millis();
    dt = (now - lastTime);
    derivative = (error - lastError) / (dt);
    integral += error * (dt / 1000.0);
    lastTime = now;
    lastError = error;

    state = int(error * Kp) + int(derivative * Kd) + int(integral * Ki);
  }

  return state;
}

// take action after retrieving decision
void lineFollowAction(int pwm) {
  if (pwm == 3000) {  // junction
    junction();
    delay(2);
  } else if (pwm == 3001) {  // straight
    mobotDrive(1, desiredPWM);
    delay(2);
  } else {  // turn dependent on location of line along sensors
    mobotTurn(0, constrain(desiredPWM + pwm, 0, 255), constrain(desiredPWM - pwm, 0, 255));
    delay(2);
  }
}

void endSequence() {
  mobotTurn(0, desiredPWM - 7, desiredPWM);
  double proximity = 400;
  double response = 0;

  while (proximity > 5.0) {
    digitalWrite(dsTriggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(dsTriggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(dsTriggerPin, LOW);

    response = pulseIn(dsEchoPin, HIGH);
    proximity = (response * 0.017f);
    if (response == 0) {
      proximity = 10;
    }

    delay(20);
  }

  mobotStop();
  postArrivedAndGetNextTarget(GROUP_NO, 5);
  while (1) {
    digitalWrite(LED_BUILTIN, HIGH);
    delay(500);
    digitalWrite(LED_BUILTIN, LOW);
    delay(500);
  }
}

void setup() {
  // initialize pins
  pinMode(leftMotorPhase, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPhase, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  pinMode(osLinePins[0], INPUT);
  pinMode(osLinePins[1], INPUT);
  pinMode(osLinePins[2], INPUT);
  pinMode(osJunctionPins[0], INPUT);
  pinMode(osJunctionPins[1], INPUT);

  pinMode(dsTriggerPin, OUTPUT);
  pinMode(dsEchoPin, INPUT);

  delay(100);

  Serial.begin(9600);
  Serial.println();
  mobotStop();
  wifiConnectBlocking();

  lastTime = millis();
}
double response;
double proximity;
void loop() {
  /*
  digitalWrite(dsTriggerPin, LOW);
  delayMicroseconds(2);
  digitalWrite(dsTriggerPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(dsTriggerPin, LOW);

  response = pulseIn(dsEchoPin, HIGH);
  proximity = (response * 0.017f);
  Serial.print(response);
  Serial.print("...");
  Serial.println(proximity);
  delay(50);
  */
  lineFollowAction(lineFollowPoll());
}
