// pin initialization
int leftMotorPhase = 20;
int leftMotorPWM = 39;
int rightMotorPhase = 38;
int rightMotorPWM = 37;

int osLinePins[3] = { 7, 6, 5 };
int osJunctionPins[2] = { 15, 4 };

// optical sensor values
int lineAnalogValues[3] = { 0, 0, 0 };
int junctionAnalogValues[2] = { 0, 0 };

// physical dimensions
int wheelbase = 136;  // mm

// PID control/speed values
// 145, 220, 10, 0
int desiredPWM = 145;  // 0-255
float Kp = 220.0; // 220
float Kd = 10.0; // 25
float Ki = 0.5;
double derivative, integral = 1.0;
double dt = 0;
unsigned long now, lastTime = 0;
float error, lastError = 0.0;

// map, index is junction, clockwise, counterclockwise, third (if there is a 3rd adjacent)
int junctions[7][3] = { { 4, 6, -1 }, { 5, 6, -1 }, { 6, 3, -1 }, { 2, 5, -1 }, { 5, 0, -1 }, { 3, 4, 1 }, { 0, 2, 1 } };
int routes[4][7] = {  // maximum routes, maximum stops in a route
  { -1, -1, -1, -1, -1, -1, -1 },
  { -1, -1, -1, -1, -1, -1, -1 },
  { -1, -1, -1, -1, -1, -1, -1 },
  { -1, -1, -1, -1, -1, -1, -1 }
};
int routeNum = 0;
int routeIndex = 0;
int cwccw = 0; // clockwise/counter clockwise

// route calculation
bool visited[7];
int path[7];
int path_len = 0;

// junction tracking
int sourceJunction = -1;
int destinationJunction = 1;

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
  float distance = 0;
  int time = 0;
  float pwmScale = 1.9;
  distance = 3.141 * 0.25 * wheelbase;
  time = int((distance / (pwm * pwmScale)) * 875);
  time *= (degrees / 90);

  digitalWrite(leftMotorPhase, direction);
  analogWrite(leftMotorPWM, pwm);
  digitalWrite(rightMotorPhase, direction);
  analogWrite(rightMotorPWM, pwm);

  delay(time);
}

// junction logic
void junction() {
  int spinDirection;
  mobotDrive(1,desiredPWM);
  delay(80);
  mobotStop();
  delay(1000);
  // start (facing 0 from 4)
  if (sourceJunction == -1) {
    sourceJunction = 0;
    findRoutes(sourceJunction, destinationJunction);
    routeNum = 3;
  } else {
    // increment along route, update sourceJunction to new junction
    routeIndex += 1;
    sourceJunction = routes[routeNum][routeIndex];
  
    
    if (sourceJunction == destinationJunction) {  // path complete
      // communicate with server, get new destination
      // findRoutes(sourceJunction,destinationJunction);
      // routeNum = 0;
      routeIndex = 0;
      findRoutes(sourceJunction,2);
      delay(50);
      //if (sourceJunction == 2) {
        while (1) {
          digitalWrite(LED_BUILTIN, HIGH);
          delay(500);
          digitalWrite(LED_BUILTIN, LOW);
          delay(500);
        };
      //}
    }
  }

  // flip direction if facing wrong way
  if (routes[routeNum][routeIndex + 1] == junctions[sourceJunction][cwccw]) {
    mobotSpin(180, desiredPWM, 0);
    while (analogRead(osJunctionPins[cwccw]) > 700);
    mobotStop();
    cwccw = !cwccw;
  }

  // turning to/from 1
  if (routes[routeNum][routeIndex + 1] == 1) { // to
    mobotSpin(90, desiredPWM, cwccw);
    while (analogRead(osJunctionPins[cwccw]) <= 400);
    while (analogRead(osLinePins[1] > 700));
    mobotStop();
  } else if ((sourceJunction == 5 || sourceJunction == 6) && routes[routeNum][routeIndex - 1] == 1) { // from
    spinDirection = (!(routes[routeNum][routeIndex + 1] - 2));
    mobotSpin(90, desiredPWM, spinDirection);
    while (analogRead(osJunctionPins[!spinDirection]) <= 400);
    while (analogRead(osLinePins[1] > 700));
    mobotStop();
  }

  // drive and mark lastTime for PID
  mobotDrive(1, desiredPWM);
  lastTime = millis();
}

// find all routes from source to destination
void findRoutes(int current, int destination) {
  visited[current] = true;
  path[path_len++] = current;

  if (current == destination) { // route found
    for (int i = 0; i < path_len; i++) {
      routes[routeNum][i] = path[i];
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

// determine action
int lineFollowPoll() {
  float weight, sum, average = 0;

  // correct readings (white = high, black = low)
  lineAnalogValues[0] = 4095 - analogRead(osLinePins[0]);
  lineAnalogValues[1] = 4095 - analogRead(osLinePins[1]);
  lineAnalogValues[2] = 4095 - analogRead(osLinePins[2]);
  junctionAnalogValues[0] = 4095 - analogRead(osJunctionPins[0]);
  junctionAnalogValues[1] = 4095 - analogRead(osJunctionPins[1]);

  sum = junctionAnalogValues[0] + lineAnalogValues[0] + lineAnalogValues[1] + lineAnalogValues[2] + junctionAnalogValues[1] + 1;
  average = (sum - lineAnalogValues[1]) / 4; // average of side optical sensors

  // decision. may need to adjust comparison values dependent on light
  if ((average >= 2350) && (lineAnalogValues[1] > 3700)) {  // at junction
    state = 3000;
  } else if (lineAnalogValues[1] > 3600) { // middle sensor on line, drive forward
    state = 3001;
  } else { // PID
    weight = (-2.5*junctionAnalogValues[0] - 1.0 * lineAnalogValues[0] + 0.0 * lineAnalogValues[1] + 1.0 * lineAnalogValues[2] + 2.5*junctionAnalogValues[1]) / sum;
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
  } else if (pwm == 3001) { // straight
    mobotDrive(1,desiredPWM);
    delay(2);
  } else { // turn dependent on location of line along sensors
    mobotTurn(0,constrain(desiredPWM + pwm,0,255), constrain(desiredPWM - pwm,0,255));
    delay(2);
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

  delay(100);

  Serial.begin(9600);
  mobotDrive(1, 255);
  Serial.println();
  //mobotStop();
  lastTime = millis();
}

void loop() {
  //lineFollowPoll();
  lineFollowAction(lineFollowPoll());
}
