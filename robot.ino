int leftMotorPhase = 20;
int leftMotorPWM = 39;
int rightMotorPhase = 38;
int rightMotorPWM = 37;

int osLinePins[3] = {7,6,5};
int osJunctionPins[2] = {4,15};

int lineAnalogValues[3] = {0,0,0};
int junctionAnalogValues[2] = {0,0};

int lowerBound = 190;
int upperBound = 2000;

int wheelbase = 136; // mm

int desiredPWM = 255; // 0-255

int junctions[7][3] = {{4,6,-1},{5,6,-1},{6,3,-1},{2,5,-1},{5,0,-1},{3,4,1},{0,2,1}};
int routes[4][7] = { // maximum routes, maximum stops in a route
  {-1,-1,-1,-1,-1,-1,-1},
  {-1,-1,-1,-1,-1,-1,-1},
  {-1,-1,-1,-1,-1,-1,-1},
  {-1,-1,-1,-1,-1,-1,-1}
};
int routeNum = 0;
int routeIndex = 0;
int cwccw = 0;

bool visited[7];
int path[7];
int path_len = 0;

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
  float pwmScale = 1.75;
  distance = 3.141*0.25*wheelbase;
  time = int((distance/(pwm*pwmScale))*875);
  time *= (degrees / 90);

  digitalWrite(leftMotorPhase, direction);
  analogWrite(leftMotorPWM, pwm);
  digitalWrite(rightMotorPhase, direction);
  analogWrite(rightMotorPWM, pwm);

  delay(time);
}

void junction() {
  int spinDirection;
  // communicate position with server
  // if destination, find new destination
  // if junction in route, continue route
  mobotStop();
  delay(1000);
  if (sourceJunction == -1) {
    sourceJunction = 0;
    findRoutes(sourceJunction,destinationJunction);
    routeNum = 0;
  } else {
    routeIndex += 1;
    sourceJunction = routes[routeNum][routeIndex];

    if (sourceJunction == destinationJunction) { // path complete
      // communicate with server, get new destination
      // findRoutes(sourceJunction,destinationJunction);
      // routeNum = 0;
      // routeIndex = 0;
      delay(50);
      mobotStop();
      while (1) {
      digitalWrite(LED_BUILTIN, HIGH);
      delay(500);
      digitalWrite(LED_BUILTIN, LOW);
      delay(500);
      };
    }
  }

  if (routes[routeNum][routeIndex + 1] == junctions[sourceJunction][cwccw]) {
    if (cwccw) {
      mobotSpin(180,desiredPWM,0);
      while (analogRead(osJunctionPins[!cwccw])  > 300);
      mobotStop();
      cwccw = 0;
    } else if (!cwccw) {
      mobotSpin(180,desiredPWM,0);
      while (analogRead(osJunctionPins[!cwccw])  > 300);
      mobotStop();
      cwccw = 1;
    }
  }

  /*
  Serial.print(routeIndex); Serial.print("---");
  Serial.print(routes[routeNum][routeIndex + 1]); Serial.print("---");
  Serial.println(sourceJunction);
  */

  // turning to/from 1
  if (routes[routeNum][routeIndex + 1] == 1) {
    spinDirection = (cwccw)^(sourceJunction - 5); 
    mobotSpin(90,desiredPWM,!spinDirection);
    while (analogRead(osJunctionPins[!spinDirection]) <= 300);
    while (analogRead(osLinePins[1] > 300));
    mobotStop();
  } else if ((sourceJunction == 5 || sourceJunction == 6) && routes[routeNum][routeIndex - 1] == 1) {
    spinDirection = (!(routes[routeNum][routeIndex + 1] - 2));
    mobotSpin(90,desiredPWM,spinDirection);
    while (analogRead(osJunctionPins[!spinDirection]) <= 300);
    while (analogRead(osLinePins[1] > 300));
    mobotStop();
  }
  
  mobotDrive(1,desiredPWM);
  while(junctionAnalogValues[0] < 250);
}


void findRoutes(int current, int destination) {
  visited[current] = true;
  path[path_len++] = current;

  if (current == destination) {
    for (int i = 0; i < path_len; i++) {
      routes[routeNum][i] = path[i];
    }
    routeNum+=1;
  } else {
    for (int i = 0; i < 3; i++) {
        int neighbor = junctions[current][i];
        if (neighbor == -1) continue;

        if (!visited[neighbor]) {
          findRoutes(neighbor, destination);
        }
    }
  }
  // Serial.println(routeNum);
  // backtrack
  path_len--;
  visited[current] = false;
}

int lineFollowPoll() {
  float outer,middle = 0;

  lineAnalogValues[0] = analogRead(osLinePins[0]);
  lineAnalogValues[1] = analogRead(osLinePins[1]);
  lineAnalogValues[2] = analogRead(osLinePins[2]);
  junctionAnalogValues[0] = analogRead(osJunctionPins[0]);
  junctionAnalogValues[1] = analogRead(osJunctionPins[1]);

  if ((lineAnalogValues[0] < 250) && (lineAnalogValues[2] < 250)) { // at junction
    state = 257;
  } else if ((lineAnalogValues[0] > upperBound) && (lineAnalogValues[2] > upperBound) && (junctionAnalogValues[0] > upperBound) && (junctionAnalogValues[2] > upperBound)) { // straight line
    state = 256; 
  } else {
    // 0-255 pwm for inside wheel
    outer = min(lineAnalogValues[0], lineAnalogValues[2]); // find line bias
    middle = lineAnalogValues[1];
    // normalize reading
    outer -= lowerBound;
    middle -= lowerBound;
    if (outer < 0) {outer = 0;}
    if (middle < 0) {middle = 0;}
    if (middle > (upperBound - lowerBound)) {
      state = 0;
    } else if (outer < (upperBound - lowerBound)) {
      state = int(((float)desiredPWM/((float)upperBound-(float)lowerBound))*(middle-outer)); // scale based off desired max wheelspeed
      state = desiredPWM - state;
    }
  }
  /*
  Serial.print(lineAnalogValues[0]);
  Serial.print("---");
  Serial.print(lineAnalogValues[1]);
  Serial.print("---");
  Serial.print(lineAnalogValues[2]);
  Serial.print("---");
  Serial.println(lineAnalogValues[state]);
  */
  return state;
}

void lineFollowAction(int pwm) {
  if (pwm == 256) { // forward
    mobotDrive(1,desiredPWM);
    //delay(2);
    
  } else if (pwm == 257) { // junction, change later to stop/continue base on anticipated action, and communicate position with server
    if (junctionAnalogValues[0] < 250 && junctionAnalogValues[1] < 250) {
      junction();
    }
    //delay(2);

  } else if (lineAnalogValues[0] < lineAnalogValues[2]) { // white line left biased, turn left
    mobotTurn(0,desiredPWM,pwm);
    while (analogRead(osLinePins[1] > 250));
    //delay(2);
    
  } else if (lineAnalogValues[0] > lineAnalogValues[2]) { // white line right biased, turn right
    mobotTurn(1,desiredPWM,pwm);
    while (analogRead(osLinePins[1] > 250));
    //delay(2);
    
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
  /*
  lowerBound = analogRead(osLinePins[1]) - 20;
  upperBound = (analogRead(osJunctionPins[0]) - lowerBound) / 2;
  */
  Serial.begin(9600);
  mobotDrive(1,255);
  Serial.println();
  //mobotStop();
}

void loop() {
  //lineFollowPoll();
  lineFollowAction(lineFollowPoll());  

}
