int leftMotorPhase = 20;
int leftMotorPWM = 39;
int rightMotorPhase = 38;
int rightMotorPWM = 37;

int osLinePins[3] = {7,6,5};
int osJunctionPins[2] = {4,15};

int lineAnalogValues[3] = {0,0,0};
int junctionAnalogValues[2] = {0,0};

int wheelbase = 136; // mm

int desiredPWM = 255; // 0-255

int junctions[7][3] = {{4,6,-1},{5,6,-1},{3,6,-1},{2,5,-1},{0,5,-1},{1,3,4},{1,0,2}};
int routes[4][7] = { // maximum routes, maximum stops in a route
  {-1,-1,-1,-1,-1,-1,-1},
  {-1,-1,-1,-1,-1,-1,-1},
  {-1,-1,-1,-1,-1,-1,-1},
  {-1,-1,-1,-1,-1,-1,-1}
};
int routeNum = 0;

bool visited[7];
int path[7];
int path_len = 0;

int sourceJunction = -1;
int prevJunction = 4;

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

// turn left
void mobotTurnLeft(int outsidePWM, int insidePWM) {
  digitalWrite(leftMotorPhase, 1);
  analogWrite(leftMotorPWM, insidePWM);
  digitalWrite(rightMotorPhase, 0);
  analogWrite(rightMotorPWM, outsidePWM);
}

// turn right
void mobotTurnRight(int outsidePWM, int insidePWM) {
  digitalWrite(leftMotorPhase, 1);
  analogWrite(leftMotorPWM, outsidePWM);
  digitalWrite(rightMotorPhase, 0);
  analogWrite(rightMotorPWM, insidePWM);
}

void junction() {
    // communicate position with server
    // if destination, find new destination
    // if junction in route, continue route
    if (sourceJunction == -1) {
      sourceJunction = 0;
    } 
    //findRoutes(sourceJunction,destinationJunction);
    routeNum = 0;

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
  Serial.println(routeNum);
  // backtrack
  path_len--;
  visited[current] = false;
}

int lineFollowPoll() {
  float outer,middle = 0;

  int lowerBound = 190;
  int upperBound = 1500;
  lineAnalogValues[0] = analogRead(osLinePins[0]);
  lineAnalogValues[1] = analogRead(osLinePins[1]);
  lineAnalogValues[2] = analogRead(osLinePins[2]);
  junctionAnalogValues[0] = analogRead(osJunctionPins[0]);
  junctionAnalogValues[1] = analogRead(osJunctionPins[1]);

  if ((junctionAnalogValues[0] < 500) && (junctionAnalogValues[2] < 500)) { // at junction
    state = 257;
  } else if ((lineAnalogValues[0] > upperBound) && (lineAnalogValues[2] > upperBound)) { // straight line
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
    if (middle > (upperBound - lowerBound)) {middle = upperBound - lowerBound;}
    if (outer < (upperBound - lowerBound)) {
      state = int(((float)desiredPWM/((float)upperBound-(float)lowerBound))*(middle-outer)); // scale based off desired max wheelspeed
      state = desiredPWM - state + int((float)desiredPWM/5);
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
    delay(5);
    
  } else if (pwm == 257) { // junction, change later to stop/continue base on anticipated action, and communicate position with server
    junction();
    delay(5);

  } else if (lineAnalogValues[0] < lineAnalogValues[2]) { // white line left biased, turn left
    mobotTurnLeft(desiredPWM,pwm);
    delay(5);
    
  } else if (lineAnalogValues[0] > lineAnalogValues[2]) { // white line right biased, turn right
    mobotTurnRight(desiredPWM,pwm);
    delay(5);
    
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
  mobotDrive(1,255);
}

void loop() {
  //lineFollowPoll();
  lineFollowAction(lineFollowPoll());
  
}
