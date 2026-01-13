int leftMotorPhase = 20;
int leftMotorPWM = 39;
int rightMotorPhase = 38;
int rightMotorPWM = 37;

int osLinePins[3] = {7,6,5};
int osJunctionPins[2] = {4,15};

int lineAnalogValues[3] = {0,0,0};
int junctionAnalogValues[2] = {0,0};

int lineBools[3] = {0,0,0};
int junctionBools[2] = {0,0};

int lineTruths[8][3] = {
                        {0,0,0}, // all black - move forward
                        {0,1,0}, // line on middle, move forward
                        {1,0,1}, // probably wont encounter, move forward
                        {1,1,1}, // possible junction, move forward unless turning
                        {0,0,1}, // line on right, move right
                        {0,1,1}, // line slightly right, move right
                        {1,0,0}, // line left, move left
                        {1,1,0}, // line slightly left, move left
};

int state;

int wheelbase = 136;

// move forward/back
void mobotDrive(int direction, int speed) {
  digitalWrite(leftMotorPhase, !direction);
  analogWrite(leftMotorPWM, speed);

  digitalWrite(rightMotorPhase, direction);
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
void mobotTurnLeft(int speed, int radius) {
  int rmPWM,rmPhase,lmPWM,lmPhase;
  float leftMotorDistance,rightMotorDistance,ratio;

  rightMotorDistance = (radius + 0.5*wheelbase);
  leftMotorDistance = (radius - 0.5*wheelbase);
  if (leftMotorDistance == 0) {
    ratio = speed/10000;
  } else {
    ratio = leftMotorDistance/rightMotorDistance;
  }
  rmPWM = speed;
  lmPWM = int(speed*ratio);

  rmPhase = 1;
  (ratio < 0) ? lmPhase = 0 : lmPhase = 1;

  digitalWrite(leftMotorPhase, lmPhase);
  analogWrite(leftMotorPWM, lmPWM);
  digitalWrite(rightMotorPhase, rmPhase);
  analogWrite(rightMotorPWM, rmPWM);
}

void mobotTurnRight(int speed, int radius) {
  int rmPWM,rmPhase,lmPWM,lmPhase;
  float leftMotorDistance,rightMotorDistance,ratio;

  rightMotorDistance = (radius - 0.5*wheelbase);
  leftMotorDistance = (radius + 0.5*wheelbase);

  if (rightMotorDistance == 0) {
    ratio = speed/10000;
  } else {
    ratio = rightMotorDistance/leftMotorDistance;
  }

  rmPWM = int(speed*ratio);
  lmPWM = speed;

  (ratio < 0) ? rmPhase = 1 : rmPhase = 0;
  lmPhase = 0;

  digitalWrite(leftMotorPhase, lmPhase);
  analogWrite(leftMotorPWM, lmPWM);
  digitalWrite(rightMotorPhase, rmPhase);
  analogWrite(rightMotorPWM, rmPWM);
}

int lineFollowPoll() {
  lineAnalogValues[0] = analogRead(osLinePins[0]);
  lineAnalogValues[1] = analogRead(osLinePins[1]);
  lineAnalogValues[2] = analogRead(osLinePins[2]);
  junctionAnalogValues[0] = analogRead(osJunctionPins[0]);
  junctionAnalogValues[1] = analogRead(osJunctionPins[1]);

  for (int i = 0; i < 3; i++) {
    if (lineAnalogValues[i] <= 500) {
      lineBools[i] = 1;
    } else {
      lineBools[i] = 0;
    }
  }

  for (int i = 0; i < 8; i++) {
    if (lineTruths[i][0] == lineBools[0] && lineTruths[i][1] == lineBools[1] && lineTruths[i][2] == lineBools[2]) {
      state = i;
    }
  }

  return state;
}

void lineFollowAction(int action) {
  if (action < 4) {
    mobotDrive(1,255);
    delay(5);
    
  } else if (action == 4 || action == 5) {
    mobotTurnRight(255,wheelbase + (action - 4)*wheelbase);
    delay(5);
    
  } else if (action == 6 || action == 7) {
    mobotTurnLeft(255,wheelbase + (action - 6)*wheelbase);
    delay(5);
    
  }
}

void setup() {
  // put your setup code here, to run once:
  pinMode(leftMotorPhase, OUTPUT);
  pinMode(leftMotorPWM, OUTPUT);
  pinMode(rightMotorPhase, OUTPUT);
  pinMode(rightMotorPWM, OUTPUT);

  pinMode(osLinePins[0], INPUT);
  pinMode(osLinePins[1], INPUT);
  pinMode(osLinePins[2], INPUT);
  pinMode(osJunctionPins[0], INPUT);
  pinMode(osJunctionPins[1], INPUT);

  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:

  lineFollowAction(lineFollowPoll());

}
