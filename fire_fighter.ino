const int irSensor = 2;
#define AIN1 4
#define BIN1 6
#define AIN2 3
#define BIN2 7
#define PWMA 9
#define PWMB 10

int P, D, I, previousError, PIDvalue, error;
int lsp, rsp;
int lfSpeed = 60;

float Kp = 0.1;
float Kd = 1.5;
float Ki = 0;

int onLine = 1;
int minValues[8], maxValues[8], threshold[8], sensorValue[8];
int Flame_minValues[5], Flame_maxValues[5], Flame_threshold[5], Flame_sensorValue[5];
int Obs_minValues[5], Obs_maxValues[5], Obs_threshold[5], Obs_sensorValue[5];
int maxFlameValue;
bool flameDetected = 0;
bool obsDetected = 0;
bool lineDetected = 0;
int steering = 0;
int speed;
int obsZone;
int lineSensorTriggered;

void setup() {

  Serial.begin(9600);
  pinMode(11, INPUT_PULLUP);
  pinMode(12, INPUT_PULLUP);
  pinMode(13, OUTPUT);

  pinMode(irSensor, OUTPUT);
  pinMode(5, OUTPUT);    
  digitalWrite(5, LOW);  
}


void loop() {
  while (digitalRead(11)) {}
  delay(1000);
  calibrateLine();
  while (digitalRead(12)) {}
  delay(1000);

  while (1) {
    readLine();
    readFlame();
    readObs();
    if (obsDetected) {
      if (obsZone > 2) {
        botreverse();
        delay(200);
        botLeftTurn();
      } else {
        botreverse();
        delay(200);
        botRightTurn();
      }
      botStop();
    } else if (lineDetected) {
      if (lineSensorTriggered > 6) {
        botreverse();
        delay(200);
        botLeftTurn();
      } else {
        botreverse();
        delay(200);
        botRightTurn();
      }

    } else if (flameDetected) {
      botStop();
      locateFlame();
      botStop();
    } else {
      botForward();
    }
  }
}


void linefollow() {
  error = 500 - sensorValue[5];
  P = error;
  I = I + error;
  D = error - previousError;

  PIDvalue = (Kp * P) + (Ki * I) + (Kd * D);
  previousError = error;

  lsp = lfSpeed - PIDvalue;
  rsp = lfSpeed + PIDvalue;

  if (lsp > 255) {
    lsp = 255;
  }
  if (lsp < 0) {
    lsp = 0;
  }
  if (rsp > 255) {
    rsp = 255;
  }
  if (rsp < 0) {
    rsp = 0;
  }
  motor1run(lsp);
  motor2run(rsp);
}



void calibrateLine() {
  for (int i = 5; i < 8; i++) {
    minValues[i] = analogRead(i);
    maxValues[i] = analogRead(i);
  }

  for (int i = 0; i < 3000; i++) {
    motor1run(50);
    motor2run(-50);

    for (int i = 5; i < 8; i++) {
      if (analogRead(i) < minValues[i]) {
        minValues[i] = analogRead(i);
      }
      if (analogRead(i) > maxValues[i]) {
        maxValues[i] = analogRead(i);
      }
    }
  }

  for (int i = 5; i < 8; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
    Serial.print(threshold[i]);
    Serial.print(" ");
  }
  Serial.println();
  motor1run(0);
  motor2run(0);
}

void readLine() {

  lineDetected = 0;
  for (int i = 5; i < 8; i++) {
    sensorValue[i] = map(analogRead(i), minValues[i], maxValues[i], 0, 1000);
    sensorValue[i] = constrain(sensorValue[i], 0, 1000);
    if (sensorValue[i] > 850) {
      lineDetected = 1;
      lineSensorTriggered = i;
    }
  }
}

void readFlame() {
  flameDetected = 0;
  int flameCount = 0;
  digitalWrite(irSensor, LOW);
  delay(5);
  for (int j = 0; j < 3; j++) {
    for (int i = 0; i < 5; i++) {
      Flame_sensorValue[i] = analogRead(i);
      if (Flame_sensorValue[i] > 100) {
        flameCount++;
      }
    }
  }
  if (flameCount > 2) flameDetected = 1;
}

void readObs() {
  obsDetected = 0;
  digitalWrite(irSensor, HIGH);
  delay(5);
  for (int i = 0; i < 5; i++) {
    Obs_sensorValue[i] = analogRead(i) - Flame_sensorValue[i];
    if (Obs_sensorValue[i] > 300) {
      obsDetected = 1;
      obsZone = i;
    }
  }
  digitalWrite(irSensor, LOW);
}

//--------Function to run Motor 1-----------------
void motor1run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 0);
    analogWrite(PWMA, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(AIN1, 0);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, abs(motorSpeed));
  } else {
    digitalWrite(AIN1, 1);
    digitalWrite(AIN2, 1);
    analogWrite(PWMA, 0);
  }
}

//--------Function to run Motor 2-----------------
void motor2run(int motorSpeed) {
  motorSpeed = constrain(motorSpeed, -255, 255);
  if (motorSpeed > 0) {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 0);
    analogWrite(PWMB, motorSpeed);
  } else if (motorSpeed < 0) {
    digitalWrite(BIN1, 0);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, abs(motorSpeed));
  } else {
    digitalWrite(BIN1, 1);
    digitalWrite(BIN2, 1);
    analogWrite(PWMB, 0);
  }
}

void botForward() {
  motor1run(50);
  motor2run(50);
}
void botStop() {
  motor1run(0);
  motor2run(0);
}
void botreverse() {
  motor1run(-50);
  motor2run(-50);
}
void botRightTurn() {
  motor1run(50);
  motor2run(-50);
  delay(200);
}
void botLeftTurn() {
  motor1run(-50);
  motor2run(50);
  delay(200);
}
void locateFlame() {
  for (int i = 0; i < 3000; i++) {
    steering = (analogRead(0) + analogRead(1) - analogRead(3) - analogRead(4));
    steering = constrain(steering, -60, 60);
    motor2run(steering);
    motor1run(-1 * steering);
  }

  motor1run(-40);
  motor2run(40);
  delay(500);
  botStop();
  delay(500);
  maxFlameValue = analogRead(2);
  for (int j = 0; j < 5000; j++) {
    motor1run(40);
    motor2run(-40);
    if (analogRead(2) > maxFlameValue) maxFlameValue = analogRead(2);
  }
  botStop();
  delay(500);
  for (int j = 0; j < 5000; j++) {
    motor1run(-40);
    motor2run(40);
    if (analogRead(2) > maxFlameValue) maxFlameValue = analogRead(2);
  }
  botStop();
  delay(500);
  while (analogRead(2) < maxFlameValue - 10) {
    motor1run(40);
    motor2run(-40);
  }
  botStop();
  delay(500);
  readLine();
  while (lineDetected == 0) {
    readLine();
    steering = 2 * (analogRead(1) - analogRead(3));
    motor1run(50);
    motor2run(50);
  }
  botStop();
  delay(1000);
  botForward();
  delay(300);
  botStop();
  digitalWrite(5, HIGH);
  delay(3000);
  digitalWrite(5, LOW);
  botreverse();
  delay(3000);
  botRightTurn();
  botForward();
  delay(500);
}
