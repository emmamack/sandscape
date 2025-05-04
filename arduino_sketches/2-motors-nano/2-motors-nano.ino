int stepPinRot = 3;
int dirPinRot = 4;

int enPin = 5;

int stepPinLin = 6;
int dirPinLin = 7;

int motSpeedRot = 500;
int motSpeedLin = 500;
int limitOut = A1;
int limitIn = A2;
int prox = A3;

bool motorLinActivated = false;
bool motorRotActivated = false;

char receivedChar;

int curR = 0;
int curTheta = 0;

int CLOCKWISE = LOW;
int COUNTERCLOCKWISE = HIGH;
int OUT = LOW;
int IN = HIGH;

int x;

void checkSafeties() {
  if (Serial.available() > 0) {
      receivedChar = Serial.read();
      Serial.println(receivedChar);
//      if (receivedChar == 121) {
//        motorLinActivated = true;
//        motorRotActivated = true;
//      }
      if (receivedChar == 110) {
        motorLinActivated = false;
        motorRotActivated = false;
      }
  }

  bool limitOutPressed = false;
  if (analogRead(limitOut) > 512 ) {
    limitOutPressed = true;
  }

  bool limitInPressed = false;
  if (analogRead(limitIn) > 512 ) {
    limitInPressed = true;
  }

  if (limitOutPressed || limitInPressed) {
    motorLinActivated = false;
  }
}

void goToPoint(int r, int theta) {
  // TODO: potential smoothness improvements -
  // sync timing so r and theta end at the same time
  // use flexystepper for smooth acceleration
  // normalize speed between middle and outside

  int deltaR = r - curR;
  
  int deltaTheta = theta - curTheta;
  if (deltaTheta > 4000) {
    deltaTheta = deltaTheta - 8000;
  }
  if (deltaTheta < -4000) {
    deltaTheta = deltaTheta + 8000;
  }

  if (deltaR >= 0) {
    digitalWrite(dirPinLin, OUT);
  }  else {
    digitalWrite(dirPinLin, IN);
  }

  if (deltaTheta >= 0) {
    digitalWrite(dirPinRot, COUNTERCLOCKWISE);
  }  else {
    digitalWrite(dirPinRot, CLOCKWISE);
  }

  deltaR = abs(deltaR);
  deltaTheta = abs(deltaTheta);

  int counterR = 0;
  int counterTheta = 0;

  motorLinActivated = true;
  motorRotActivated = true;

  while (counterR < deltaR || counterTheta < deltaTheta) {
    checkSafeties();
    
    if (motorLinActivated) {
      digitalWrite(stepPinLin, HIGH);
      delayMicroseconds(motSpeedLin);
      digitalWrite(stepPinLin, LOW);
      delayMicroseconds(motSpeedLin);
      counterR++;
    }

    if (counterR >= deltaR) {
      motorLinActivated = false;
    }
  
    if (motorRotActivated) {
      digitalWrite(stepPinRot, HIGH);
      delayMicroseconds(motSpeedRot);
      digitalWrite(stepPinRot, LOW);
      delayMicroseconds(motSpeedRot);
      counterTheta++;
    }

    if (counterTheta >= deltaTheta) {
      motorRotActivated = false;
    }
  }

  curR = r;
  curTheta = theta;

  Serial.print("Went to "); Serial.print(r); Serial.print(", "); Serial.println(theta);
}

void goToHome() {
//  while (1) {
//    if (Serial.available() > 0) {
//      receivedChar = Serial.read();
//      Serial.println(receivedChar);
//      if (receivedChar == 121) {
//        break;
//      }
//    }
//  }
  while (analogRead(limitIn) < 512) {
    digitalWrite(stepPinLin, HIGH);
    delayMicroseconds(motSpeedLin);
    digitalWrite(stepPinLin, LOW);
    delayMicroseconds(motSpeedLin);
  }

  delayMicroseconds(2000);
  digitalWrite(dirPinLin, OUT);
  for (int i=0; i<50; i++) {
    digitalWrite(stepPinLin, HIGH);
    delayMicroseconds(motSpeedLin);
    digitalWrite(stepPinLin, LOW);
    delayMicroseconds(motSpeedLin);
  }

  while (analogRead(prox) < 50) {
    digitalWrite(stepPinRot, HIGH);
    delayMicroseconds(motSpeedRot*2);
    digitalWrite(stepPinRot, LOW);
    delayMicroseconds(motSpeedRot*2);
  }

  Serial.println("Homing complete.");
}

void setup() {
  pinMode(stepPinRot, OUTPUT);
  pinMode(dirPinRot, OUTPUT);
  digitalWrite(dirPinRot, CLOCKWISE);
  
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);

  pinMode(stepPinLin, OUTPUT);
  pinMode(dirPinLin, OUTPUT);
  digitalWrite(dirPinLin, IN);

  Serial.begin(9600);

//  goToHome();
}

void loop() {
  char buff[8];
  if (Serial.readBytes(buff, 8) == 8) {
    int cmdR = (buff[0]-48)*1000 + (buff[1]-48)*100 + (buff[2]-48)*10 + (buff[3]-48);
    int cmdTheta = (buff[4]-48)*1000 + (buff[5]-48)*100 + (buff[6]-48)*10 + (buff[7]-48);

    if (cmdR == 0 && cmdTheta == 0) {
      goToHome();
    } else {
      goToPoint(cmdR, cmdTheta);
    }
  }
  // TOD): listen to outward limit switch even in homing, for safety
}
