int stepPinRot = 3;
int dirPinRot = 4;

int enPin = 5;

int stepPinLin = 6;
int dirPinLin = 7;

int motSpeedRot = 500;
int motSpeedLin = 500;
int limitOut = A1;
int limitIn = A2;

bool motorLinActivated = false;
bool motorRotActivated = false;

char receivedChar;

int counter = 0;

void setup() {
  pinMode(stepPinRot, OUTPUT);
  pinMode(dirPinRot, OUTPUT);
  digitalWrite(dirPinRot, HIGH);
  
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);

  pinMode(stepPinLin, OUTPUT);
  pinMode(dirPinLin, OUTPUT);
  digitalWrite(dirPinLin, LOW);
  
  pinMode(limitOut, INPUT);

  Serial.begin(9600);

}

void loop() {

  if (Serial.available() > 0) {
      receivedChar = Serial.read();
      Serial.println(receivedChar);
      if (receivedChar == 121) {
        motorLinActivated = true;
        motorRotActivated = true;
      }
      if (receivedChar == 110) {
        motorLinActivated = false;
        motorRotActivated = false;
      }
  }

//  Serial.println(motorLinActivated);

//  if (motorRotActivated) {
//    digitalWrite(stepPinRot, HIGH);
//    delayMicroseconds(motSpeedRot);
//    digitalWrite(stepPinRot, LOW);
//    delayMicroseconds(motSpeedRot);
//    counter = counter +1;
//  }

  if (motorLinActivated) {
    digitalWrite(stepPinLin, HIGH);
    delayMicroseconds(motSpeedLin);
    digitalWrite(stepPinLin, LOW);
    delayMicroseconds(motSpeedLin);
    counter = counter +1;
  }

  bool limitOutPressed = false;
//  Serial.println(analogRead(limitOut));
  if (analogRead(limitOut) > 512 ) {
    limitOutPressed = true;
  }
//  Serial.println(limitOutPressed);

  bool limitInPressed = false;
//  Serial.println(analogRead(limitIn));
  if (analogRead(limitIn) > 512 ) {
    limitInPressed = true;
  }

  if (limitOutPressed || limitInPressed) {
    motorLinActivated = false;
  }

//  Serial.println(counter);
  if (counter >= 13000) {
    motorRotActivated = false;
    motorLinActivated = false;
  }
  

}
