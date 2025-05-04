int stepPin = 3;          //Define travel stepper motor step pin
int dirPin = 4;           //Define travel stepper motor direction pin
int enPin = 5;
int motSpeed = 1;
int limit1 = A1;
bool motorActivated = true;

void setup() {
  pinMode(stepPin, OUTPUT);                 //Define pins and set direction
  pinMode(dirPin, OUTPUT);
  digitalWrite(dirPin, LOW);
  pinMode(enPin, OUTPUT);
  digitalWrite(enPin, LOW);
  pinMode(limit1, INPUT);

  Serial.begin(9600);           //  setup serial
}

void loop() {
//  motSpeed = map(analogRead(A0),0,1023,50,1);           //Read in potentiometer value from A0, map to a delay between 1 and 50 milliseconds

  if (motorActivated) {
    digitalWrite(stepPin, HIGH);                          //Step the motor with the set delay
    delay(motSpeed);
    digitalWrite(stepPin, LOW);
    delay(motSpeed);
  }

  bool limit1Pressed = false;
  if (analogRead(limit1) > 512 ) {
    limit1Pressed = true;
  }
  Serial.println(limit1Pressed);

  if (limit1Pressed) {
    motorActivated = false;
  }

}
