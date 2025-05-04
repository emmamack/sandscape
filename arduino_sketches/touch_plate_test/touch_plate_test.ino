int sensePin = A0; 
                   // outside leads to ground and +5V
int touchReadout = 0;  // variable to store the value read

void setup() {
  Serial.begin(9600);           //  setup serial
}

void loop() {
  touchReadout = analogRead(sensePin);  // read the input pin
//  if (touchReadout > 500) { 
//    Serial.println("no touch detected"); 
//  } else {
//    Serial.println("touch detected"); 
//  }
  Serial.println(touchReadout);
  delay(10);
}
