#include <Adafruit_MCP23X17.h>

#define SENSOR_PIN_0 0
#define SENSOR_PIN_1 1
#define SENSOR_PIN_2 2
#define SENSOR_PIN_3 3
#define SENSOR_PIN_4 4
#define SENSOR_PIN_5 5
#define SENSOR_PIN_6 6
#define SENSOR_PIN_7 7
#define SENSOR_PIN_8 8
#define SENSOR_PIN_9 9
#define SENSOR_PIN_10 10
#define SENSOR_PIN_11 11
#define SENSOR_PIN_12 12
#define SENSOR_PIN_13 13
#define SENSOR_PIN_14 14
#define SENSOR_PIN_15 15

int res0 = 0;
int res1 = 1;
int res2 = 2;
int res3 = 3;
int res4 = 4;
int res5 = 5;
int res6 = 6;
int res7 = 7;
int res8 = 8;
int res9 = 9;
int res10 = 10;
int res11 = 11;
int res12 = 12;
int res13 = 13;
int res14 = 14;
int res15 = 15;

Adafruit_MCP23X17 mcp;

void setup() {
  Serial.begin(9600);

  if (!mcp.begin_I2C()) {
    Serial.println("Error.");
    while (1);
  }

  mcp.pinMode(SENSOR_PIN_0, INPUT);
  mcp.pinMode(SENSOR_PIN_1, INPUT);
  mcp.pinMode(SENSOR_PIN_2, INPUT);
  mcp.pinMode(SENSOR_PIN_3, INPUT);
  mcp.pinMode(SENSOR_PIN_4, INPUT);
  mcp.pinMode(SENSOR_PIN_5, INPUT);
  mcp.pinMode(SENSOR_PIN_6, INPUT);
  mcp.pinMode(SENSOR_PIN_7, INPUT);
  mcp.pinMode(SENSOR_PIN_8, INPUT);
  mcp.pinMode(SENSOR_PIN_9, INPUT);
  mcp.pinMode(SENSOR_PIN_10, INPUT);
  mcp.pinMode(SENSOR_PIN_11, INPUT);
  mcp.pinMode(SENSOR_PIN_12, INPUT);
  mcp.pinMode(SENSOR_PIN_13, INPUT);
  mcp.pinMode(SENSOR_PIN_14, INPUT);
  mcp.pinMode(SENSOR_PIN_15, INPUT);

  Serial.println("Looping...");
}

void loop() {
  res0 = mcp.digitalRead(SENSOR_PIN_0);
  res1 = mcp.digitalRead(SENSOR_PIN_1);
  res2 = mcp.digitalRead(SENSOR_PIN_2);
  res3 = mcp.digitalRead(SENSOR_PIN_3);
  res4 = mcp.digitalRead(SENSOR_PIN_4);
  res5 = mcp.digitalRead(SENSOR_PIN_5);
  res6 = mcp.digitalRead(SENSOR_PIN_6);
  res7 = mcp.digitalRead(SENSOR_PIN_7);
  res8 = mcp.digitalRead(SENSOR_PIN_8);
  res9 = mcp.digitalRead(SENSOR_PIN_9);
  res10 = mcp.digitalRead(SENSOR_PIN_10);
  res11 = mcp.digitalRead(SENSOR_PIN_11);
  res12 = mcp.digitalRead(SENSOR_PIN_12);
  res13 = mcp.digitalRead(SENSOR_PIN_13);
  res14 = mcp.digitalRead(SENSOR_PIN_14);
  res15 = mcp.digitalRead(SENSOR_PIN_15);
  
  Serial.print("["); 
  Serial.print(res0); Serial.print(","); Serial.print(res1); Serial.print(","); 
  Serial.print(res2); Serial.print(","); Serial.print(res3); Serial.print(","); 
  Serial.print(res4); Serial.print(","); Serial.print(res5); Serial.print(","); 
  Serial.print(res6); Serial.print(","); Serial.print(res7); Serial.print(","); 
  Serial.print(res8); Serial.print(","); Serial.print(res9); Serial.print(","); 
  Serial.print(res10); Serial.print(","); Serial.print(res11); Serial.print(","); 
  Serial.print(res12); Serial.print(","); Serial.print(res13); Serial.print(","); 
  Serial.print(res14); Serial.print(","); Serial.print(res15); Serial.print(","); 
  Serial.println("]");
  delay(10);
}
