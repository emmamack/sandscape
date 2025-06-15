// ~~~~~~~~~~~~~ Lights stuff ~~~~~~~~~~~~~~~~~~~~~~~~ //

#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23X17.h>

#define LED_COUNT 284
#define LED_PIN 6

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGBW + NEO_KHZ400);

long firstPixelHue = 0;

// ~~~~~~~~~~ Touch sensor stuff ~~~~~~~~~~~~~~~~~~~~~ //

#define SENSOR_PIN_0 15
#define SENSOR_PIN_1 14
#define SENSOR_PIN_2 13
#define SENSOR_PIN_3 12
#define SENSOR_PIN_4 11
#define SENSOR_PIN_5 10
#define SENSOR_PIN_6 9
#define SENSOR_PIN_7 8
#define SENSOR_PIN_8 7
#define SENSOR_PIN_9 6
#define SENSOR_PIN_10 5
#define SENSOR_PIN_11 4
#define SENSOR_PIN_12 3
#define SENSOR_PIN_13 2
#define SENSOR_PIN_14 1
#define SENSOR_PIN_15 0

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

int sectionSizes[18] = {17, 18, 18, 18, 17, 18, 18, 18, 17, 18, 18, 18, 17, 18, 18, 18, 17, 18}; // two too long because of the looparound check
int beforeTrapezoidProfile[18] = {14, 28, 42, 56, 70, 84, 98, 112, 126, 140, 154, 168, 182, 196, 210, 224, 238, 252};
int afterTrapezoidProfile[18] = {252, 238, 224, 210, 196, 182, 168, 154, 140, 126, 112, 98, 84, 70, 56, 42, 28, 14};

Adafruit_MCP23X17 mcp;

// ~~~~~~~~~~ Prox sensor stuff ~~~~~~~~~~~~~~~~~~~~~ //

#define PROX_PIN A0

int proxReading = 0;

void setup() {
  Serial.begin(9600);
  
  strip.begin();
  strip.show();
  strip.setBrightness(60);

  if (!mcp.begin_I2C()) {
    Serial.println("I2C error.");
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
}

void readTouchSensors() {
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
}

void sendSensorData() {
  Serial.print(res0);  Serial.print(res1);  Serial.print(res2);  Serial.print(res3);  Serial.print(res4);  Serial.print(res5);  Serial.print(res6);  Serial.print(res7); 
  Serial.print(res8);  Serial.print(res9);  Serial.print(res10); Serial.print(res11); Serial.print(res12); Serial.print(res13); Serial.print(res14); Serial.print(res15);
  Serial.println(proxReading);
}

void readProxSensor() {
  int rawProxReading = analogRead(PROX_PIN);
  if (rawProxReading > 500) {
    proxReading = 1;
  } else {
    proxReading = 0;
  }
}

void touchResponseRainbow(int wait) {
  firstPixelHue += 256;
  if (firstPixelHue >= 5*65536) firstPixelHue = 0;
  
  int touchSensorReading[18] = {!res15, !res0, !res1, !res2, !res3, !res4, !res5, !res6, !res7, !res8, !res9, !res10, !res11, !res12, !res13, !res14, !res15, !res0}; // two too long because of the looparound check

  int curr_pixel = 0;
  for(int s=1; s<17; s++) {
    for(int i=0; i<18; i++) {
      int white_amount = (touchSensorReading[s+1] * beforeTrapezoidProfile[i]) + 
                         (touchSensorReading[s] * 255) +
                         (touchSensorReading[s-1] * afterTrapezoidProfile[i]);
      if (white_amount > 255) white_amount = 255;

      uint16_t rainbow_hue = firstPixelHue + ((curr_pixel+i) * 65536) / LED_COUNT;
      uint32_t rainbow_base_color = strip.ColorHSV(rainbow_hue, 255, 255);
      rainbow_base_color = strip.gamma32(rainbow_base_color);

      uint32_t rainbow_plus_white = ((uint32_t)white_amount << 24) | rainbow_base_color;
      strip.setPixelColor(curr_pixel+i, rainbow_plus_white);
    }

    curr_pixel += sectionSizes[s];
  }
  
  strip.show();
  delay(wait);
}

void loop() {
  readTouchSensors();
  readProxSensor();
  sendSensorData(); 
  touchResponseRainbow(5);
}
