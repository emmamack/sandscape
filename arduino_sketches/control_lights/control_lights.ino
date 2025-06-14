#include <Adafruit_NeoPixel.h>
#include <Adafruit_MCP23X17.h>

#ifdef __AVR__
 #include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#define LED_COUNT 284
#define LED_PIN 6

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_RGBW + NEO_KHZ400);

// ~~~~~~~~~~ Sensor stuff ~~~~~~~~~~~~~~~~~~~~~ //

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
//int beforeTrapezoidProfile[18] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 28, 56, 84, 112, 140, 168, 196, 224, 252};
int beforeTrapezoidProfile[18] = {14, 28, 42, 56, 70, 84, 98, 112, 126, 140, 154, 168, 182, 196, 210, 224, 238, 252};

//int afterTrapezoidProfile[18] = {252, 224, 196, 168, 140, 112, 84, 56, 28, 0, 0, 0, 0, 0, 0, 0, 0};
int afterTrapezoidProfile[18] = {252, 238, 224, 210, 196, 182, 168, 154, 140, 126, 112, 98, 84, 70, 56, 42, 28, 14};

Adafruit_MCP23X17 mcp;


void setup() {
  Serial.begin(9600);
  
  strip.begin();           // INITIALIZE NeoPixel strip object (REQUIRED)
  strip.show();            // Turn OFF all pixels ASAP
  strip.setBrightness(60); // Set BRIGHTNESS (max = 255)

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
}

// Fill strip pixels one after another with a color. Strip is NOT cleared
// first; anything there will be covered pixel by pixel. Pass in color
// (as a single 'packed' 32-bit value, which you can get by calling
// strip.Color(red, green, blue) as shown in the loop() function above),
// and a delay time (in milliseconds) between pixels.
void colorWipe(uint32_t color, int wait) {
  for(int i=0; i<strip.numPixels(); i++) { // For each pixel in strip...
    strip.setPixelColor(i, color);         //  Set pixel's color (in RAM)
    strip.show();                          //  Update strip to match
    readTouchSensors();
    delay(wait);                           //  Pause for a moment
  }
}

// Theater-marquee-style chasing lights. Pass in a color (32-bit value,
// a la strip.Color(r,g,b) as mentioned above), and a delay time (in ms)
// between frames.
void theaterChase(uint32_t color, int wait) {
  for(int a=0; a<10; a++) {  // Repeat 10 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in steps of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show(); // Update strip with new contents
      delay(wait);  // Pause for a moment
    }
  }
}

// Rainbow cycle along whole strip. Pass delay time (in ms) between frames.
void rainbow(int wait) {
  // Hue of first pixel runs 5 complete loops through the color wheel.
  // Color wheel has a range of 65536 but it's OK if we roll over, so
  // just count from 0 to 5*65536. Adding 256 to firstPixelHue each time
  // means we'll make 5*65536/256 = 1280 passes through this loop:
  for(long firstPixelHue = 0; firstPixelHue < 5*65536; firstPixelHue += 256) {
    // strip.rainbow() can take a single argument (first pixel hue) or
    // optionally a few extras: number of rainbow repetitions (default 1),
    // saturation and value (brightness) (both 0-255, similar to the
    // ColorHSV() function, default 255), and a true/false flag for whether
    // to apply gamma correction to provide 'truer' colors (default true).
    strip.rainbow(firstPixelHue);
    // Above line is equivalent to:
    // strip.rainbow(firstPixelHue, 1, 255, 255, true);
    strip.show(); // Update strip with new contents
    delay(wait);  // Pause for a moment
  }
}

// Rainbow-enhanced theater marquee. Pass delay time (in ms) between frames.
void theaterChaseRainbow(int wait) {
  int firstPixelHue = 0;     // First pixel starts at red (hue 0)
  for(int a=0; a<30; a++) {  // Repeat 30 times...
    for(int b=0; b<3; b++) { //  'b' counts from 0 to 2...
      strip.clear();         //   Set all pixels in RAM to 0 (off)
      // 'c' counts up from 'b' to end of strip in increments of 3...
      for(int c=b; c<strip.numPixels(); c += 3) {
        // hue of pixel 'c' is offset by an amount to make one full
        // revolution of the color wheel (range 65536) along the length
        // of the strip (strip.numPixels() steps):
        int      hue   = firstPixelHue + c * 65536L / strip.numPixels();
        uint32_t color = strip.gamma32(strip.ColorHSV(hue)); // hue -> RGB
        strip.setPixelColor(c, color); // Set pixel 'c' to value 'color'
      }
      strip.show();                // Update strip with new contents
      delay(wait);                 // Pause for a moment
      firstPixelHue += 65536 / 90; // One cycle of color wheel over 90 frames
    }
  }
}

void  touchResponseBlock(int wait) {
  int base_color = strip.Color( 0, 0, 255, 0); // blue
  readTouchSensors();
  int touchSensorReading[18] = {!res15, !res0, !res1, !res2, !res3, !res4, !res5, !res6, !res7, !res8, !res9, !res10, !res11, !res12, !res13, !res14, !res15, !res0}; // two too long because of the looparound check

  int curr_pixel = 0;
  for(int s=1; s<17; s++) {
//    if(touchSensorReading[s]==1) {
//       strip.fill(strip.Color(0, 0, 255, 255), curr_pixel, sectionSizes[s]);

//      int white_amount = 0;
//      for(int i=curr_pixel; i<curr_pixel+9; i++) {
//        strip.setPixelColor(i, strip.Color(0, 0, 255, white_amount));
//        white_amount += 28;
//      }
//      for(int i=curr_pixel+9; i<curr_pixel+18; i++) {
//        strip.setPixelColor(i, strip.Color(0, 0, 255, white_amount));
//        white_amount -= 28;
//      }
//    } else {
//      strip.fill(base_color, curr_pixel, sectionSizes[s]);
//    }

    for(int i=0; i<18; i++) {
      int white_amount = (touchSensorReading[s+1] * beforeTrapezoidProfile[i]) + 
                         (touchSensorReading[s] * 255) +
                         (touchSensorReading[s-1] * afterTrapezoidProfile[i]);
      if (white_amount > 255) white_amount = 255;
//      Serial.println(white_amount);
      strip.setPixelColor(curr_pixel+i, strip.Color(0, 0, 255, white_amount));
    }

    curr_pixel += sectionSizes[s];
  }
  
  strip.show();
  delay(wait);
}

void loop() {
  touchResponseBlock(10);

//  colorWipe(strip.Color(255,   0,   0), 50); // Red
//  colorWipe(strip.Color(  0, 255,   0), 50); // Green
//  colorWipe(strip.Color(  0,   0, 255), 50); // Blue

//  theaterChase(strip.Color(127, 127, 127), 50); // White, half brightness

//  rainbow(20);             // Flowing rainbow cycle along the whole strip
//  theaterChaseRainbow(50); // Rainbow-enhanced theaterChase variant
}
