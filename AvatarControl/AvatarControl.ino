
/***********************************************************
*                         DEFINES                          *
***********************************************************/

#define BL_NEOPIXEL_S  12
#define BR_NEOPIXEL_S  11
#define FL_NEOPIXEL_S  10
#define FR_NEOPIXEL_S  9
#define FAN_1_LED      8
#define FAN_2_LED      7
#define FAN_3_LED      6
#define FAN_4_LED      5
#define FANS_CONTROL   4 
#define AUDIO_IN       A0
#define REMOTE_ON      3
#define REMOTE_OFF     2


/***********************************************************
*                         INCLUDES                         *
***********************************************************/
#include <Adafruit_NeoPixel.h>
#include <math.h>

/***********************************************************
*                      GLOBAL VARS                         *
***********************************************************/
//global vars to go here

/***********************************************************
*                          SETUP                           *
***********************************************************/
void setup(){
  //stuff and foo
  //pin defines
  //initial states
  
}


/***********************************************************
*                          LOOP                            *
***********************************************************/

void loop(){
  //stuff and foo
  //Read state of remote control pins
  //State machine to determine if starting up, shutting down, VU Meter, Attract Loop
}  
void startUp(){
  //Do some fancy power on animation here
  //Enable the computer's power on button
}

void shutDown(){
  //do some fancy power off animation here
  //pull the power button high again momentarily
}

void vuMeter(){
 // Code below is from adafruit amplitie. use parts as needed in other code. Be sure to credit original source.  
  /*
#include <Adafruit_NeoPixel.h>
#include <math.h>
 
#define N_PIXELS  16  // Number of pixels in strand
#define MIC_PIN   A9  // Microphone is attached to this analog pin
#define LED_PIN    6  // NeoPixel LED strand is connected to this pin
#define SAMPLE_WINDOW   10  // Sample window for average level
#define PEAK_HANG 24 //Time of pause before peak dot falls
#define PEAK_FALL 4 //Rate of falling peak dot
#define INPUT_FLOOR 10 //Lower range of analogRead input
#define INPUT_CEILING 300 //Max range of analogRead input, the lower the value the more sensitive (1023 = max)
 
 
 
byte peak = 16;      // Peak level of column; used for falling dots
unsigned int sample;
 
byte dotCount = 0;  //Frame counter for peak dot
byte dotHangCount = 0; //Frame counter for holding peak dot
 
Adafruit_NeoPixel strip = Adafruit_NeoPixel(N_PIXELS, LED_PIN, NEO_GRB + NEO_KHZ800);
 
void setup() 
{
  // This is only needed on 5V Arduinos (Uno, Leonardo, etc.).
  // Connect 3.3V to mic AND TO AREF ON ARDUINO and enable this
  // line.  Audio samples are 'cleaner' at 3.3V.
  // COMMENT OUT THIS LINE FOR 3.3V ARDUINOS (FLORA, ETC.):
  //  analogReference(EXTERNAL);
 
  // Serial.begin(9600);
  strip.begin();
  strip.show(); // Initialize all pixels to 'off'
 
}
 
void loop() 
{
  unsigned long startMillis= millis();  // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level
 
  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;
 
 
  // collect data for length of sample window (in mS)
  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    sample = analogRead(MIC_PIN);
    if (sample < 1024)  // toss out spurious readings
    {
      if (sample > signalMax)
      {
        signalMax = sample;  // save just the max levels
      }
      else if (sample < signalMin)
      {
        signalMin = sample;  // save just the min levels
      }
    }
  }
  peakToPeak = signalMax - signalMin;  // max - min = peak-peak amplitude
 
  // Serial.println(peakToPeak);
 
 
  //Fill the strip with rainbow gradient
  for (int i=0;i<=strip.numPixels()-1;i++){
    strip.setPixelColor(i,Wheel(map(i,0,strip.numPixels()-1,30,150)));
  }
 
 
  //Scale the input logarithmically instead of linearly
  c = fscale(INPUT_FLOOR, INPUT_CEILING, strip.numPixels(), 0, peakToPeak, 2);
 
  
 
 
  if(c < peak) {
    peak = c;        // Keep dot on top
    dotHangCount = 0;    // make the dot hang before falling
  }
  if (c <= strip.numPixels()) { // Fill partial column with off pixels
    drawLine(strip.numPixels(), strip.numPixels()-c, strip.Color(0, 0, 0));
  }
 
  // Set the peak dot to match the rainbow gradient
  y = strip.numPixels() - peak;
  
  strip.setPixelColor(y-1,Wheel(map(y,0,strip.numPixels()-1,30,150)));
 
  strip.show();
 
  // Frame based peak dot animation
  if(dotHangCount > PEAK_HANG) { //Peak pause length
    if(++dotCount >= PEAK_FALL) { //Fall rate 
      peak++;
      dotCount = 0;
    }
  } 
  else {
    dotHangCount++; 
  }
}
 
//Used to draw a line between two points of a given color
void drawLine(uint8_t from, uint8_t to, uint32_t c) {
  uint8_t fromTemp;
  if (from > to) {
    fromTemp = from;
    from = to;
    to = fromTemp;
  }
  for(int i=from; i<=to; i++){
    strip.setPixelColor(i, c);
  }
}
 
 
float fscale( float originalMin, float originalMax, float newBegin, float
newEnd, float inputValue, float curve){
 
  float OriginalRange = 0;
  float NewRange = 0;
  float zeroRefCurVal = 0;
  float normalizedCurVal = 0;
  float rangedValue = 0;
  boolean invFlag = 0;
 
 
  // condition curve parameter
  // limit range
 
  if (curve > 10) curve = 10;
  if (curve < -10) curve = -10;
 
  curve = (curve * -.1) ; // - invert and scale - this seems more intuitive - postive numbers give more weight to high end on output 
  curve = pow(10, curve); // convert linear scale into lograthimic exponent for other pow function
 
  /*
   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println(); 
   */
 /*
  // Check for out of range inputValues
  if (inputValue < originalMin) {
    inputValue = originalMin;
  }
  if (inputValue > originalMax) {
    inputValue = originalMax;
  }
 
  // Zero Refference the values
  OriginalRange = originalMax - originalMin;
 
  if (newEnd > newBegin){ 
    NewRange = newEnd - newBegin;
  }
  else
  {
    NewRange = newBegin - newEnd; 
    invFlag = 1;
  }
 
  zeroRefCurVal = inputValue - originalMin;
  normalizedCurVal  =  zeroRefCurVal / OriginalRange;   // normalize to 0 - 1 float
 
  // Check for originalMin > originalMax  - the math for all other cases i.e. negative numbers seems to work out fine 
  if (originalMin > originalMax ) {
    return 0;
  }
 
  if (invFlag == 0){
    rangedValue =  (pow(normalizedCurVal, curve) * NewRange) + newBegin;
 
  }
  else     // invert the ranges
  {   
    rangedValue =  newBegin - (pow(normalizedCurVal, curve) * NewRange); 
  }
 
  return rangedValue;
}
 
 
// Input a value 0 to 255 to get a color value.
// The colours are a transition r - g - b - back to r.
uint32_t Wheel(byte WheelPos) {
  if(WheelPos < 85) {
    return strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}
*/
  //respond to audio input and make VU meters with the neopixel strips
}

void attract(){
  // do some fancy rainbow flashy thingys to make it more attractive
  
}
  
 
