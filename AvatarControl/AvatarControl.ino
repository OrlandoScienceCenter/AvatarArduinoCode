 
/***********************************************************
*                         DEFINES                          *
***********************************************************/
//Pin Defines
#define BL_NEOPIXEL_S      9
#define BR_NEOPIXEL_S      6
#define MON_NEOPIXEL_S     5
#define SPARE_NEOPIXEL_S   3

#define AMP_SLEEP          12
#define FANS_CONTROL       11
#define AUDIO_IN           A0
#define REMOTEIN_ON        3
#define REMOTEIN_OFF       2
#define COMPUTER_PWR       A1
#define COMPUTER_SENSE     A2

//MagicNumber Defines
#define N_PIXELS_BACK  16        // Number of pixels in strand
#define N_PIXELS_MON  16        // Number of pixels in strand
#define N_PIXELS_SPARE  16        // Number of pixels in strand
#define SAMPLE_WINDOW   10  // Sample window for average level
#define PEAK_HANG 24        //Time of pause before peak dot falls
#define PEAK_FALL 4         //Rate of falling peak dot
#define INPUT_FLOOR 10      //Lower range of analogRead input
#define INPUT_CEILING 300   //Max range of analogRead input, the lower the value the more sensitive (1023 = max)


/***********************************************************
*                         INCLUDES                         *
***********************************************************/
#include <Adafruit_NeoPixel.h>
#include <math.h>

/***********************************************************
*                      GLOBAL VARS                         *
***********************************************************/
Adafruit_NeoPixel bl_strip = Adafruit_NeoPixel(N_PIXELS_BACK, BL_NEOPIXEL_S, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel br_strip = Adafruit_NeoPixel(N_PIXELS_BACK, BR_NEOPIXEL_S, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel mon_strip = Adafruit_NeoPixel(N_PIXELS_MON, MON_NEOPIXEL_S, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel spare_strip = Adafruit_NeoPixel(N_PIXELS_SPARE, SPARE_NEOPIXEL_S, NEO_GRB + NEO_KHZ800);

byte peak = 16;      // Peak level of column; used for falling dots
unsigned int sample;
byte dotCount = 0;  //Frame counter for peak dot
byte dotHangCount = 0; //Frame counter for holding peak dot

boolean  remoteBtnA_state = 0;
boolean  remoteBtnD_state = 0;
boolean  compState        = 0;

/***********************************************************
*                          SETUP                           *
***********************************************************/
void setup(){
    //stuff and foo
  //pin defines
  //initial states
  pinMode(REMOTEIN_ON, INPUT);
  pinMode(REMOTEIN_OFF, INPUT); 
  pinMode(AUDIO_IN, INPUT); 
  pinMode(FANS_CONTROL, OUTPUT); 
  pinMode(AMP_SLEEP, OUTPUT); 
  pinMode(BL_NEOPIXEL_S, OUTPUT);
  pinMode(BR_NEOPIXEL_S, OUTPUT);
  pinMode(MON_NEOPIXEL_S, OUTPUT);
  pinMode(SPARE_NEOPIXEL_S, OUTPUT);
  pinMode(COMPUTER_PWR, INPUT_PULLUP);
  pinMode(COMPUTER_SENSE, INPUT);
  
  Serial.begin(9600);
  //neopixel init
  bl_strip.begin();
  br_strip.begin();
  mon_strip.begin();
  spare_strip.begin();
  //Initialize all pixels to OFF
  bl_strip.show();
  br_strip.show();
  mon_strip.show();
  spare_strip.show();
 
}


/***********************************************************
*                          LOOP                            *
***********************************************************/

void loop(){
  readRemoteButtonStates();
  if (remoteBtnA_state) {
      startUp;
  }
  if (remoteBtnD_state){
      shutDown;
  }
  
  //stuff and foo
  //Read state of remote control pins
  //Starup in normal/off mode
  //State machine to determine if starting up, shutting down, VU Meter, Attract Loop

  }  
void startUp(){
    //Do some fancy power on animation here
    //Enable the computer's power on button
  digitalWrite(FANS_CONTROL, HIGH);
  digitalWrite(AMP_SLEEP, LOW);
  turnOnComputer();
  


}

void shutDown(){
  //do some fancy power off animation here
  //pull the power button high again momentarily
  digitalWrite(FANS_CONTROL, LOW);
  digitalWrite(AMP_SLEEP, HIGH);
  
  bl_strip.show();
  br_strip.show();
  mon_strip.show();
  spare_strip.show();
  
}

void readRemoteButtonStates(){
  remoteBtnA_state = digitalRead(REMOTEIN_ON);
  remoteBtnD_state = digitalRead(REMOTEIN_OFF);
}

void readComputerPowerStates(){
  compState = digitalRead(COMPUTER_SENSE);
}

void turnOnComputer(){
  if (!compState) {
    pinMode (COMPUTER_PWR, OUTPUT);
    digitalWrite(COMPUTER_PWR, LOW);
  }
 delay(100);
  if (!compState) {
     pinMode(COMPUTER_PWR, INPUT);
  }
}
void turnOffComputer(){
  if (compState) {
    pinMode (COMPUTER_PWR, OUTPUT);
    digitalWrite(COMPUTER_PWR, LOW);
  }
 delay(100);
  if (compState) {
     pinMode(COMPUTER_PWR, INPUT);
  }
}




void vuMeter(){
 // Code below is from adafruit amplitie. use parts as needed in other code. Be sure to credit original source.  
  unsigned long startMillis= millis();  // Start of sample window
  float peakToPeak = 0;   // peak-to-peak level
 
  unsigned int signalMax = 0;
  unsigned int signalMin = 1023;
  unsigned int c, y;
 
 
  // collect data for length of sample window (in mS)
  while (millis() - startMillis < SAMPLE_WINDOW)
  {
    sample = analogRead(AUDIO_IN);
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
  for (int i=0;i<=bl_strip.numPixels()-1;i++){
    bl_strip.setPixelColor(i,Wheel(map(i,0,bl_strip.numPixels()-1,30,150)));
  }
 
 
  //Scale the input logarithmically instead of linearly
  c = fscale(INPUT_FLOOR, INPUT_CEILING, bl_strip.numPixels(), 0, peakToPeak, 2);
 
  
 
 
  if(c < peak) {
    peak = c;        // Keep dot on top
    dotHangCount = 0;    // make the dot hang before falling
  }
  if (c <= bl_strip.numPixels()) { // Fill partial column with off pixels
    drawLine(bl_strip.numPixels(), bl_strip.numPixels()-c, bl_strip.Color(0, 0, 0));
  }
 
  // Set the peak dot to match the rainbow gradient
  y = bl_strip.numPixels() - peak;
  
  bl_strip.setPixelColor(y-1,Wheel(map(y,0,bl_strip.numPixels()-1,30,150)));
 
  bl_strip.show();
 
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
    bl_strip.setPixelColor(i, c);
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
 

   Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   Serial.println(); 


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
    return bl_strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return bl_strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return bl_strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

  //respond to audio input and make VU meters with the neopixel strips

void attract(){
  // do some fancy rainbow flashy thingys to make it more attractive
  
}
  
 
