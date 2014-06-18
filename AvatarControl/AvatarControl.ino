 
/***********************************************************
*                         DEFINES                          *
***********************************************************/
//Pin Defines
#define BL_NEOPIXEL_S 9
#define BR_NEOPIXEL_S 6
#define MON_NEOPIXEL_S 5
#define SPARE_NEOPIXEL_S 3

#define AMP_SLEEP 4
#define FANS_CONTROL 2
#define AUDIO_IN A0
#define REMOTEIN_ON 10
#define REMOTEIN_OFF 11
#define COMPUTER_PWR A2
#define COMPUTER_SENSE A3

//MagicNumber Defines
#define N_PIXELS_BACK 58       // Number of pixels in strand
#define N_PIXELS_MON 30        // Number of pixels in strand
#define N_PIXELS_SPARE 30        // Number of pixels in strand
#define SAMPLE_WINDOW 30  // Sample window for average level
#define PEAK_HANG 1       //Time of pause before peak dot falls
#define PEAK_FALL 1         //Rateof falling peak dot
#define INPUT_FLOOR 8      //Lower range of analogRead input
#define INPUT_CEILING 100   //Max range of analogRead input, the lower the value the more sensitive (1023 = max)


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
int      rawAudioVal      = 0;
int      speakDelayCount  = 0;
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
 
 startUp();
}


/***********************************************************
*                          LOOP                            *
***********************************************************/

void loop(){
  readRemoteButtonStates();
  if (remoteBtnA_state) {
    Serial.println("Remote Button A Detected");
      startUp();
  }
  if (remoteBtnD_state){
    Serial.println("Remote Button D Detected");
      shutDown();
  }
  
  //stuff and foo
  //Read state of remote control pins
  //Starup in normal/off mode
  //State machine to determine if starting up, shutting down, VU Meter, Attract Loop
  int val = analogRead(A0);
 // Serial.println(val);
   vuMeter();
   
if (peak > 56){
  speakDelayCount++;
  Serial.println(speakDelayCount);
  delay(1);
  }
if (peak < 50){
  speakDelayCount = 0;
  delay (1);
  }  
if (speakDelayCount > 350){
  Serial.println("I'm not speaking. time to engage the attract loop");
  attract();
  delay(5);
  }  
}



void startUp(){
    //Do some fancy power on animation here
    //Enable the computer's power on button
  Serial.println("Startup sequence started");
  digitalWrite(FANS_CONTROL, HIGH); // Turns on the clear case fans (red light)
  digitalWrite(AMP_SLEEP, LOW); // Takes the amplifier out of sleep state
  
  turnOnComputer(); // Turns on the computer
  // Activates the onboard LED to show that the system is busy. 
  // also pauses for 5 seconds to make sure there are not repeat remote commands sent
  digitalWrite(13, HIGH);
  delay(5000);
  digitalWrite(13, LOW);
  


}

void shutDown(){
  //do some fancy power off animation here
  //pull the power button high again momentarily
  Serial.println("Shutdown sequence started");
  
  digitalWrite(FANS_CONTROL, LOW); // Turns off the clear case fans(red)
  digitalWrite(AMP_SLEEP, HIGH); // Mutes and sleeps the audio amplifier
  turnOffComputer(); // Turns computer off

  // Initialize all lighting strips to off
  bl_strip.show();
  br_strip.show();
  mon_strip.show();
  spare_strip.show();
  // Activates the onboard LED to show that the system is busy. 
  // also pauses for 5 seconds to make sure there are not repeat remote commands sent
  digitalWrite(13, HIGH);
  delay(5000);
  digitalWrite(13, LOW);
}

void readRemoteButtonStates(){

  remoteBtnA_state = digitalRead(REMOTEIN_ON);
  remoteBtnD_state = digitalRead(REMOTEIN_OFF);
}

void readComputerPowerStates(){
  compState = digitalRead(COMPUTER_SENSE);
}

void turnOnComputer(){
  readComputerPowerStates();
  Serial.print("turn on computer - state = ");
  Serial.println(compState);
  if (!compState) {
    pinMode (COMPUTER_PWR, OUTPUT);
    digitalWrite(COMPUTER_PWR, LOW);
  }
 delay(100);
  if (!compState) {
     pinMode(COMPUTER_PWR, INPUT);
  }
Serial.println("computer should be on now");
}


void turnOffComputer(){
  readComputerPowerStates();
    Serial.print("turn off computer - state = ");
  Serial.println(compState);
  if (compState) {
    pinMode (COMPUTER_PWR, OUTPUT);
    digitalWrite(COMPUTER_PWR, LOW);
  }
 delay(100);
  if (compState) {
     pinMode(COMPUTER_PWR, INPUT);
  }
Serial.println("computer should be off now");
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
    br_strip.setPixelColor(i,Wheel(map(i,0,bl_strip.numPixels()-1,30,150)));
  }
 
 
  //Scale the input logarithmically instead of linearly
  c = fscale(INPUT_FLOOR, INPUT_CEILING, bl_strip.numPixels(), 0, peakToPeak, 2);
 
  
 
 
  if(c < peak) {
    peak = c;        // Keep dot on top
    dotHangCount = 0;    // make the dot hang before falling
  }
  if (c <= bl_strip.numPixels()) { // Fill partial column with off pixels
    drawLine(bl_strip.numPixels(), bl_strip.numPixels()-c, bl_strip.Color(0, 0, 0));
    drawLine(br_strip.numPixels(), br_strip.numPixels()-c, br_strip.Color(0, 0, 0));
  }
 
  // Set the peak dot to match the rainbow gradient
  y = bl_strip.numPixels() - peak;
  y = br_strip.numPixels() - peak;
  
  bl_strip.setPixelColor(y-1,Wheel(map(y,0,bl_strip.numPixels()-1,30,150)));
  br_strip.setPixelColor(y-1,Wheel(map(y,0,br_strip.numPixels()-1,30,150)));
  
  bl_strip.show();
  br_strip.show();
 
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
//Serial.println(peak);
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
    br_strip.setPixelColor(i, c);
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
 

  // Serial.println(curve * 100, DEC);   // multply by 100 to preserve resolution  
   //Serial.println(); 


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
    return br_strip.Color(WheelPos * 3, 255 - WheelPos * 3, 0);
  } 
  else if(WheelPos < 170) {
    WheelPos -= 85;
    return bl_strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
    return br_strip.Color(255 - WheelPos * 3, 0, WheelPos * 3);
  } 
  else {
    WheelPos -= 170;
    return bl_strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
    return br_strip.Color(0, WheelPos * 3, 255 - WheelPos * 3);
  }
}

  //respond to audio input and make VU meters with the neopixel strips

void attract(){
  Serial.print("attract loop");
  uint32_t c;
   c = (127, 127, 127);
  for (int j=0; j<10; j++) {  //do 10 cycles of chasing
      checkRawAudio();
      for (int q=0; q < 3; q++) {
        for (int i=0; i < bl_strip.numPixels(); i=i+3) {
        bl_strip.setPixelColor(i+q, c);    //turn every third pixel on
        br_strip.setPixelColor(i+q, c);    //turn every third pixel on
      }
      bl_strip.show();
      br_strip.show();
      delay(50);
     
      for (int i=0; i < bl_strip.numPixels(); i=i+3) {
        bl_strip.setPixelColor(i+q, 0);        //turn every third pixel off
        br_strip.setPixelColor(i+q, 0);        //turn every third pixel off

      }
    }
  }
}
  // do some fancy rainbow flashy thingys to make it more attractive

void checkRawAudio(){ // Pseudo interrupt to get more quickly back to VU meter
      rawAudioVal = analogRead(AUDIO_IN);
      Serial.println(rawAudioVal);
      if (rawAudioVal < 510){
        for (int i=0; i < bl_strip.numPixels(); i++) {
          bl_strip.setPixelColor(i, 0);    //turn every third pixel on
          br_strip.setPixelColor(i, 0);    //turn every third pixel on       
        }
       bl_strip.show();
       br_strip.show(); 
       loop(); 
      }  
}
