//Avatar Kiosk Arduino Control

/***********************************************************
*                         DEFINES                          *
***********************************************************/
//Pin Defines
#define BL_NEOPIXEL_S        9       // Back left neopixel strip data pin
#define BR_NEOPIXEL_S        6       // Back right neopixel strip data pin
#define MON_NEOPIXEL_S       5       // Monitor attached neopixel strip data pin
#define SPARE_NEOPIXEL_S     3       // Spare neopixel pin - (NOT USED)

#define AMP_SLEEP            4       // Logic control pin for sleep of Audio Amplifier
#define FANS_CONTROL         2       // Fan Control pin - N-Channel Mosfet control (Fairchild FQP20N06)
#define AUDIO_IN            A0       // Audio input sampling pin. Used with voltage divider/offset
#define REMOTEIN_ON         10       // RF Remote input -  Pin D3(A) from RF - ON control
#define REMOTEIN_OFF        11       // RF Remote input -  Pin D0(D) from RF - OFF control
#define COMPUTER_PWR        A2       // Computer power button pin - ground for press
#define COMPUTER_SENSE      A3       // Computer power LED pin - tied to +5v of computer PWR LED pin

//MagicNumber Defines
#define N_PIXELS_BACK       58       // Number of pixels in strand
#define N_PIXELS_MON        35       // Number of pixels in strand
#define N_PIXELS_SPARE      30       // Number of pixels in strand
#define SAMPLE_WINDOW       30       // Sample window for average level
#define PEAK_HANG            1       //Time of pause before peak dot falls
#define PEAK_FALL            1       //Rateof falling peak dot
#define INPUT_FLOOR          9       //Lower range of analogRead input
#define INPUT_CEILING      100       //Max range of analogRead input, the lower the value the more sensitive (1023 = max)

/***********************************************************
*                         INCLUDES                         *
***********************************************************/
#include <Adafruit_NeoPixel.h>
#include <math.h>

/***********************************************************
*                      GLOBAL VARS                         *
***********************************************************/
// Define neopixel strip names
Adafruit_NeoPixel bl_strip = Adafruit_NeoPixel(N_PIXELS_BACK, BL_NEOPIXEL_S, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel br_strip = Adafruit_NeoPixel(N_PIXELS_BACK, BR_NEOPIXEL_S, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel mon_strip = Adafruit_NeoPixel(N_PIXELS_MON, MON_NEOPIXEL_S, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel spare_strip = Adafruit_NeoPixel(N_PIXELS_SPARE, SPARE_NEOPIXEL_S, NEO_GRB + NEO_KHZ800);

unsigned int sample;                 // Integer value setup for sample var
byte peak                 = 16;      // Peak level of column; used for falling dots
byte dotCount             = 0;       // Frame counter for peak dot
byte dotHangCount         = 0;       // Frame counter for holding peak dot

boolean  remoteBtnA_state = 0;       // Init variables and Set initial button states
boolean  remoteBtnD_state = 0;       // Init variables and Set initial button states
boolean  compState        = 0;       // Set computer status to default as off
int      rawAudioVal      = 0;       // Raw input from analog pin - possibly extraneous now 
int      speakDelayCount  = 0;       // Counter for arbitrary time system is not speaking/playing audio
boolean  systemState      = 0;       // Used to track system intended state for light control

/***********************************************************
*                          SETUP                           *
***********************************************************/
void setup(){
  // Set all pin modes appropriately. See above defines for pin descriptions
  pinMode(REMOTEIN_ON, INPUT);
  pinMode(REMOTEIN_OFF, INPUT); 
  pinMode(AUDIO_IN, INPUT); 
  pinMode(FANS_CONTROL, OUTPUT); 
  pinMode(AMP_SLEEP, OUTPUT); 
  pinMode(BL_NEOPIXEL_S, OUTPUT);
  pinMode(BR_NEOPIXEL_S, OUTPUT);
  pinMode(MON_NEOPIXEL_S, OUTPUT);
  pinMode(SPARE_NEOPIXEL_S, OUTPUT);
  pinMode(COMPUTER_PWR, INPUT);      // Set initially to INPUT to act like high impedance (we'll triger it as output later)
  pinMode(COMPUTER_SENSE, INPUT);
  
  // Initialize serial for debug
  Serial.begin(9600);
  
  // Initialize all neopixel strips             
  bl_strip.begin();                
  br_strip.begin();
  mon_strip.begin();
  spare_strip.begin();
  
  //Initialize all pixels to OFF
  bl_strip.show();
  br_strip.show();
  mon_strip.show();
  spare_strip.show();
  
  // Run the startUp function before heading to the main loop
  // ensure that the computer is initialized on a full power up 
  startUp();
}

/***********************************************************
*                          LOOP                            *
***********************************************************/

void loop(){
  readRemoteButtonStates();         // Function to read RF remote button states
  
  if (remoteBtnA_state) {           // If A button is being pressed
    Serial.println("Remote Button A Detected");
      startUp();                    // run the startUp function
  }
      
  if (remoteBtnD_state){            // If D button is being pressed
    Serial.println("Remote Button D Detected");
      shutDown();                   // run the shutDown function
  }
  
   // Every loop, go to the code that writes to the neopixel strips like a VU meter
   // This way, we can transition quickly from attract loop to VU meter display   
   vuMeter();                        // run VU meter function - will do nothing if no audio is above threshold, but
                                     // will check and report back the peak audio variable
  
  
  // The following 3 IF statements control the attract loop and motherboard lighting. VU meters are seperately initiated
  //
  if (peak > 56 && systemState){     // checks the peak audio var to know if audio is present and intended system state
    speakDelayCount++;               // if audio is below the threshold, add one to the speakDelayCount
    Serial.println(speakDelayCount); // print the current delay counter
    delay(1);                        // just a little delay so the program doesn't trip on itself
  }
  
  if (peak < 50){                    // check the peak audio var if < 50, then audio present
    speakDelayCount = 0;             // reset the no audio timer/counter
    moboLightsWhite();               // set the lights on the back of the monitor to white, to illuminate motherboard
    delay (1);                   
  }
  
  if (speakDelayCount > 350){        // If we've reached 350 program loops without an audio signal
    Serial.println("I'm not speaking. time to engage the attract loop");
    moboLightsBlue();                // Turn the motherboaard illumination to blue (sleep)
    attract();                       // Send the program to the attract loop
    delay(2);                        // delay for a little while to keep the program from tripping on itself
  }  
}                                    // end of main loop


/***********************************************************
*                        startUp                           *
***********************************************************/

void startUp(){                      // system start up sequence
 
  Serial.println("Startup sequence started");
    digitalWrite(FANS_CONTROL, HIGH); // Turns on the clear case fans (red light)
    digitalWrite(AMP_SLEEP, LOW);     // Takes the amplifier out of sleep state
 
  //Send the computer's power on button low
  turnOnComputer();                   // function to turn on the computer
  
  // Activates the onboard LED to show that the system is busy. 
  // also pauses for 5 seconds to make sure there are not repeat remote commands sent
  digitalWrite(13, HIGH);
    delay(5000);
  digitalWrite(13, LOW);
  
  systemState = 1;                // Set the system state representation to on
  moboLightsWhite();              // turn the motherboard lights to white after all has been completed


}


/***********************************************************
*                       shutDown                           *
***********************************************************/

void shutDown(){

  Serial.println("Shutdown sequence started");
    digitalWrite(FANS_CONTROL, LOW); // Turns off the clear case fans(red)
    digitalWrite(AMP_SLEEP, HIGH);   // Mutes and sleeps the audio amplifier
 
  //Send the computer's power on button low
  turnOffComputer();                 // function to turn computer off

  // Initialize all lighting strips to off
  bl_strip.show();
  br_strip.show();
  mon_strip.show();
  spare_strip.show();
  
  // Activates the onboard LED to show that the system is busy. 
  // also pauses for 5 seconds to make sure there are not repeat remote commands sent
  digitalWrite(13, HIGH);
    moboLightsOff();
    delay(5000);
  digitalWrite(13, LOW);
  
  systemState = 0;                      // Set system state represenation to off
  speakDelayCount = 0;                  // Reset the speaker delay count to 0 - this keeps system out of attract loop

}


/***********************************************************
*                 readRemoteButtonStates                   *
***********************************************************/

void readRemoteButtonStates(){
  // Read both digitial inputs for the RF remote buttons and store as state
  remoteBtnA_state = digitalRead(REMOTEIN_ON);
  remoteBtnD_state = digitalRead(REMOTEIN_OFF);
}


/***********************************************************
*                readComputerPowerStates                   *
***********************************************************/

void readComputerPowerStates(){
  // Read the computer LED power on pin and store as computer state
  compState = digitalRead(COMPUTER_SENSE);
}


/* Power on and Power off code below was adapted from code written by 
David Sikes for Orlando Science Center                       */

/***********************************************************
*                     turnOnComputer                       *
***********************************************************/

void turnOnComputer(){
  readComputerPowerStates();
  
  Serial.print("turn on computer - state = ");
  Serial.println(compState);
  
  if (!compState) {                     // If the computer is NOT on
    pinMode (COMPUTER_PWR, OUTPUT);     // Set control pin to output
    digitalWrite(COMPUTER_PWR, LOW);    // send the pin LOW (press btn to gnd) for 100 ms
  }
 
 delay(100);
 
 if (!compState) {                      // Sets the computer control pin to high-impedance
   pinMode(COMPUTER_PWR, INPUT);        // so that the on-board button still works
 }
 
 Serial.println("computer should be on now");
}


/***********************************************************
*                    turnOffComputer                       *
***********************************************************/

void turnOffComputer(){
  readComputerPowerStates();
 
  Serial.print("turn off computer - state = ");
  Serial.println(compState);
 
  if (compState) {                        // If the computer is ON
    pinMode (COMPUTER_PWR, OUTPUT);       // set the control pin mode to OUTPUT
    digitalWrite(COMPUTER_PWR, LOW);      // Send the pin LOW (press ptn to gnd) for 100ms
  }
  
 delay(100);                    
 
 if (compState) {                         // Sets the computer control pin to high-impedance 
   pinMode(COMPUTER_PWR, INPUT);          // so that the on-board button still works
 }

  Serial.println("computer should be off now");
}


/***********************************************************
*                         attract                          *
***********************************************************/

void attract(){
  moboLightsBlue();                              // send the motherboard illumination back to blue (sleep)
  Serial.print("attract loop");
      for (int q=0; q < 3; q++) {                // Chase flash, adapted from adafruit neopixel test code
        for (int i=0; i < bl_strip.numPixels(); i=i+3) {
        bl_strip.setPixelColor(i+q, 0,0,255);    //turn every third pixel on
        br_strip.setPixelColor(i+q, 0,0,255);    //turn every third pixel on
      }
      bl_strip.show();                           // send color/pixel information out to neopixel strips
      br_strip.show();
      delay(50);
     
      for (int i=0; i < bl_strip.numPixels(); i=i+3) {
        bl_strip.setPixelColor(i+q, 0);          //turn every third pixel off
        br_strip.setPixelColor(i+q, 0);          //turn every third pixel off
    }
  }
}

/***********************************************************
*                moboLights Color functions                *
***********************************************************/

void moboLightsRed(){
uint32_t red = mon_strip.Color(255,0,0);
  for (int i=0; i < mon_strip.numPixels(); i++) {
        mon_strip.setPixelColor(i, red);   
        mon_strip.setBrightness(255);
        }
        mon_strip.show();
}
  
  
void moboLightsWhite(){
uint32_t fullWhite = mon_strip.Color(255,255,255);
  for (int i=0; i < mon_strip.numPixels(); i++) {
        mon_strip.setPixelColor(i, fullWhite);   
        mon_strip.setBrightness(255);
        }
        mon_strip.show();
}


void moboLightsBlue(){
uint32_t blue = mon_strip.Color(0,0,255);
  for (int i=0; i < mon_strip.numPixels(); i++) {
        mon_strip.setPixelColor(i, blue);   
        mon_strip.setBrightness(255);
        }
        mon_strip.show();
}

void moboLightsOff(){
uint32_t off = mon_strip.Color(0,0,0);
  for (int i=0; i < mon_strip.numPixels(); i++) {
        mon_strip.setPixelColor(i, off);   
//        mon_strip.setBrightness(255);
        }
        mon_strip.show();
}


/***********************************************************
*                         vuMeter                          *
***********************************************************/

/*
LED VU meter for Arduino and Adafruit NeoPixel LEDs. More info: http://learn.adafruit.com/led-ampli-tie/
 
 Written by Adafruit Industries.  Distributed under the BSD license.
 This paragraph must be included in any redistribution.
 
 fscale function:
 Floating Point Autoscale Function V0.1
 Written by Paul Badger 2007
 Modified from code by Greg Shakar
 
*/
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


