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
  //respond to audio input and make VU meters with the neopixel strips
}

void attract(){
  // do some fancy rainbow flashy thingys to make it more attractive
  
}
  
 
