#include <Arduino.h>
#include <Streaming.h>
#include <SoftwareSerial.h>
#include "WiFlySerial.h"

//Declare run state constants
const int RUN_STATE_BOOT_PINS           = 1001;
const int RUN_STATE_BOOT_RNXV           = 1002;
const int RUN_STATE_BOOT_ADXL           = 1003;
const int RUN_STATE_BOOT_MAX17043       = 1004;
const int RUN_STATE_BOOT_ID12           = 1005;
const int RUN_STATE_ENTER_CONFIG_RNXV   = 1006;
const int RUN_STATE_CONFIG_RNXV         = 1007;
const int RUN_STATE_CONFIG_IDENTIFY     = 1008;
const int RUN_STATE_CONFIG_BTN_DOWN     = 1009;
const int RUN_STATE_RUN                 = 1010;
const int RUN_STATE_BTN_DOWN            = 1011;
const int RUN_STATE_STATUS              = 1012;
const int RUN_STATE_POUR_START          = 1013;
const int RUN_STATE_POUR_ON             = 1014;
const int RUN_STATE_POUR_END            = 1015;
const int RUN_STATE_BADGE_SCAN          = 1016;
const int RUN_STATE_FATAL_ERROR         = 1017;
const int RUN_STATE_TEST                = 1018;

int runState;
int counter;

const int RNXV_RX             = 4;
const int RNXV_TX             = 5;


WiFlySerial WiFly(RNXV_RX, RNXV_TX);
#define REQUEST_BUFFER_SIZE 120
#define HEADER_BUFFER_SIZE 150 
#define BODY_BUFFER_SIZE 100
char bufRequest[REQUEST_BUFFER_SIZE];
char bufHeader[HEADER_BUFFER_SIZE];
char bufBody[BODY_BUFFER_SIZE];

String SPIInputString;


void setup()
{
  Serial.begin(9600);
}

void loop()
{
  switch (runState)
  {
    case RUN_STATE_BOOT_RNXV:
      /////////////////////
      //    BOOT RNXV    //
      /////////////////////
      //Boot sequence for the RN-XV WiFi radio
      //load the stored WiFi AP access settings
      //attempt to join the stored SSID
      //if successful:
      //continue boot
      //else:
      //dump to config mode so user can connect and update the stored 'ap' config settings
      Serial.println("BOOT_RNXV");
      
      delay(500);
      
      WiFly.begin();
      delay(10);
      Serial.println("WiFly UP");
      
      runState = RUN_STATE_ENTER_CONFIG_RNXV;
      
      //clear out buffer
      WiFly.flush();
      delay(100);
      
      break;
    case RUN_STATE_CONFIG_RNXV:
      ///////////////////////
      //    CONFIG_RNXV    //
      ///////////////////////
      //Config mode parking state
      //Wait here while config settings are updated via iOS app
      //iOS app talks directly to RN-XV, updating settings and storing in default config file
      //When the iOS app is done, it will send 'RNXVREBOOT' over the socket connection
      //When we hear the reboot code come over, go to BOOT_PINS
      //Serial.println("CONFIG_RNXV");
      //Serial.println(freeMemory());
      
      //Testing: un-comment to send commands via serialMonitor
      
      if(Serial.available()) 
      { // Outgoing data
        //WiFly.write( (chOut = Serial.read()) );
        //Serial.write (chOut);
        WiFly.write( Serial.read() );
      }
      
      
      
      //Check the RNXV buffer for any new bytes that have arrived over the socket connection
      //Push them into the SPIInputString input buffer
      
      while(WiFly.available() > 0)
      {
        char s = WiFly.read();
        SPIInputString.concat(s);
        Serial.print(s);
        //Serial.write(WiFly.read());
        
      }
      
      break;
  }
}
