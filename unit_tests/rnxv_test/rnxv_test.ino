#include <Arduino.h>
#include <Streaming.h>
#include <SoftwareSerial.h>
#include "WiFlySerial.h"
#include "MemoryFree.h"
#include "Credentials.h"

//Declare internal run state stuff
int runState;
int counter;
int btn_counter;

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

//Declare pin constants
const int MODE_BUTTON_DATA    = 3;
const int MODE_BUTTON_PWR     = 2;
const int STATUS_LED          = 10;
const int RNXV_RX             = 4;
const int RNXV_TX             = 5;

//Declare ID-12 RFID reader and buffer
//SoftwareSerial id12SoftSerial(ID12_RX, ID12_TX);
char tagString[13];
int index;
boolean reading;
int t;

//Declare RNXV Object and I/O buffers
WiFlySerial WiFly(RNXV_RX, RNXV_TX);
#define REQUEST_BUFFER_SIZE 120
#define HEADER_BUFFER_SIZE 150 
#define BODY_BUFFER_SIZE 100
char bufRequest[REQUEST_BUFFER_SIZE];
char bufHeader[HEADER_BUFFER_SIZE];
char bufBody[BODY_BUFFER_SIZE];

String SPIInputString;

//Start and end times for pour measuring
int pourStartTime;
int pourEndTime;
int pourDuration;
String req;

void setup()
{
  digitalWrite(STATUS_LED, HIGH);
  runState = RUN_STATE_BOOT_PINS;
  Serial.begin(9600);
  //Serial.println("SETUP( )");
}

void loop()
{
  switch (runState)
  {
    case RUN_STATE_BOOT_PINS:
      /////////////////////
      //    BOOT PINS    //
      /////////////////////
      //Energize and set I/O mode for all pins
      Serial.println("BOOT_PINS");
      Serial.println(freeMemory());

      
      pinMode(STATUS_LED, OUTPUT);
      digitalWrite(STATUS_LED, LOW);
      pinMode(MODE_BUTTON_DATA, INPUT);
      digitalWrite(MODE_BUTTON_DATA, LOW);
      pinMode(MODE_BUTTON_PWR, OUTPUT);
      digitalWrite(MODE_BUTTON_PWR, HIGH);
      
      runState = RUN_STATE_BOOT_RNXV;
      
      break;
      
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
      Serial.println(freeMemory());
      
      //Flash STATUS LED 2x
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(500);
      
      WiFly.begin();
      Serial.println("WiFly UP");
      
      delay(1000);
      
      WiFly.setRemotePort(80);
      
      
      WiFly.SendCommand("load ap",">", bufBody, BODY_BUFFER_SIZE);
      delay(1000);
      WiFly.SendCommand("save",">", bufBody, BODY_BUFFER_SIZE);
      delay(1000);
      WiFly.SendCommand("reboot","Ver", bufBody, BODY_BUFFER_SIZE);
      delay(3000);
      WiFly.SendCommand("join",">", bufBody, BODY_BUFFER_SIZE);
      delay(3000);
      
      Serial.println("getDeviceStatus");
      Serial.println(freeMemory());
      
      WiFly.getDeviceStatus();
      if (WiFly.isifUp())
      {
        // if association succeeded, advance state
        runState = RUN_STATE_BOOT_ADXL;
      }
      else
      {
        // if association failed, enter config mode
        runState = RUN_STATE_ENTER_CONFIG_RNXV;
      }
      
      //clear out buffer
      WiFly.flush();
      
      break;
    
    
    case RUN_STATE_BOOT_ADXL:
      /////////////////////
      //    BOOT ADXL    //
      /////////////////////
      //Boot sequence for the ADXL345 accelerometer
      //Energize the accel.
      //wait 5 seconds for orientation vals to settle into low-pass filter
      
      //Flash STATUS LED 2x
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      
      
      Serial.println("BOOT_ADXL");
      runState = RUN_STATE_BOOT_MAX17043;
      
      break;
      
    case RUN_STATE_BOOT_MAX17043:
      /////////////////////////
      //    BOOT MAX17043    //
      /////////////////////////
      //Boot sequence for the MAX17043 battery fuel gauge
      //Energize and set up initial state of fuel gauge
      
      //Flash STATUS LED 2x
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      
      Serial.println("BOOT_MAX17043");
      runState = RUN_STATE_BOOT_ID12;
      
      break;
    
    case RUN_STATE_BOOT_ID12:
      /////////////////////
      //    BOOT ID12    //
      /////////////////////
      //Boot sequence for ID-12 RFID reader
      //Set ID-12 SoftwareSerial instance to active
      //reset reader
      
      //Flash STATUS LED 2x
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      
      Serial.println("BOOT_ID12");
      runState = RUN_STATE_RUN;
      
      break;
    
    case RUN_STATE_ENTER_CONFIG_RNXV:
      /////////////////////////////
      //    ENTER CONFIG RNXV    //
      /////////////////////////////
      //Send the RN-XV radio into config mode
      //Make the WiFly's SPI active
      //try to send RN-XV into command mode
      //if successful:
      //load the AdHoc network settings and reboot the radio
      //else:
      //go to FATAL_ERROR state
      
      //Flash STATUS LED 4x
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      
      Serial.println("ENTER_CONFIG_RNXV");
      Serial.println(freeMemory());
      
      WiFly.SendCommand("load adhoc",">", bufBody, BODY_BUFFER_SIZE);
      delay(1000);
      Serial.println("adhoc loaded");
      WiFly.SendCommand("save",">", bufBody, BODY_BUFFER_SIZE);
      delay(1000);
      Serial.println("settings saved");
      WiFly.SendCommand("reboot","Ver", bufBody, BODY_BUFFER_SIZE);
      delay(3000);
      Serial.println("reboot called");      
      runState = RUN_STATE_CONFIG_RNXV;
      digitalWrite( STATUS_LED, HIGH );
      Serial.println(freeMemory());
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
      
      //Checking ot see if ARD has crashed here somehow
      //counter++;
      //Serial.print(counter);
      
      /*
      //Blink the LED to show we're in config mode
      t = millis() % 2000;
      if(t>1000)
      {
        digitalWrite( STATUS_LED, HIGH );
      }
      else
      {
        digitalWrite( STATUS_LED, LOW );
      }
      */
      
      //Check the state of the mode button
      //if pressed while in config mode, reboot
      /*
      if(digitalRead(MODE_BUTTON_DATA) == LOW)
      {
        runState = RUN_STATE_CONFIG_BTN_DOWN;
        digitalWrite( STATUS_LED, LOW );
      }
      */
      
      
      
      //Check the RNXV buffer for any new bytes that have arrived over the socket connection
      //Push them into the SPIInputString input buffer
      
      while(WiFly.available() > 0)
      {
        char s = WiFly.read();
        SPIInputString.concat(s);
        delay(10);
        //Serial.print(s);
        //Serial.write(WiFly.read());
        
      }
      
      /*
      //Write any serial data sent into the Arduio out to the WiFly
      //(via the SerialMonitor)
      if(Serial.available()) { // Outgoing data
        WiFly.write( Serial.read() );
      }
      */
      
      
      //Scan the RNXV input buffer for char command sequences from the iOS config app
      if(SPIInputString.length() > 2 && SPIInputString.indexOf("CNX") != -1)
      {
        Serial.println("CNX received");
        WiFly.flush();
        delay(100);
        SPIInputString = " ";
        //This iOS device is requesting a connection check
        //respond with 'CNX'
        
        WiFly.write("CNX");
        delay(100);
      }
      else if(SPIInputString.length() > 2 && SPIInputString.indexOf("IDFY") != -1)
      {
        Serial.println("IDFY received");
        WiFly.flush();
        delay(100);
        SPIInputString = " ";
        //This iOS device is requesting a that this handle identify itself
        //switch to identify state
        runState = RUN_STATE_CONFIG_IDENTIFY;
        delay(100);
      }
      else if(SPIInputString.length() > 2 && SPIInputString.indexOf("ECONF") != -1)
      {
        if((SPIInputString.indexOf("BCONF") != -1))
        {
          
          //Flash STATUS LED 2x to acknowledge
          digitalWrite(STATUS_LED, HIGH);
          delay(50);
          digitalWrite(STATUS_LED, LOW);
          delay(100);
          digitalWrite(STATUS_LED, HIGH);
          delay(50);
          digitalWrite(STATUS_LED, LOW);
          delay(200);
          
          //grab config string from the buffer
          //wiil look like this:
          //BCONF+HallMonitor+b33rb33r+0+207.138.10.1+2000+ECONF
          int startIndex = SPIInputString.indexOf("BCONF") + 5;
          int endIndex = SPIInputString.indexOf("ECONF");
          String confStr = SPIInputString.substring(startIndex, endIndex);
          Serial.println("\n"+confStr);
          SPIInputString = " ";
          delay(100);
          
          //pick the props out of the config string one by one starting at the end:
          String ssid;
          String phrase;
          String host;
          
          //grab ssid
          endIndex = confStr.length() - 1;
          startIndex = confStr.lastIndexOf("+", endIndex - 1) + 1;
          host = "set ip host " + confStr.substring(startIndex, endIndex);
          char hostBuf[host.length()+1];
          host.toCharArray(hostBuf, host.length()+1);
          
          //grab password
          endIndex = startIndex - 1;
          startIndex = confStr.lastIndexOf("+", endIndex - 1) + 1;
          phrase = "set wlan phrase " + confStr.substring(startIndex, endIndex);
          char phraseBuf[phrase.length()+1];
          phrase.toCharArray(phraseBuf, phrase.length()+1);
          
          //grab pourdb host
          endIndex = startIndex - 1;
          startIndex = confStr.lastIndexOf("+", endIndex - 1) + 1;
          ssid = "set wlan ssid " + confStr.substring(startIndex, endIndex);
          char ssidBuf[ssid.length()+1];
          ssid.toCharArray(ssidBuf, ssid.length()+1);
          
          
          Serial.println(ssidBuf);
          Serial.println(phraseBuf);
          Serial.println(hostBuf);
          
          delay(100);
          //Push all the commands out to the RNXV
          WiFly.SendCommand(ssidBuf,">", bufBody, BODY_BUFFER_SIZE);
          delay(100);
          WiFly.SendCommand(phraseBuf,">", bufBody, BODY_BUFFER_SIZE);
          delay(100);
          WiFly.SendCommand(hostBuf,">", bufBody, BODY_BUFFER_SIZE);
          delay(100);
          WiFly.SendCommand("set ip dhcp 0",">", bufBody, BODY_BUFFER_SIZE);
          delay(100);
          WiFly.SendCommand("set wlan join 0",">", bufBody, BODY_BUFFER_SIZE);
          delay(100);
          WiFly.SendCommand("set wlan auth 4",">", bufBody, BODY_BUFFER_SIZE);
          delay(100);
          WiFly.SendCommand("set wlan channel 0",">", bufBody, BODY_BUFFER_SIZE);
          delay(100);
          WiFly.SendCommand("save ap",">", bufBody, BODY_BUFFER_SIZE);
          delay(1000);
          
          digitalWrite( STATUS_LED, LOW );
          runState = RUN_STATE_BOOT_PINS;
        }
      }
      
      delay(10);
      
      break;
    

     case RUN_STATE_CONFIG_IDENTIFY:
      ///////////////////////////
      //    CONFIG_IDENTIFY    //
      ///////////////////////////
      //Config mode indentify state
      //Flash the LED a few times upon request from the
      //configuring iOS app
      //Return to config_rnxv when done
      //Flash STATUS LED 5x
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(50);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(50);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(50);
      digitalWrite(STATUS_LED, LOW);
      delay(100);
      digitalWrite(STATUS_LED, HIGH);
      delay(50);
      digitalWrite(STATUS_LED, LOW);
      delay(10);
      
      runState = RUN_STATE_CONFIG_RNXV;
      
      break;
    
    case RUN_STATE_CONFIG_BTN_DOWN:
      /////////////////////////////
      //     CONFIG_BTN_DOWN     //
      /////////////////////////////
      //Mode Button has been pressed while handle is in config mode (Mode Button data pin is LOW)
      //In config mode, a mode button press reboots the tap handle
      
      digitalWrite(STATUS_LED, HIGH);
      
      //if btnPin goes to HIGH, the button was released
      //jump to boot_pins
      if(digitalRead(MODE_BUTTON_DATA) == HIGH)
      {
        counter=0;
        //turn off led as we exit button down state
        digitalWrite(STATUS_LED, LOW);
        runState = RUN_STATE_BOOT_PINS;
      }
      
      break;
      
      
      
    case RUN_STATE_RUN:
      ///////////////////////
      //        RUN        //
      ///////////////////////
      //Main operating loop
      //Watch inputs for activity:
      //Accel tilt: go to POUR_START
      //ID-12 SPI data available: go to BADGE_SCAN
      //Mode Button data pin to LOW: go to BTN_DOWN
      
      //Turn off status LED for running
      digitalWrite(STATUS_LED, LOW);
      
      //Serial.println("RUN");
      
      //Read Mode Button data pin state
      //If pressed, go to BTN_DOWN state
      if(digitalRead(MODE_BUTTON_DATA) == HIGH)
      {
        counter=0;
        btn_counter=0;
        runState = RUN_STATE_BTN_DOWN;
      }
      
      //Wait guard time before looping
      delay(10);
      break;
      
    case RUN_STATE_BTN_DOWN:
      //////////////////////
      //     BTN_DOWN     //
      //////////////////////
      //Mode Button has been pressed (Mode Button data pin is LOW)
      //Mode Button has 2 operations:
      //If held in for > 10 seconds, reboot in Config mode (state = ENTER_CONFIG_RNXV)
      //If released before 10 seconds have elapsed, go to STATUS mode
      counter++;
      btn_counter++;
      //turn on led when pressed
      digitalWrite(STATUS_LED, HIGH);
      Serial.println("BTNDOWN");
      //if btnPin goes to HIGH, the button was released before config mode kicked in
      //jump right to status
      if(digitalRead(MODE_BUTTON_DATA) == LOW)
      {
        counter=0;
        //turn off led as we exit button down state
        digitalWrite(STATUS_LED, LOW);
        runState = RUN_STATE_STATUS;
        
        //testing: button press triggers pour on
        //runState = RUN_STATE_POUR_START;
      }
      //if we make it to 180 on the counter before btnPin goes HIGH,
      //jump right to config mode
      if(btn_counter == 180)
      {
        //turn off led as we exit button down state
        digitalWrite(STATUS_LED, LOW);
        runState = RUN_STATE_ENTER_CONFIG_RNXV;
      }
      
      //Guard time before looping
      delay(10);
      break;
      
    case RUN_STATE_STATUS:
      ////////////////////////
      //       STATUS       //
      ////////////////////////
      //Show battery charge level via status LED blinks
      Serial.println("STATUS");
      //Status displaty routine
      
      //turn off the led for a bit
      digitalWrite(STATUS_LED, LOW);
      delay(200);
      
      //long blink
      digitalWrite(STATUS_LED, HIGH);
      delay(1000);
      digitalWrite(STATUS_LED, LOW);
      delay(200);
      
      //show battery status
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(200);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(200);
      digitalWrite(STATUS_LED, HIGH);
      delay(100);
      digitalWrite(STATUS_LED, LOW);
      delay(200);
      
      //long blink
      delay(500);
      digitalWrite(STATUS_LED, HIGH);
      delay(1000);
      digitalWrite(STATUS_LED, LOW);
      delay(200);
      
      //Return to RUN state
      runState = RUN_STATE_RUN;
      break;
      
    case RUN_STATE_POUR_START:
      //////////////////////////////
      //        POUR_START        //
      //////////////////////////////
      //Pour Start state
      //Pour has started
      //Start timer, advance to pour on
      Serial.println("Pour Start");
      
      pourStartTime = millis();
      
      runState = RUN_STATE_POUR_ON;
      break;
      
      
    case RUN_STATE_POUR_ON:
      ///////////////////////////
      //        POUR_ON        //
      ///////////////////////////
      //Pour On state
      //Pour is hapening now
      //Look for pour end
      Serial.println("Pour On");
      runState = RUN_STATE_POUR_END;
      
      delay(500);
      
      break;

    case RUN_STATE_POUR_END:
      ////////////////////////////
      //        POUR_END        //
      ////////////////////////////
      //Pour End state
      //Pour has ended
      //Log the pour against the database
      Serial.println("Pour End");
      
      pourEndTime = millis();
      pourDuration = pourEndTime - pourStartTime;
      req = "GET /taphandled/p.php?d=";
      req += pourDuration;
      req += " HTTP/1.0\n\n";
      Serial.println(req);
      char reqBuf[req.length()+1];
      req.toCharArray(reqBuf, req.length()+1);
      
      WiFly.flush();
      delay(100);
      
      boolean suc;
      //Open socket to host stored in RNXV on port 80
      //suc = WiFly.SendCommand("open 173.201.58.131 80","OPEN", bufBody, BODY_BUFFER_SIZE);
      //WiFly.SendCommand("open","OPEN", bufBody, BODY_BUFFER_SIZE);
      //suc = WiFly.openConnection("192.168.1.124", 5000);
      suc = WiFly.openConnection("173.201.58.131", 5000);
      delay(100);
      //WiFly.exitCommandMode();
      //delay(100);
      
      if(suc)
      {
        Serial.println("Open succeeded");
      }
      else
      {
        Serial.println("Open failed");
      }

      //Make and send HTTP Request to log pour
      //WiFly.write("GET /taphandled/p.php HTTP/1.0\n\n");
      WiFly.write(reqBuf);
      delay(100);
      
      //suc = WiFly.SendCommandSimple("close","CLOS");
      WiFly.closeConnection(true);
      delay(100);
      
      runState = RUN_STATE_TEST;
      Serial.print("TO_TEST");
      break;
      
     
    case RUN_STATE_TEST:
      
      /*
      while(WiFly.available() > 0) {
        Serial.write(WiFly.read());
      }
      */
      
      //Read Mode Button data pin state
      //If pressed, go to BTN_DOWN state
      if(digitalRead(MODE_BUTTON_DATA) == LOW)
      {
        counter=0;
        btn_counter=0;
        runState = RUN_STATE_BTN_DOWN;
      }
      
      break;
     
  }
  
}

