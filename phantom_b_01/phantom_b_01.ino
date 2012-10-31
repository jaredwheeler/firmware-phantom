#include <Arduino.h>
#include <Streaming.h>
#include <SoftwareSerial.h>
#include <ADXL345.h>
#include <Wire.h>
#include "WiFlySerial.h"
//#include "MemoryFree.h"
//#include "Credentials.h"

//Declare internal run state stuff
char runState;
int counter;
int btn_counter;
int pour_counter;

//holder for iterator int
int i;

//Declare run state constants
const char RUN_STATE_BOOT_PINS           = 101;
const char RUN_STATE_BOOT_RNXV           = 102;
const char RUN_STATE_BOOT_ADXL           = 103;
const char RUN_STATE_BOOT_MAX17043       = 104;
const char RUN_STATE_BOOT_ID12           = 105;
const char RUN_STATE_ENTER_CONFIG_RNXV   = 106;
const char RUN_STATE_CONFIG_RNXV         = 107;
const char RUN_STATE_CONFIG_IDENTIFY     = 108;
const char RUN_STATE_CONFIG_BTN_DOWN     = 109;
const char RUN_STATE_RUN                 = 110;
const char RUN_STATE_BTN_DOWN            = 111;
const char RUN_STATE_STATUS              = 112;
const char RUN_STATE_POUR_START          = 113;
const char RUN_STATE_POUR_ON             = 114;
const char RUN_STATE_POUR_END            = 115;
const char RUN_STATE_BADGE_SCAN          = 116;
const char RUN_STATE_FATAL_ERROR         = 117;
const char RUN_STATE_TEST                = 118;

//Declare pin constants
const int MODE_BUTTON_DATA    = 3;
const int MODE_BUTTON_PWR     = 2;
const int STATUS_LED          = 10;
const int RNXV_RX             = 4;
const int RNXV_TX             = 5;

//Declare RNXV Object and I/O buffers
WiFlySerial WiFly(RNXV_RX, RNXV_TX);
#define REQUEST_BUFFER_SIZE 120
#define HEADER_BUFFER_SIZE 150 
#define BODY_BUFFER_SIZE 100
char bufRequest[REQUEST_BUFFER_SIZE];
char bufHeader[HEADER_BUFFER_SIZE];
char bufBody[BODY_BUFFER_SIZE];
#define BODY_BUFFER_SIZE 100
//String SPIInputString;
String CONFString;
char s;

//Stuf to hold Config process data
char startIndex;
char endIndex;
String confStr;
String ssidStr;
char ssidBuf[64];
String phraseStr;
char phraseBuf[64];
String hostStr;
char hostBuf[32];

//buffer to hold http request
char reqBuf[80];
boolean conx_suc; //hold the result of a connection attempt

//Declare ADXL stuff
ADXL345 adxl; //variable adxl is an instance of the ADXL345 library
int xl_raw_x,xl_raw_y,xl_raw_z; //The raw input from the ADXL
float xl_cur_x,xl_cur_y,xl_cur_z; //The current value of the tilt vector
float xl_diff_x,xl_diff_y,xl_diff_z; //
float xl_base_x,xl_base_y,xl_base_z;
float xl_raw_mag; //magnitude of vector from last frame's vector -> vector of raw accel input
float xl_mag_clip_ratio; //ratio of raw_mag to acceptable dif_mag
//float xl_dif_mag; 
float xl_offset_mag; //magnitude of vector from base vector -> clipped, low-passed current vector

//Declare Max17043 Fuel Gauge stuff
#define MAX17043_ADDRESS 0x36  // R/W =~ 0x6D/0x6C
unsigned int soc;
int maxIntBuf; //all reads from fuel gauge end up in here
float batPercentage;

//Start and end times for pour measuring
int pourStartTime;
int pourEndTime;
int pourDuration;
String req;

void setup()
{
  digitalWrite(STATUS_LED, HIGH);
  runState = RUN_STATE_BOOT_PINS;
  analogReference(INTERNAL);
  Serial.begin(9600);
  //Serial.println("SETUP( )");
}


void loop()
{
  switch (runState)
  {
    case RUN_STATE_BOOT_PINS:
      bootPins();
      break;
    case RUN_STATE_BOOT_RNXV:
      bootRNXV();
      break;
    
    case RUN_STATE_BOOT_ADXL:
      bootADXL();
      break;
      
    case RUN_STATE_BOOT_MAX17043:
      bootMax17043();
      break;
    
    case RUN_STATE_ENTER_CONFIG_RNXV:
      enterConfigRNXV();
      break;
       
    case RUN_STATE_CONFIG_RNXV:
      configRNXV();
      break;
      
    case RUN_STATE_RUN:
      run();
      break;
      
    case RUN_STATE_BTN_DOWN:
      btnDown();
      break;
      
    case RUN_STATE_STATUS:
      showStatus();
      break;
      
    case RUN_STATE_POUR_START:
      pourStart();
      break;
      
    case RUN_STATE_POUR_ON:
      pourOn();
      break;

    case RUN_STATE_POUR_END:
      pourEnd();
      break;
  }
  
}

void bootPins()
{
  /////////////////////
  //    BOOT PINS    //
  /////////////////////
  //Energize and set I/O mode for all pins
  //Serial.println("BOOT_PINS");
  //Serial.println(freeMemory());

  
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  pinMode(MODE_BUTTON_DATA, INPUT);
  digitalWrite(MODE_BUTTON_DATA, LOW);
  pinMode(MODE_BUTTON_PWR, OUTPUT);
  digitalWrite(MODE_BUTTON_PWR, HIGH);
  
  runState = RUN_STATE_BOOT_RNXV;
}

void bootRNXV()
{
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
  //Serial.println("BOOT_RNXV");
  //Serial.println(freeMemory());
  
  //Flash STATUS LED 2x
  digitalWrite(STATUS_LED, HIGH);
  delay(500);
  
  WiFly.begin();
  //Serial.println("WiFly UP");
  
  delay(1000);
  
  WiFly.setRemotePort(80);
  
  char load_ap_str[8] = "load ap";
  char save_str[5] = "save";
  char reboot_str[7] = "reboot";
  char join_str[5] = "join";

  WiFly.SendCommand(load_ap_str,">", bufBody, BODY_BUFFER_SIZE);
  delay(1000);
  WiFly.SendCommand(save_str,">", bufBody, BODY_BUFFER_SIZE); 
  delay(1000);
  WiFly.SendCommand(reboot_str,"Ver", bufBody, BODY_BUFFER_SIZE);
  delay(3000);
  WiFly.SendCommand(join_str,">", bufBody, BODY_BUFFER_SIZE);
  delay(3000);

  analogWrite(STATUS_LED, 150);
  //Serial.println("getDeviceStatus");
  //Serial.println(freeMemory());
  
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
  delay(100);
}

void bootADXL()
{
  /////////////////////
  //    BOOT ADXL    //
  /////////////////////
  //Boot sequence for the ADXL345 accelerometer
  //Energize the accel.
  //wait 5 seconds for orientation vals to settle into low-pass filter
  
  analogWrite(STATUS_LED, 80);
  delay(100);
  
  //Turn on ADXL Lib
  adxl.powerOn();
  delay(100);
  
  
  //Loop for 3 seconds then use the current state of the accel as the base vector
  for(i=0;i<120;i++)
  {
    readADXL();
    delay(10);
  }
  xl_base_x = xl_cur_x;
  xl_base_y = xl_cur_y;
  xl_base_z = xl_cur_z;
  
  //Serial.println("BOOT_ADXL");
  runState = RUN_STATE_BOOT_MAX17043;
}

void bootMax17043()
{
  /////////////////////////
  //    BOOT MAX17043    //
  /////////////////////////
  //Boot sequence for the MAX17043 battery fuel gauge
  //Energize and set up initial state of fuel gauge
  
  analogWrite(STATUS_LED, 30);
  delay(100);
  
  configMAX17043();  // Configure the MAX17043's alert percentage
  delay(300);
  qsMAX17043();  // restart fuel-gauge calculations
  delay(300);
  
  //Serial.println("BOOT_MAX17043");
  runState = RUN_STATE_RUN;
}

void enterConfigRNXV()
{
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
  digitalWrite(STATUS_LED, LOW);
  delay(100);
  
  //Serial.println("ENTER_CONFIG_RNXV");
  //Serial.println(freeMemory());
  
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
  //Serial.println(freeMemory());
}

void configRNXV()
{
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
  
  
  //Check the RNXV buffer for any new bytes that have arrived over the socket connection
  //Push them into the SPIInputString input buffer
  String capture = "_";
  boolean newData = false;
  while(WiFly.available() > 0)
  {
    s = WiFly.read();
    delay(10);
    capture.concat(s);
    delay(10);
    newData = true;
    //Serial.print(s);
  }
  
  
  if(newData == true)
  {
    
    Serial.println(capture);
    Serial.print("^-");
    Serial.print(capture.length());
    Serial.println("-^");
    
    /*
    //Write any serial data sent into the Arduio out to the WiFly
    //(via the SerialMonitor)
    if(Serial.available()) { // Outgoing data
      WiFly.write( Serial.read() );
    }
    */
    
    if(capture.length() > 2 && capture.indexOf("ECONF") != -1)
    {
      if((capture.indexOf("BCONF") != -1))
      {
        CONFString = capture.substring(capture.indexOf("BCONF"));
        handleCONF();
      }
    }
    else if(capture.length() > 2 && capture.indexOf("CNX") != -1)
    {
      capture = "_";
      handleCNX();
    }
  }
  
  delay(10);
}

void run()
{
  ///////////////////////
  //        RUN        //
  ///////////////////////
  //Main operating loop
  //Watch inputs for activity:
  //Accel tilt: go to POUR_START
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
  
  //Read the accelerometer
  readADXL();
  if(xl_offset_mag > 50)
  {
    runState = RUN_STATE_POUR_START;
  }
  
  //Wait guard time before looping
  delay(10);
}

void btnDown()
{
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
  //Serial.println("BTNDOWN");
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
}

void showStatus()
{
  ////////////////////////
  //       STATUS       //
  ////////////////////////
  //Show battery charge level via status LED blinks
  Serial.println("STATUS");
  //Status displaty routine
  
  //Grab batt values from Max17043
  percentMAX17043();
  delay(10);
  //Serial.print(batPercentage, 2);  // Print the battery percentage
  //Serial.println(" %");
  
  //turn off the led for a bit
  digitalWrite(STATUS_LED, LOW);
  delay(200);
  
  //blink once for every 10% of battery charge remaining
  for(i=0; i<(int)batPercentage; i+=10)
  {
    digitalWrite(STATUS_LED, HIGH);
    delay(100);
    digitalWrite(STATUS_LED, LOW);
    delay(100);
  }
  
  //Return to RUN state
  runState = RUN_STATE_RUN;
}

void pourStart()
{
  //////////////////////////////
  //        POUR_START        //
  //////////////////////////////
  //Pour Start state
  //Pour has started
  //Start timer, advance to pour on
  //Serial.println("Pour Start");
  
  //pourStartTime = millis();
  
  digitalWrite(STATUS_LED, HIGH);
  
  pour_counter = 0;
  runState = RUN_STATE_POUR_ON;
  delay(10);
}

void pourOn()
{
  ///////////////////////////
  //        POUR_ON        //
  ///////////////////////////
  //Pour On state
  //Pour is hapening now
  //Look for pour end
  //Serial.println("Pour On");
  
  pour_counter++;
  //read ADXL
  readADXL();
  //if offset_mag drops below threshold, end pour
  if(xl_offset_mag < 20)
  {
    runState = RUN_STATE_POUR_END;
  }
  
  delay(10);
}

void pourEnd()
{
  ////////////////////////////
  //        POUR_END        //
  ////////////////////////////
  //Pour End state
  //Pour has ended
  //Log the pour against the database
  //Serial.println("Pour End");
  digitalWrite(STATUS_LED, LOW);
  
  //pourEndTime = millis();
  //pourDuration = pourEndTime - pourStartTime;
  req = "GET /taphandled/p.php?d=";
  req += pour_counter;
  req += " HTTP/1.0\n\n";
  //Serial.println(req);
  //char reqBuf[req.length()+1];
  req.toCharArray(reqBuf, req.length()+1);
  
  WiFly.flush();
  delay(100);
  
  //Open socket to host stored in RNXV on port 80
  conx_suc = WiFly.openConnection("173.201.58.131", 5000);
  //conx_suc = WiFly.openConnection("192.168.11.3", 80);
  delay(100);
  //WiFly.exitCommandMode();
  //delay(100);
  //Serial.println(conx_suc);
  
  //Make and send HTTP Request to log pour
  //WiFly.write("GET /taphandled/p.php HTTP/1.0\n\n");
  WiFly.write(reqBuf);
  delay(100);
  
  analogWrite(STATUS_LED, 50);
  delay(100);
  digitalWrite(STATUS_LED, LOW);
  //suc = WiFly.SendCommandSimple("close","CLOS");
  //WiFly.closeConnection(true);
  runState = RUN_STATE_RUN;
  delay(10);
}

///////////////////////////////////////
//        CONFIG MODE STUFF          //
///////////////////////////////////////
//Functions for dealing with config mode input
void handleCNX()
{
  Serial.println("handleCNX");
  WiFly.flush();
  delay(100);
  //This iOS device is requesting a connection check
  //respond with 'CNX'
  WiFly.write("CNX");
  delay(100);
}

void handleCONF()
{
  Serial.println("handleCONF");
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
  startIndex = CONFString.indexOf("BCONF") + 5;
  endIndex = CONFString.indexOf("ECONF");
  confStr = CONFString.substring(startIndex, endIndex);
  //Serial.println("\n"+confStr);
  //SPIInputString = " ";
  delay(100);
  
  //pick the props out of the config string one by one starting at the end:
  
  //grab ssid
  endIndex = confStr.length() - 1;
  startIndex = confStr.lastIndexOf("+", endIndex - 1) + 1;
  hostStr = "set ip host " + confStr.substring(startIndex, endIndex);
  hostStr.toCharArray(hostBuf, hostStr.length()+1);
  
  //grab password
  endIndex = startIndex - 1;
  startIndex = confStr.lastIndexOf("+", endIndex - 1) + 1;
  phraseStr = "set wlan phrase " + confStr.substring(startIndex, endIndex);
  phraseStr.toCharArray(phraseBuf, phraseStr.length()+1);
  
  //grab pourdb host
  endIndex = startIndex - 1;
  startIndex = confStr.lastIndexOf("+", endIndex - 1) + 1;
  ssidStr = "set wlan ssid " + confStr.substring(startIndex, endIndex);
  ssidStr.toCharArray(ssidBuf, ssidStr.length()+1);
  
  delay(100);
  //Push all the commands out to the RNXV
  WiFly.SendCommand(ssidBuf,">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  WiFly.SendCommand(phraseBuf,">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  WiFly.SendCommand(hostBuf,">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  WiFly.SendCommand("set ip dhcp 1",">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  WiFly.SendCommand("set wlan join 0",">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  WiFly.SendCommand("set wlan auth 4",">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  WiFly.SendCommand("set wlan channel 0",">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  /*
  WiFly.SendCommand("set ip proto 2",">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  
  WiFly.SendCommand("set ip address 0",">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  WiFly.SendCommand("set ip remote 80",">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  WiFly.SendCommand("set com remote 0",">", bufBody, BODY_BUFFER_SIZE);
  delay(100);
  */
  WiFly.SendCommand("save ap",">", bufBody, BODY_BUFFER_SIZE);
  delay(1000);
  
  //clear out the used char buffers
  memset(ssidBuf,0,sizeof(ssidBuf));
  memset(phraseBuf,0,sizeof(phraseBuf));
  memset(hostBuf,0,sizeof(hostBuf));
  CONFString = "_";
  
  digitalWrite( STATUS_LED, LOW );
  runState = RUN_STATE_BOOT_PINS;
}


/////////////////////////////////////////
//        ADXL345 Accelerometer        //
/////////////////////////////////////////
//Functions for controlling the accelerometer
void readADXL()
{
  //read the accelerometer values and store them in variables  x,y,z
  adxl.readAccel(&xl_raw_x, &xl_raw_y, &xl_raw_z);
  //make a magnitude for the difference vector from the current tilt to raw ADXL input
  xl_diff_x = xl_raw_x - xl_cur_x;
  xl_diff_y = xl_raw_y - xl_cur_y;
  xl_diff_z = xl_raw_z - xl_cur_z;
  xl_raw_mag = sqrt( xl_diff_x*xl_diff_x + xl_diff_y*xl_diff_y + xl_diff_z*xl_diff_z );
  
  //clip if raw_mag > threshold value
  if(xl_raw_mag > 18)
  {
    //make clip ratio
    xl_mag_clip_ratio = 18 / xl_raw_mag;
    //multiply diffs by ratio
    xl_diff_x *= xl_mag_clip_ratio;
    xl_diff_y *= xl_mag_clip_ratio;
    xl_diff_z *= xl_mag_clip_ratio;
  }
  
  //low pass the clipped dif vector
  xl_diff_x *= 0.2;
  xl_diff_y *= 0.2;
  xl_diff_z *= 0.2;
  
  //add low passed vector to current tilt vector
  xl_cur_x += xl_diff_x;
  xl_cur_y += xl_diff_y;
  xl_cur_z += xl_diff_z;
  
  
  xl_offset_mag = sqrt( (xl_cur_x - xl_base_x)*(xl_cur_x - xl_base_x) + (xl_cur_y - xl_base_y)*(xl_cur_y - xl_base_y) + (xl_cur_z - xl_base_z)*(xl_cur_z - xl_base_z) );
  
  //Serial.println((int)xl_offset_mag);
  /*
  Serial.print(", ");
  Serial.print(xl_cur_x);
  Serial.print(", ");
  Serial.print(xl_cur_y);
  Serial.print(", ");
  Serial.println(xl_cur_z);
  */
  
}


///////////////////////////////////////
//        MAX17043 Fuel Gauge        //
///////////////////////////////////////
//Functions for controlling the battery fuel gauge
/*
percentMAX17043() returns a float value of the battery percentage
reported from the SOC register of the MAX17043.
*/
void percentMAX17043()
{ 
  i2cRead16(0x04);  // Read SOC register of MAX17043
  batPercentage = (byte) (maxIntBuf >> 8);  // High byte of SOC is percentage
  batPercentage += ((float)((byte)maxIntBuf))/256;  // Low byte is 1/256%
}

/* 
configMAX17043(byte percent) configures the config register of
the MAX170143, specifically the alert threshold therein. Pass a 
value between 1 and 32 to set the alert threshold to a value between
1 and 32%. Any other values will set the threshold to 32%.
void configMAX17043(byte percent)
{
  if ((percent >= 32)||(percent == 0))  // Anything 32 or greater will set to 32%
    i2cWrite16(0x9700, 0x0C);
  else
  {
    byte percentBits = 32 - percent;
    i2cWrite16((0x9700 | percentBits), 0x0C);
  }
}
MODIFIED - removed argument, hard-coded to 32% 10/21/12
*/
void configMAX17043()
{
  i2cWrite16(0x9700, 0x0C);
}

/* 
qsMAX17043() issues a quick-start command to the MAX17043.
A quick start allows the MAX17043 to restart fuel-gauge calculations
in the same manner as initial power-up of the IC. If an application's
power-up sequence is very noisy, such that excess error is introduced
into the IC's first guess of SOC, the Arduino can issue a quick-start
to reduce the error.
*/
void qsMAX17043()
{
  i2cWrite16(0x4000, 0x06);  // Write a 0x4000 to the MODE register
}

/* 
i2cRead16(unsigned char address) reads a 16-bit value beginning
at the 8-bit address, and continuing to the next address. A 16-bit
value is returned.
unsigned int i2cRead16(unsigned char address)
{
  int data = 0;
  
  Wire.beginTransmission(MAX17043_ADDRESS);
  Wire.write(address);
  Wire.endTransmission();
  
  Wire.requestFrom(MAX17043_ADDRESS, 2);
  while (Wire.available() < 2);
  data = ((int) Wire.read()) << 8;
  data |= Wire.read();
  
  return data;
}
MODIFIED - changed data to a top-level buffer
*/
void i2cRead16(unsigned char address)
{
  Wire.beginTransmission(MAX17043_ADDRESS);
  delay(10);
  Wire.write(address);
  delay(10);
  Wire.endTransmission();
  delay(10);
  
  Wire.requestFrom(MAX17043_ADDRESS, 2);
  delay(10);
  while (Wire.available() < 2);
  maxIntBuf = ((int) Wire.read()) << 8;
  delay(10);
  maxIntBuf |= Wire.read();
  delay(10);
}

/*
i2cWrite16(unsigned int data, unsigned char address) writes 16 bits
of data beginning at an 8-bit address, and continuing to the next.
*/
void i2cWrite16(unsigned int data, unsigned char address)
{
  Wire.beginTransmission(MAX17043_ADDRESS);
  delay(10);
  Wire.write(address);
  delay(10);
  Wire.write((byte)((data >> 8) & 0x00FF));
  delay(10);
  Wire.write((byte)(data & 0x00FF));
  delay(10);
  Wire.endTransmission();
  delay(10);
}
