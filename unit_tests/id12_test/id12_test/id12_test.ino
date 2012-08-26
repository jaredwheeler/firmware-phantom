#include <SoftwareSerial.h>

const int RUN_STATE_PRE       = 1001;
const int RUN_STATE_CONFIG    = 1002;
const int RUN_STATE_SETTLE    = 1003;
const int RUN_STATE_RUN       = 1004;
const int RUN_STATE_STATUS    = 1005;
const int RUN_STATE_BTNDOWN   = 1006;

const int MODE_BUTTON_IN      = 6;
const int MODE_BUTTON_OUT     = 2;

const int STATUS_LED          = 10;

const int ID12_RESET          = 7;
const int ID12_RX             = 8;
const int ID12_TX             = 9;

int runState;
int counter;
int btn_counter;

char tagString[13];
int index;
boolean reading;

SoftwareSerial id12SoftSerial(ID12_RX, ID12_TX);

void setup()
{
  runState = RUN_STATE_PRE;
  Serial.begin(9600);
  id12SoftSerial.begin(9600);
  
  pinMode(ID12_RESET, OUTPUT);
  digitalWrite(ID12_RESET, HIGH);

  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  pinMode(MODE_BUTTON_IN, INPUT);
  digitalWrite(MODE_BUTTON_IN, LOW);
  pinMode(MODE_BUTTON_OUT, OUTPUT);
  digitalWrite(MODE_BUTTON_OUT, HIGH);
}

void loop()
{
  switch (runState)
  {
    case RUN_STATE_PRE:
      
      resetReader(); //eset the RFID reader
      
      runState = RUN_STATE_RUN;
      counter = 0;
      Serial.println("PRE");
      break;
    case RUN_STATE_CONFIG:
      //runState = RUN_STATE_RUN;
      Serial.println("CONFIG");
      break;
    case RUN_STATE_RUN:
      //Serial.println("RUN");
      
      //Read status button state
      if(digitalRead(6) == LOW)
      {
        counter=0;
        btn_counter=0;
        runState = RUN_STATE_BTNDOWN;
      }
      
      //Check ID-12 for incoming serial data
      //char tagString[13];
      index = 0;
      reading = false;
      
      while(id12SoftSerial.available()){
        int readByte = id12SoftSerial.read(); //read next available byte
        Serial.write(readByte);
        //Serial.print('x');
        if(readByte == 2) reading = true; //begining of tag
        if(readByte == 3) reading = false; //end of tag
    
        if(reading && readByte != 2 && readByte != 10 && readByte != 13){
          //store the tag
          tagString[index] = readByte;
          index ++;
        }
      }
      
      checkTag(tagString); //Check if it is a match
      clearTag(tagString); //Clear the char of all value
      //resetReader(); //eset the RFID reader
      
      delay(10);
      break;
    case RUN_STATE_STATUS:
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
      
      //show connection status
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
      
      runState = RUN_STATE_RUN;
      break;
    case RUN_STATE_BTNDOWN:
      counter++;
      btn_counter++;
      //turn on led when pressed
      digitalWrite(STATUS_LED, HIGH);
      Serial.println("BTNDOWN");
      //if btnPin goes to HIGH, the button was released before config mode kicked in
      //jump right to status
      if(digitalRead(6) == HIGH)
      {
        counter=0;
        //turn off led as we exit button down state
        digitalWrite(STATUS_LED, LOW);
        runState = RUN_STATE_STATUS;
      }
      //if we make it to 180 on the counter before btnPin goes HIGH,
      //jump right to config mode
      if(btn_counter == 180)
      {
        //turn off led as we exit button down state
        digitalWrite(STATUS_LED, LOW);
        runState = RUN_STATE_CONFIG;
      }
      delay(10);
      break;
  }
  
}

void checkTag(char tag[]){
///////////////////////////////////
//Check the read tag against known tags
///////////////////////////////////

    //Serial.println(tag); //read out any unknown tag
}


void resetReader(){
///////////////////////////////////
//Reset the RFID reader to read again.
///////////////////////////////////
  digitalWrite(7, LOW);
  digitalWrite(7, HIGH);
  delay(150);
}

void clearTag(char one[]){
///////////////////////////////////
//clear the char array by filling with null - ASCII 0
//Will think same tag has been read otherwise
///////////////////////////////////
  for(int i = 0; i < strlen(one); i++){
    one[i] = 0;
  }
}

