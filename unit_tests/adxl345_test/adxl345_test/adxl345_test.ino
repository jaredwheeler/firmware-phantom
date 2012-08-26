#include <Wire.h>
#include <ADXL345.h>

ADXL345 adxl; //variable adxl is an instance of the ADXL345 library

const int RUN_STATE_PRE       = 1001;
const int RUN_STATE_CONFIG    = 1002;
const int RUN_STATE_SETTLE    = 1003;
const int RUN_STATE_RUN       = 1004;
const int RUN_STATE_STATUS    = 1005;
const int RUN_STATE_BTNDOWN   = 1006;

int runState;
int counter;

int xl_raw_x,xl_raw_y,xl_raw_z;
float xl_filt_x,xl_filt_y,xl_filt_z;
float xl_diff_x,xl_diff_y,xl_diff_z;
float xl_base_x,xl_base_y,xl_base_z;
float xl_dif_mag;

void setup()
{
  runState = RUN_STATE_CONFIG;
  analogReference(INTERNAL);
  Serial.begin(9600);
}

void loop()
{
  byte interrupts = adxl.getInterruptSource();
  switch (runState)
  {
    case RUN_STATE_PRE:
      runState = RUN_STATE_RUN;
      counter = 0;
      break;
    case RUN_STATE_CONFIG:
    //set up accel
      adxl.powerOn();
      /*
      adxl.setActivityThreshold(15); //62.5mg per increment
      //adxl.setActivityAc(true);
      adxl.setInactivityThreshold(75); //62.5mg per increment
      adxl.setTimeInactivity(10); // how many seconds of no activity is inactive?
     
      //look of activity movement on this axes - 1 == on; 0 == off 
      adxl.setActivityX(1);
      adxl.setActivityY(1);
      adxl.setActivityZ(1);
     
      //look of inactivity movement on this axes - 1 == on; 0 == off
      adxl.setInactivityX(1);
      adxl.setInactivityY(1);
      adxl.setInactivityZ(1);
      
      //setting all interupts to take place on int pin 1
      //I had issues with int pin 2, was unable to reset it
      adxl.setInterruptMapping( ADXL345_INT_SINGLE_TAP_BIT,   ADXL345_INT1_PIN );
      adxl.setInterruptMapping( ADXL345_INT_DOUBLE_TAP_BIT,   ADXL345_INT1_PIN );
      adxl.setInterruptMapping( ADXL345_INT_FREE_FALL_BIT,    ADXL345_INT1_PIN );
      adxl.setInterruptMapping( ADXL345_INT_ACTIVITY_BIT,     ADXL345_INT1_PIN );
      adxl.setInterruptMapping( ADXL345_INT_INACTIVITY_BIT,   ADXL345_INT1_PIN );
     
      //register interupt actions - 1 == on; 0 == off  
      adxl.setInterrupt( ADXL345_INT_SINGLE_TAP_BIT, 1); 
      adxl.setInterrupt( ADXL345_INT_DOUBLE_TAP_BIT, 1);
      adxl.setInterrupt( ADXL345_INT_FREE_FALL_BIT,  1);
      adxl.setInterrupt( ADXL345_INT_ACTIVITY_BIT,   1);
      adxl.setInterrupt( ADXL345_INT_INACTIVITY_BIT, 1);
      */
      runState = RUN_STATE_SETTLE;
      break;
    case RUN_STATE_SETTLE:
      counter++;
      if(counter < 200)
      {
        adxl.readAccel(&xl_raw_x, &xl_raw_y, &xl_raw_z); //read the accelerometer values and store them in variables  x,y,z
        xl_filt_x += ((float)xl_raw_x - xl_filt_x)*0.1;
        xl_filt_y += ((float)xl_raw_y - xl_filt_y)*0.1;
        xl_filt_z += ((float)xl_raw_z - xl_filt_z)*0.1;
        Serial.print(xl_filt_x);
        Serial.print(',');
        Serial.print(xl_filt_y);
        Serial.print(',');
        Serial.println(xl_filt_z);
      }
      else
      {
        counter = 0;
        //use the low-passed data to set the base orientation vals
        xl_base_x = xl_filt_x;
        xl_base_y = xl_filt_y;
        xl_base_z = xl_filt_z;
        runState = RUN_STATE_RUN;
      }
      delay(10);
      break;
    case RUN_STATE_RUN:
      
      
      if(adxl.triggered(interrupts, ADXL345_ACTIVITY))
      {
        //Serial.println("activity"); 
         //add code here to do when activity is sensed
      }
        
      adxl.readAccel(&xl_raw_x, &xl_raw_y, &xl_raw_z); //read the accelerometer values and store them in variables  x,y,z
      xl_filt_x += ((float)xl_raw_x - xl_filt_x)*0.1;
      xl_filt_y += ((float)xl_raw_y - xl_filt_y)*0.1;
      xl_filt_z += ((float)xl_raw_z - xl_filt_z)*0.1;
      // Output x,y,z values - Commented out
      /*
      Serial.print((float)xl_raw_x);
      Serial.print(',');
      Serial.print((float)xl_raw_y);
      Serial.print(',');
      Serial.print((float)xl_raw_z);
      Serial.print(' ');
      Serial.print(xl_filt_x);
      Serial.print(',');
      Serial.print(xl_filt_y);
      Serial.print(',');
      Serial.println(xl_filt_z);
      */
      //Get the magnitude of the vector from the base orientation to the current, filtered orientation
      //If that magnitude is greater than the threshold, we are tilted
      xl_diff_x = xl_filt_x - xl_base_x;
      xl_diff_y = xl_filt_y - xl_base_y;
      xl_diff_z = xl_filt_z - xl_base_z;
      xl_dif_mag = sqrt( xl_diff_x*xl_diff_x + xl_diff_y*xl_diff_y + xl_diff_z*xl_diff_z );
      Serial.println( xl_dif_mag );
      delay(100);
      break;
    case RUN_STATE_STATUS:
      
      break;
    case RUN_STATE_BTNDOWN:
      
      break;
  }
}
