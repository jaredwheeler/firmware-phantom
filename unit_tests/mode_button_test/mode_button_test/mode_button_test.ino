const int RUN_STATE_PRE       = 1001;
const int RUN_STATE_CONFIG    = 1002;
const int RUN_STATE_SETTLE    = 1003;
const int RUN_STATE_RUN       = 1004;
const int RUN_STATE_STATUS    = 1005;
const int RUN_STATE_BTNDOWN   = 1006;

const int MODE_BUTTON_IN      = 3;
const int MODE_BUTTON_OUT     = 2;
const int MODE_BUTTON_DATA    = 10;

const int STATUS_LED          = 13;

int runState;
int counter;
int btn_counter;

void setup()
{
  runState = RUN_STATE_PRE;
  Serial.begin(9600);


  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW);
  pinMode(MODE_BUTTON_IN, INPUT);
  digitalWrite(MODE_BUTTON_IN, LOW);
  pinMode(MODE_BUTTON_OUT, OUTPUT);
  digitalWrite(MODE_BUTTON_OUT, HIGH);
  pinMode(MODE_BUTTON_DATA, INPUT);
  digitalWrite(MODE_BUTTON_DATA, LOW);
}

void loop()
{
  switch (runState)
  {
    case RUN_STATE_PRE:
      runState = RUN_STATE_RUN;
      counter = 0;
      Serial.println("PRE");
      break;
    case RUN_STATE_CONFIG:
      //runState = RUN_STATE_RUN;
      Serial.println("CONFIG");
      break;
    case RUN_STATE_RUN:
      Serial.println("RUN");
      if(digitalRead(MODE_BUTTON_IN) == HIGH)
      {
        counter=0;
        btn_counter=0;
        runState = RUN_STATE_BTNDOWN;
        Serial.println("MODE DATA HIGH");
      }
      else
      {
        Serial.println("MODE DATA LOW");
      }
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
      if(digitalRead(MODE_BUTTON_IN) == LOW)
      {
        counter=0;
        //turn off led as we exit button down state
        digitalWrite(STATUS_LED, LOW);
        //runState = RUN_STATE_STATUS;
        runState = RUN_STATE_RUN;
      }
      /*
      //if we make it to 180 on the counter before btnPin goes HIGH,
      //jump right to config mode
      if(btn_counter == 180)
      {
        //turn off led as we exit button down state
        digitalWrite(STATUS_LED, LOW);
        runState = RUN_STATE_CONFIG;
      }
      */
      delay(10);
      break;
  }
  
}
