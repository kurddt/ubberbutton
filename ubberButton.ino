#include <LiquidCrystal.h>
#include <ubberFrame.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <RH_ASK.h>
#include <Wire.h>

#define MSG_OK "OK              "

typedef enum {
  WAITING,
  SENDING,
  RECEIVED,
  FOROTHER,
  SENT,
} state_t; 

const int buttonPinPause = 2;     // the number of the pushbutton pin
const int buttonPinRepas = 3;
const int buttonPinReset = A6;

      // the number of the LED pin
const int ledPinR= 9;
const int DIPPin = A7;
const int LEDExtIOAddr = 0x38;

//LCD
const int LCDRSPin = 7;
const int LCDEPin = 6;
const int LCDData0 = A0;
const int LCDData1 = A1;
const int LCDData2 = A2;
const int LCDData3 = A3;

// variables will change:
int buttonPause = 0;         // variable for reading the pushbutton status
int buttonRepas = 1;
int buttonReset = 0;
int dip;

//433
const int d433_rx = 4;
const int d433_tx = 5;

UbberFrame f;
RH_NRF24 nrf24;

RH_ASK driver(2000, d433_rx, d433_tx, A6, false);

state_t state = WAITING;
state_t old_state = WAITING;

const int me = UbberFrame::GUILLAUME_L;


// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCDRSPin, LCDEPin, LCDData3, LCDData2, LCDData1, LCDData0);


unsigned long time;
unsigned long lastStateChangeTime = 0;

void changeState(state_t new_state)
{
  state = new_state;
  lastStateChangeTime = millis();
}

unsigned long timeSinceStateChange()
{
	return millis() - lastStateChangeTime;
}

static void SetExtIO(int bit_no, int val)
{
  static uint8_t IOval = -1;
  
  if(val) IOval |= 1 << bit_no;
  else IOval &= ~(1 << bit_no);
  
  Wire.beginTransmission(LEDExtIOAddr); // transmit to device #4
  Wire.write(IOval);              // sends one byte setState 
  Wire.endTransmission();    // stop transmitting
  
}

static void setLED(int val)
{
  SetExtIO(0, !val);
}

int setDisplayState(const char * state)
{
  lcd.setCursor(0, 0);
  lcd.clear();
  // print the number of seconds since reset:
  lcd.print(state);
}

int setStatus(const char *s)
{
  lcd.setCursor(0,1);
  lcd.print(s);
}

int setStatus(const char *s1, const char *s2)
{
    char toto[30];
    strcpy(toto, s1);
    strcat(toto, "-");
    strcat(toto, (const char *)s2);
    setStatus(toto);    
}

void setup() {  

  // initialize the LED pin as an output:
  pinMode(ledPinR, OUTPUT); 
  // initialize the pushbutton pin as an input:
  pinMode(buttonPinPause, INPUT);     
  pinMode(buttonPinRepas, INPUT);
  pinMode(DIPPin, INPUT);


  Serial.begin(9600);


  if(!nrf24.init())
    Serial.println("nrf init failed");
  if(!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate2Mbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
  if(!driver.init()) {
    Serial.println("433 init failed");
  }  
  Wire.begin(); // join i2c bus (address optional for master)
  analogReference(EXTERNAL);
  lcd.begin(16, 2);
  setDisplayState("Begin");
  changeState(WAITING);

}






int getDip()
{
  int value = analogRead(DIPPin);
  Serial.println(value & 0xFFF8, HEX);
  switch(value & 0xFFF8)
  {
  case 0:
    return 0xff;
  case 0x2D8:
    return 0x1;
  case 0x360:
    return 0x3;
  case 0x2A8:
    return 0x2;
  case 0x338:
    return 0x6;
  case 0x920:
    return 0x7;
  case 0x260:
    return 4;
  default:
    return 0xff;
  }
}

static int send(UbberFrame & frame)
{
  nrf24.send(frame.frameToChar(), frame.getLength());
  driver.send(frame.frameToChar(), frame.getLength());
  driver.waitPacketSent();
  nrf24.waitPacketSent();
   
  return 0;
}

void clear()
{
  lcd.clear();
  if(f.getSourceID() != me && f.getType() != UbberFrame::ACQUITTEMENT)  {
    f.setType(UbberFrame::ACQUITTEMENT);
    f.setDestID(f.getSourceID());
    f.setSourceID(me);
    send(f);
  }
  setDisplayState("Waiting");
  digitalWrite(ledPinR, LOW);
  setLED(0);
  changeState(WAITING);
}

uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

int nrf_received = 0;
int r433_received = 0;
int count;


static int ledblue = 0;
static int ledred = 1;

void loop(){
  // read the state of the pushbutton value:
  buttonPause = digitalRead(buttonPinPause);
  buttonRepas = digitalRead(buttonPinRepas);
  buttonReset = analogRead(A6);
  
  if(buttonReset > 256)
    clear();
            
   //uint8_t addr = nrf24.spiReadRegister(0x3);
   //Serial.println(addr);
   static int toto = 0;
   if(toto < 3 ) {
     nrf24.printRegisters();
     toto++;
   }
 
   if( (buttonPause == HIGH || buttonRepas == HIGH) && state != SENDING) {
	changeState(SENDING);
    Serial.println("Begin send");
    dip = getDip();   

    f.setSourceID(me); 
    if(buttonPause == HIGH)
      f.setType(UbberFrame::PAUSE);
    else if(buttonRepas == HIGH)
      f.setType(UbberFrame::REPAS);
    else {
      Serial.println("Error");
      return;
    }

    Serial.println(f.getTypeString());

    Serial.print("DIP: ");
    Serial.println(dip, HEX);

    f.setDestID(dip);
    Serial.println(f.getDestIDString());
    setStatus(f.getDestIDString(), f.getTypeString());

    Serial.print("Sending ");
    Serial.print(f.getLength(), DEC);
    Serial.print("...");
    
    send(f);
    Serial.println("DONE");
    setLED(0);
  }
  
  nrf_received = nrf24.available();
  r433_received = driver.recv(buf, &len);
  len = sizeof(buf);
  if(nrf_received || r433_received )
  {  

    if(nrf_received) Serial.println("Receiving from NRF");
    if(r433_received) Serial.println("Receiving from 433");
    int dest;
    
    if (r433_received || nrf24.recv(buf, &len)) {
      int i =0;
      Serial.print("got request: ");
      for(i=0;i<len;i++) {
        Serial.print(buf[i], HEX); Serial.print(" ");
      }

      if(f.frameFromChar(buf, len)) {
        Serial.println("Bad frame");
      }
      dest = f.getDestID();
      if((dest == UbberFrame::GUILLAUME_L || dest == UbberFrame::ALL))
      {
        Serial.print("Get ");
        Serial.println(f.getTypeString());
		changeState(RECEIVED);
      } else {
		  changeState(FOROTHER);
	  }	  
    }  
  }

  if(state == SENDING && timeSinceStateChange() > 1000) {
	  changeState(SENT);
  } else if(state == RECEIVED) {
	  unsigned long time = millis();	  
	  static unsigned long lastime = time;
	  if(time - lastime > 500) {
		  lastime = time;
		  ledblue = !ledblue;
		  ledred = !ledred;

		  setLED(ledblue);
		  digitalWrite(ledPinR, ledred);
	  }	  
  }

  if(old_state != state) { // UPDATE display
	  if(state == WAITING) {
		  lcd.clear();
		  setDisplayState("Waiting");	
	  }
	  else if(state == SENDING) {
		  setLED(1);
		  setDisplayState("Sending");
		  setStatus("...");
	  }
	  else if(state == SENT) {
		  setStatus(MSG_OK);
		  setLED(0);
	  }
	  else if(state == RECEIVED) {
		  setDisplayState(f.getSourceIDString());
		  setStatus(f.getTypeString());
	      digitalWrite(ledPinR, ledred);
		  setLED(ledblue);
		  
	  }
	  else if(state == FOROTHER) {
		  setDisplayState("others");
		  setStatus("for other");
	  }	  
  }

  // State switch
  old_state = state;
  
}

