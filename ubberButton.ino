#include <LiquidCrystal.h>
#include <ubberFrame.h>
#include <SPI.h>
#include <RH_NRF24.h>

const int buttonPinPause = 2;     // the number of the pushbutton pin
const int buttonPinRepas = 3;

const int ledPin = A0;      // the number of the LED pin
const int DIPPin = A7;

//LCD
const int LCDRSPin = 7;
const int LCDEPin = 6;
const int LCDData0 = A1;
const int LCDData1 = A2;
const int LCDData2 = A3;
const int LCDData3 = A4;

// variables will change:
int buttonPause = 0;         // variable for reading the pushbutton status
int buttonRepas = 1;
int dip;

UbberFrame f;
UbberFrame *f_res;
RH_NRF24 nrf24;

void setup() {
// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCDRSPin, LCDEPin, LCDData3, LCDData2, LCDData1, LCDData0);
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
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
  lcd.begin(16, 2);
  setState("Begin");
    
}

int setState(const char * state)
{
  lcd.setCursor(0, 0);
  // print the number of seconds since reset:
  lcd.print(state);
}

int setStatus(const char *s)
{
  lcd.setCursor(0,1);
  lcd.print(s);
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

uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);
void loop(){
  // read the state of the pushbutton value:
  buttonPause = digitalRead(buttonPinPause);
  buttonRepas = digitalRead(buttonPinRepas);
  
  
 
 if( buttonPause == HIGH || buttonRepas == HIGH) {
   
   setState("Sending");
   setStatus("...");
   Serial.println("Begin send");
   dip = getDip();   
   
   digitalWrite(ledPin, HIGH);
   
   f.setSourceID(UbberFrame::GUILLAUME_L); 
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

  Serial.print("Sending ");
  Serial.print(f.getLength(), DEC);
  Serial.print("...");
  nrf24.send(f.frameToChar(), f.getLength());
  
  delay(1000);  
  Serial.println("DONE");
  setStatus("OK");
  digitalWrite(ledPin, LOW);
 }
 
 if(nrf24.available())
 {
   setState("Receiving");
   if (nrf24.recv(buf, &len)) {
     Serial.print("got request: ");
     Serial.println((char*)buf);
     
     f_res = new UbberFrame(buf, len);
  setStatus(f_res->getTypeString());   
     
     if(f_res->getDestID() == UbberFrame::GUILLAUME_L)
     {
       Serial.print("Get ");
       Serial.println(f_res->getTypeString());
     }     
   }   
 }
 
}
