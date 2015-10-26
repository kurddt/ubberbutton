#include <LiquidCrystal.h>
#include <ubberFrame.h>
#include <SPI.h>
#include <RH_NRF24.h>
#include <RH_ASK.h>

#define MSG_OK "OK              "

const int buttonPinPause = 2;     // the number of the pushbutton pin
const int buttonPinRepas = 3;
const int buttonPinReset = A6;

const int ledPin = A0;      // the number of the LED pin
const int ledPinR= 9;
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
int buttonReset = 0;
int dip;

//433
const int d433_rx = 4;
const int d433_tx = 5;

UbberFrame f;
UbberFrame *f_res;
RH_NRF24 nrf24;

RH_ASK driver(2000, d433_rx, d433_tx, A6, false);

// initialize the library with the numbers of the interface pins
LiquidCrystal lcd(LCDRSPin, LCDEPin, LCDData3, LCDData2, LCDData1, LCDData0);

void setup() {  

  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);     
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

  analogReference(EXTERNAL);
  lcd.begin(16, 2);
  setState("Begin");

}

int setState(const char * state)
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

void clear()
{
  lcd.clear();
  setState("Waiting");
  digitalWrite(ledPinR, LOW);
}

uint8_t buf[RH_NRF24_MAX_MESSAGE_LEN];
uint8_t len = sizeof(buf);

int nrf_received = 0;
int r433_received = 0;
int count;


void loop(){
  // read the state of the pushbutton value:
  buttonPause = digitalRead(buttonPinPause);
  buttonRepas = digitalRead(buttonPinRepas);
  buttonReset = analogRead(A6);
  
  if(buttonReset > 256)
    clear();
    
 
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
    setStatus(f.getDestIDString(), f.getTypeString());

    Serial.print("Sending ");
    Serial.print(f.getLength(), DEC);
    Serial.print("...");
    nrf24.send(f.frameToChar(), f.getLength());
    driver.send(f.frameToChar(), f.getLength());
    driver.waitPacketSent();

    delay(1000);  
    Serial.println("DONE");
    setStatus(MSG_OK);
    digitalWrite(ledPin, LOW);
  }
  
  nrf_received = nrf24.available();
  len = sizeof(buf);
  r433_received = driver.recv(buf, &len);
  if(nrf_received || r433_received)
  {
    digitalWrite(ledPinR, HIGH);
    setState("Receiving");
    setStatus("...");
    if(nrf_received) Serial.println("Receiving from NRF");
    if(r433_received) Serial.println("Receiving from 433");
    int dest;
    
    if (r433_received || nrf24.recv(buf, &len)) {
      int i =0;
      Serial.print("got request: ");
      for(i=0;i<len;i++) {
        Serial.print(buf[i], HEX); Serial.print(" ");
      }

      f_res = new UbberFrame(buf, len);
      dest = f_res->getDestID();
      if(dest == UbberFrame::GUILLAUME_L || dest == UbberFrame::ALL)
      {
        Serial.print("Get ");
        Serial.println(f_res->getTypeString());
      }     
    }
    setState(f_res->getSourceIDString());
  
    delete f_res;
  }
}

