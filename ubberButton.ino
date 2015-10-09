#include <ubberFrame.h>
#include <SPI.h>
#include <RH_NRF24.h>

const int buttonPinPause = 2;     // the number of the pushbutton pin
const int buttonPinRepas = 3;

const int buttonPinDIP0 = 4;
const int buttonPinDIP1 = 5;
const int ledPin = A0;      // the number of the LED pin

// variables will change:
int buttonPause = 0;         // variable for reading the pushbutton status
int buttonRepas = 1;
int dip;

UbberFrame f;
RH_NRF24 nrf24;

void setup() {
  // initialize the LED pin as an output:
  pinMode(ledPin, OUTPUT);      
  // initialize the pushbutton pin as an input:
  pinMode(buttonPinPause, INPUT);     
  pinMode(buttonPinRepas, INPUT);
  
  pinMode(buttonPinDIP0, INPUT);
  pinMode(buttonPinDIP1, INPUT);
  
  Serial.begin(9600);
  
  
  if(!nrf24.init())
    Serial.println("nrf init failed");
  if(!nrf24.setChannel(1))
    Serial.println("setChannel failed");
  if (!nrf24.setRF(RH_NRF24::DataRate250kbps, RH_NRF24::TransmitPower0dBm))
    Serial.println("setRF failed");
    
}

int getDip()
{
  return (digitalRead(buttonPinDIP0) == HIGH) | ( ( digitalRead(buttonPinDIP1) == HIGH) << 1);
}


void loop(){
  // read the state of the pushbutton value:
  buttonPause = digitalRead(buttonPinPause);
  buttonRepas = digitalRead(buttonPinRepas);
  
  digitalWrite(ledPin, HIGH);
 
 if( buttonPause == HIGH || buttonRepas == HIGH) {
   
   Serial.println("Begin send");
   dip = getDip();
   
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
  //digitalWrite(ledPin, LOW);
 }
}
