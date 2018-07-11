
//Canbus Veri Gönderme

#include <Arduino.h>
#include <mcp_can.h>
#include <SPI.h>


const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);


int sensorPin = A0;
int btnPin = 4;
int dsp_sw_pin = 3;




int dsp_sw_Value = 0;
int sensorValue = 0;
int cantxValue = 0;
int btnValue = 0;

int buttonPushCounter = 0;
int buttonPushCounter_4 ;    
int buttonState = 0;         
int lastButtonState = 0;  


void setup()
{
  pinMode(8, INPUT_PULLUP);
  Serial.begin(115200);


  while (CAN_OK != CAN.begin(CAN_500KBPS))              // baudrate 500kbps
  {
      Serial.println("CAN BUS Shield init fail");
      Serial.println("Init CAN BUS Shield again");
      delay(100);
  }
}


//Örnek Mesaj Taslakları
unsigned char msg1[8] = {0, 1, 2, 3, 4, 5, 6, 7};
unsigned char msg2[8] = {0xFF, 0x01, 0x10, 0x0A, 0x00, 0x00, 0x00, 0x00};
unsigned char msg3[4] = {0xFF, 0x01, 0x10, 0x0A};


void loop()
{
  sensorValue = analogRead(sensorPin);
  btnValue = digitalRead(btnPin);
  
  buttonState = digitalRead(dsp_sw_pin);

   if (buttonState != lastButtonState) {
    if (buttonState == HIGH) {
      buttonPushCounter++;
      buttonPushCounter_4  = buttonPushCounter % 4;
    } else {}
    delay(50);
  }
  lastButtonState = buttonState;

  
  cantxValue = map(sensorValue,0,1025,0,250);
  
//  Serial.print("cantxValue: ");
//  Serial.print(cantxValue);
//  Serial.print("   btnValue: ");
//  Serial.print(btnValue);
//  Serial.print("   Dsp_sw_Value: ");
//  Serial.println(buttonPushCounter_4);

  //CanBus ile gönderilecek veri paketini oluşturma
  unsigned char canMsg[8] = {cantxValue, btnValue, buttonPushCounter_4, 0x00, 0x00, 0x00, 0x00, 0x00};

  //Veri Gönderme:  id = 0x07B, standart Flame, veri uzunluğu = 8, stmp:data buf
  CAN.sendMsgBuf(0x07B, 0, 8, canMsg);
  delay(100);
}
