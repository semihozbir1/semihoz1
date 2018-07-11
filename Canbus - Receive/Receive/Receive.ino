#include <Arduino.h>
#include <TimerOne.h>
#include <PID_v1.h>
#include <Adafruit_GFX.h>
#include "Adafruit_LEDBackpack.h"
#include "mcp_can.h"
#include <SPI.h>
#include <stdio.h>

Adafruit_AlphaNum4 alpha4 = Adafruit_AlphaNum4();


//CANBUS DEĞİŞKENLERİ
#define INT8U unsigned char
const int SPI_CS_PIN = 10;
MCP_CAN CAN(SPI_CS_PIN);

INT8U len = 0;
INT8U buf[8];
unsigned char canId;
char str[20];



//MOTOR DEĞİŞKENLERİ
int hallsensor1 = 3;
int hallsensor2 = 7;
int hallsensor3 = 8;
int Motor_Out = 5;
int limit_sw_pin = 6;


//RPM HESAPLAMA DEĞİŞKENLERİ
const int numreadings = 10;
int readings[numreadings];
unsigned long average = 0;
int index = 0;
unsigned long total;

unsigned long rpm = 0;
volatile int rpm_tur_sayisi = 0;
volatile int devir_sayisi = 0;
int tur_sayisi = 0;
float kablo_boyu = 0;
bool direction ;
bool limit_sw ;
int motor_position = 0;
int potvalue = 0;
int buttonPushCounter_4 = 0;


//PID DEĞİŞKENLERİ
double Setpoint, Input, Output;
double Kp=0.009, Ki=0.02, Kd=0.002;


PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);



void setup() {
  Serial.begin(115200);
  attachInterrupt(digitalPinToInterrupt(hallsensor1), rpm_olcum, HIGH);
  Timer1.initialize(100000);
  Timer1.attachInterrupt(rpm_yaz); 
  myPID.SetMode(AUTOMATIC);
  alpha4.begin(0x70);           // Adafruit Display Başlatma
  alpha4_intro();
  alpha4.clear(); 
   while (CAN_OK != CAN.begin(CAN_500KBPS))              // canbus baudrate 500kbps
    {
        Serial.println("CAN BUS Başlatma Hatası!!!");
        Serial.println("Tekrar Dene...");
        delay(100);
    }

    Serial.println("CAN BUS Başlatma Başarılı!");

  
 
}



void rpm_olcum(){

int hall1 = digitalRead(hallsensor1);
int hall2 = digitalRead(hallsensor2);
int hall3 = digitalRead(hallsensor3);


            if (hall2 && !hall3){
        rpm_tur_sayisi = rpm_tur_sayisi + 1 ;
        }
        
            if (hall1 && !hall2 && hall3){
        motor_position = 1;
        }
           if (!hall1 && hall2 && !hall3){
        motor_position = 1;
        }
           if (hall1 && hall2 && !hall3){
        motor_position = 0;
        }
          if (!hall1 && !hall2 && hall3){
        motor_position = 0;
        }
           
    if (direction){
    devir_sayisi = devir_sayisi + 1;
    tur_sayisi = devir_sayisi/4;
      }
    else if (!direction){
  devir_sayisi = devir_sayisi - 1;
  tur_sayisi = devir_sayisi/4;

  }
  
}

void rpm_yaz(){

detachInterrupt(0);    
 total = 0;
                                            // RPM Ortalama Hesplama
 readings[index] = rpm_tur_sayisi * 300;  

 for (int x=0; x<=9; x++){
   total = total + readings[x];
 }
 average = total / numreadings;
 rpm = average;
 rpmOrtala();
 rpm_tur_sayisi = 0; 
 index++;
 if(index >= numreadings){
  index=0;
 }
  attachInterrupt(digitalPinToInterrupt(hallsensor1), rpm_olcum, HIGH);
  }


void rpmOrtala(){
  
int rpm_ortala = rpm;
int rpmortala_0 = (rpm_ortala % 10);
int rpmortala_1 = (rpm_ortala / 10) % 10;
int rpmortala_2 = (rpm_ortala /100) % 10;
int rpmortala_3 = (rpm_ortala / 1000) % 10;
int rpmortala_4 = (rpm_ortala / 10000) % 10;

if (rpmortala_1 < 5){
rpm = rpmortala_0 + (rpmortala_2 * 100) + (rpmortala_3 * 1000) + (rpmortala_4 * 10000);
}
else if (rpmortala_1 >= 5){
rpm = rpmortala_0 + ((rpmortala_2 + 1) * 100) + (rpmortala_3 * 1000) + (rpmortala_4 * 10000);
} 
}




void loop() {

  while (CAN_MSGAVAIL == CAN.checkReceive())      // CanBus Gelen Veri Kontrolü
        {
      CAN.readMsgBuf(&len, buf);
      canId = CAN.getCanId(); 
      potvalue = buf[0];                 // CanBus Gelen Veriyi Değişkene Atama
      direction = buf[1];              // CanBus Gelen Veriyi Değişkene Atama
      buttonPushCounter_4 = buf[2];   // CanBus Gelen Veriyi Değişkene Atama
        }
        
        limit_sw = digitalRead(limit_sw_pin);
        
Setpoint = map(potvalue,0,250,0,5050) ;
SetpointOrtala();
//Setpoint = 2520 ;
if (Setpoint<300) Setpoint=0;

kablo_boyu = (tur_sayisi) * (0.015);
kablo_limit_reverse();
kablo_limit_forward();
switch_display();


 Serial.print("    Yön= "); Serial.print(motor_position);
 Serial.print(" ---  Setpoint = "); Serial.print(Setpoint);
 Serial.print("  ---  RPM = "); Serial.print(rpm);
 Serial.print("  ---   Output: "); Serial.print(Output);
 Serial.print("   ---  Kablo Boyu : "); Serial.print(kablo_boyu); Serial.println(" metre ");
 

delay(50);

 }

 void SetpointOrtala(){
    int setpoint_ortala = Setpoint;
    int setpointortala_0 = (setpoint_ortala % 10);
    int setpointortala_1 = (setpoint_ortala / 10) % 10;
    int setpointortala_2 = (setpoint_ortala /100) % 10;
    int setpointortala_3 = (setpoint_ortala / 1000) % 10;
    int setpointortala_4 = (setpoint_ortala / 10000) % 10;
    
    if (setpointortala_1 < 5){
    Setpoint = (setpointortala_2 * 100) + (setpointortala_3 * 1000) + (setpointortala_4 * 10000);
    }
    else if (setpointortala_1 >= 5){
    Setpoint = ((setpointortala_2 + 1) * 100) + (setpointortala_3 * 1000) + (setpointortala_4 * 10000);
    }
  
 }



void cable_limit_operation(){
  
analogWrite(Motor_Out, 40);
  if (!limit_sw){
    
    analogWrite(Motor_Out, 40);
  }
  else if (limit_sw){
    analogWrite(Motor_Out,0);
    tur_sayisi = 0;
    kablo_boyu = 0;
    devir_sayisi = 0;
    Setpoint = 0;
    Output = 0;
    }
   
}

void kablo_limit_reverse(){       // Koblo Boyuna Göre Limit Kontrolleri
if (kablo_boyu > 2.50 && kablo_boyu <= 72.50 && !direction){
    Input = rpm ;
    myPID.SetOutputLimits(0,255);
    myPID.Compute();
    if (Setpoint < 300) Output = 0;
    analogWrite(Motor_Out, Output);
}
else if (kablo_boyu > 72.50 && !direction){ 
    Input = rpm ;
    myPID.SetOutputLimits(0,55);
    myPID.Compute();
    if (Setpoint < 300) Output = 0;
    analogWrite(Motor_Out, Output);
}
  
  else if (kablo_boyu >= 1.00 && kablo_boyu <= 2.50 && !direction){
    Input = rpm ;
    myPID.SetOutputLimits(0,255);
    myPID.Compute();
    if (Output >= 50) Output = 50;
    analogWrite(Motor_Out, Output);
  
}

  else if (kablo_boyu < 1.00 && !direction) {
    cable_limit_operation();
  }
      else if (kablo_boyu <= 2.50 && direction && Setpoint > 1500 ) {
    Setpoint = 1500;
    
  }


}


   void kablo_limit_forward(){        // Koblo Boyuna Göre Limit Kontrolleri

  if (kablo_boyu < 2.50 && direction){
    Input = rpm ;
    myPID.SetOutputLimits(0,55);
    myPID.Compute();
    if (Setpoint < 300)Output = 0;
    analogWrite(Motor_Out, Output);
  }
  
   if (kablo_boyu < 72.50 && kablo_boyu >= 2.50 && direction){
    Input = rpm ;
    myPID.SetOutputLimits(0,255);
    myPID.Compute();
    if (Setpoint < 300)Output = 0;
    analogWrite(Motor_Out, Output);
  }
  
  
  
  else if (kablo_boyu <= 74.96 && kablo_boyu >= 72.50 && direction){
   //analogWrite(Motor_Out,50);
    Input = rpm ;
    myPID.SetOutputLimits(0,255);
    myPID.Compute();
    if (Output >=50 )Output = 50;
    if (Setpoint < 400)Output = 0;
    analogWrite(Motor_Out, Output);
}
  else if (kablo_boyu > 74.96 && direction) {
    analogWrite(Motor_Out,0);
    Output = 0;
    Setpoint = 0;
  }
  else if (kablo_boyu >= 72.50 && !direction && Setpoint > 1500 ) {
    Setpoint = 1500;
    
  }

}





void switch_display(){        // Buton İle Göstergeleri Değiştirme
  
  if (buttonPushCounter_4 == 1) print_rpm();
  else if (buttonPushCounter_4 == 0) print_kablo_boyu();
  else if (buttonPushCounter_4 == 2) alpha4_warning_reverse();
  else if (buttonPushCounter_4 == 3) alpha4_warning_forward();
  else return;
}



void print_rpm(){     

int print_rpm = rpm;
int rpm_0 = (print_rpm % 10);
int rpm_1 = (print_rpm / 10) % 10;
int rpm_2 = (print_rpm /100) % 10;
int rpm_3 = (print_rpm / 1000) % 10;
int rpm_4 = (print_rpm / 10000) % 10;

if (print_rpm == 0){
   alpha4.writeDigitAscii(0, 'S');
  alpha4.writeDigitAscii(1, 'T');
  alpha4.writeDigitAscii(2, 'O');
  alpha4.writeDigitAscii(3, 'P');  
}


else if (print_rpm >0 && print_rpm <1000){
  alpha4.writeDigitAscii(0, ' ');
  alpha4.writeDigitAscii(1, '0' + rpm_2);
  alpha4.writeDigitAscii(2, '0' + rpm_1);
  alpha4.writeDigitAscii(3, '0' + rpm_0);  
}
else if (print_rpm >= 1000 && print_rpm <=9999){
  alpha4.writeDigitAscii(0, '0' + rpm_3);
  alpha4.writeDigitAscii(1, '0' + rpm_2);
  alpha4.writeDigitAscii(2, '0' + rpm_1);
  alpha4.writeDigitAscii(3, '0' + rpm_0);  
  
}
else if  (print_rpm > 9999 && print_rpm <15000 && rpm_3 ==0){
  alpha4.writeDigitAscii(0, '0' + rpm_4);
  alpha4.writeDigitRaw(1, 0x403F);
  alpha4.writeDigitAscii(2, '0' + rpm_2);
  alpha4.writeDigitAscii(3, 'K');  
} 

else if  (print_rpm > 9999 && print_rpm <15000 && rpm_3 ==1){
  alpha4.writeDigitAscii(0, '0' + rpm_4);
  alpha4.writeDigitRaw(1, 0x4006);
  alpha4.writeDigitAscii(2, '0' + rpm_2);
  alpha4.writeDigitAscii(3, 'K');  
} 

else{
  alpha4.writeDigitAscii(0, ' ');
  alpha4.writeDigitAscii(1, 'E');
  alpha4.writeDigitAscii(2, 'r');
  alpha4.writeDigitAscii(3, 'r');
  
}

  alpha4.writeDisplay();
  delay(200);
  
}



void print_kablo_boyu(){
  
int print_kablo_boyu = kablo_boyu * 100.00;
int kablo_0 = (print_kablo_boyu % 10);
int kablo_1 = (print_kablo_boyu / 10) % 10;
int kablo_2 = (print_kablo_boyu /100) % 10;
int kablo_3 = (print_kablo_boyu /1000) % 10;

//Serial.print("kablo 0 : ");
//Serial.print(kablo_0);
//Serial.print("  --  kablo 1 : ");
//Serial.print(kablo_1);
//Serial.print(" --  kablo 2 :  ");
//Serial.print(kablo_2);
//Serial.print(" -- kablo 3 :  ");
//Serial.println(kablo_3);


 if (kablo_0 < 0 ){
    alpha4.writeDigitRaw(0, 0x403F);
    alpha4.writeDigitAscii(1,'0');
    alpha4.writeDigitAscii(2,'0');    
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 0){
    alpha4.writeDigitRaw(0, 0x403F);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 1){
    alpha4.writeDigitRaw(0, 0x4006);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 2){
    alpha4.writeDigitRaw(0, 0x40DB);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 3){
    alpha4.writeDigitRaw(0, 0x408F);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 4){
    alpha4.writeDigitRaw(0, 0x40E6);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 5){
    alpha4.writeDigitRaw(0, 0x40ED);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 6){
    alpha4.writeDigitRaw(0, 0x40FD);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 7){
    alpha4.writeDigitRaw(0, 0x4007);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 8){
    alpha4.writeDigitRaw(0, 0x40FF);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}
  else if (kablo_3 == 0 && kablo_2 == 9){
    alpha4.writeDigitRaw(0, 0x40EF);
    alpha4.writeDigitAscii(1,'0' + kablo_1);  
    alpha4.writeDigitAscii(2, '0' + kablo_0);  
    alpha4.writeDigitAscii(3, 'm');  
}

// ikinci haneye geçerse


  else if (kablo_3 > 0 && kablo_2 == 0){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x403F);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}


else if (kablo_3 > 0 && kablo_2 == 1){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x4006);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}

else if (kablo_3 > 0 && kablo_2 == 2){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x40DB);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}

else if (kablo_3 > 0 && kablo_2 == 3){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x408F);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}

else if (kablo_3 > 0 && kablo_2 == 4){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x40E6);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}

else if (kablo_3 > 0 && kablo_2 == 5){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x40ED);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}

else if (kablo_3 > 0 && kablo_2 == 6){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x40FD);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}

else if (kablo_3 > 0 && kablo_2 == 7){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x4007);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}

else if (kablo_3 > 0 && kablo_2 == 8){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x40FF);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}

else if (kablo_3 > 0 && kablo_2 == 9){
    alpha4.writeDigitAscii(0,'0' + kablo_3);
    alpha4.writeDigitRaw(1, 0x40EF);
    alpha4.writeDigitAscii(2,'0' + kablo_1);    
    alpha4.writeDigitAscii(3, 'm');  
}


  else {
    alpha4.writeDigitAscii(0, ' ');
    alpha4.writeDigitAscii(1, 'E');
    alpha4.writeDigitAscii(2, 'r');
    alpha4.writeDigitAscii(3, 'r');  
  }

  alpha4.writeDisplay();
  delay(200);
  
}

void alpha4_intro(){
  alpha4.writeDigitRaw(3, 0x0);
  alpha4.writeDigitRaw(0, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(0, 0x0);
  alpha4.writeDigitRaw(1, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(1, 0x0);
  alpha4.writeDigitRaw(2, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(2, 0x0);
  alpha4.writeDigitRaw(3, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.clear();
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(0, 0x0);
  alpha4.writeDigitRaw(3, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(3, 0x0);
  alpha4.writeDigitRaw(2, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(2, 0x0);
  alpha4.writeDigitRaw(1, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.writeDigitRaw(1, 0x0);
  alpha4.writeDigitRaw(0, 0xFFFF);
  alpha4.writeDisplay();
  delay(200);
  alpha4.clear();
  alpha4.writeDisplay();
  delay(1000);
}



void alpha4_warning_reverse(){


    alpha4.writeDigitAscii(0, 'H');
    alpha4.writeDigitAscii(1, '-');
    alpha4.writeDigitAscii(2, '0');
    alpha4.writeDigitAscii(3, '0');  
    alpha4.writeDisplay();
//char *message = "   ( KABLO BOYU MINIMUM )       ";
//  
//  for (uint8_t i=0; i<strlen(message)-4; i++) {
//    alpha4.writeDigitAscii(0, message[i]);
//    alpha4.writeDigitAscii(1, message[i+1]);
//    alpha4.writeDigitAscii(2, message[i+2]);
//    alpha4.writeDigitAscii(3, message[i+3]);
//    alpha4.writeDisplay();
//    delay(200);
//  }
    
    
  }

  void alpha4_warning_forward(){

    alpha4.writeDigitAscii(0, '*');
    alpha4.writeDigitAscii(1, '*');
    alpha4.writeDigitAscii(2, '*');
    alpha4.writeDigitAscii(3, '*'); 
    alpha4.writeDisplay();
    
//char *message = "   ( KABLO BOYU MAKSIMUM )       ";
//  
//  for (uint8_t i=0; i<strlen(message)-4; i++) {
//    alpha4.writeDigitAscii(0, message[i]);
//    alpha4.writeDigitAscii(1, message[i+1]);
//    alpha4.writeDigitAscii(2, message[i+2]);
//    alpha4.writeDigitAscii(3, message[i+3]);
//    alpha4.writeDisplay();
//     delay(200);
//  }  
  }
  



   
