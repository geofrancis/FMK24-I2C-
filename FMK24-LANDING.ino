#include <Wire.h>
#include <SoftwareSerial.h>
SoftwareSerial SoftSerial(10, 11);
int target = 0;
int range = 0;

uint16_t reading_cm;
uint16_t tempread;
void setup()
{
  
  SoftSerial.begin(57600);
  Serial.begin(115200);
  reading_cm=0; 
  Wire.begin(0x71);                // join i2c bus with address #2
  Wire.onRequest(requestEvent); // register event
}

void loop()
{
    target = SoftSerial.parseInt(); //dataIn now holds 0
    tempread = SoftSerial.parseInt(); //dataIn now holds 0

 // if (target ==  1){  
       reading_cm=tempread;  
    Serial.print("target");
    Serial.print(":");
    Serial.println(reading_cm);
//}
}
void requestEvent()
{
  byte sendhi;
  byte sendli;
  byte sendbyte[2];
  uint16_t tempreading_cm;
  tempreading_cm=reading_cm;
  sendhi=tempreading_cm>>8;
  sendli=tempreading_cm&0xff;
  sendbyte[0]=sendhi;
  sendbyte[1]=sendli;
  Wire.write(sendbyte,2); 
}
