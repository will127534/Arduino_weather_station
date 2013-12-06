#include <SPI.h>
#include <Mirf.h>
#include <nRF24L01.h>
#include <MirfHardwareSpiDriver.h>

struct config_type
 {                     // 16 bytes to send to the Server (on 4 byte boundries)
float humidity;
float celsius;
int finalTempC;
float vo;
uint16_t lux;
int32_t pressure;
float mah;
float kPressure;            
} ;
 config_type data;
void setup(){
  pinMode(13,OUTPUT);
Serial.begin(9600);
Mirf.spi = &MirfHardwareSpi;
Mirf.init();
Mirf.setRADDR((byte *)"serv1");
Mirf.payload = sizeof(data);
Serial.print(Mirf.payload);
Mirf.config();
Serial.println("Listening..."); 
}

void loop(){
 
  if(!Mirf.isSending() && Mirf.dataReady()){
    digitalWrite(13,HIGH);
  // Serial.println("G");
   Mirf.getData((byte *)&data);
   Serial.print(data.humidity);
   Serial.print(',');
   Serial.print(data.celsius);
   Serial.print(',');
   //Serial.println(data.finalTempC);
   // Serial.println(data.vo);
   // Serial.println(data.lux);
   Serial.print(data.pressure);
   Serial.print(',');
   Serial.print(data.vo);
   Serial.print(',');
   Serial.println(data.mah);
    digitalWrite(13,LOW);
   //  Serial.println(data.mah);
  }
}
