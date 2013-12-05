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
   Serial.println("G");
   Mirf.getData((byte *)&data);
   Serial.println(data.humidity);
   Serial.println(data.celsius);
   //Serial.println(data.finalTempC);
   // Serial.println(data.vo);
   // Serial.println(data.lux);
   Serial.println(data.pressure);
   //  Serial.println(data.mah);
  }
}
