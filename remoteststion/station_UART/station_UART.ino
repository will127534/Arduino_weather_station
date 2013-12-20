#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <SPI.h>
#include <DHT22.h>
#include <BMP085.h>
#include <nRF24L01.h>
#include <Mirf.h>
#include "nRF24L01.h"
#include <util/atomic.h>
#include <EasyTransfer.h>
BMP085 dps = BMP085();
 ISR(WDT_vect) { watchdogEvent(); }
DHT22 myDHT22(9);

const unsigned char OSS = 3;  // Oversampling Setting
int serialValue;
int DHTPOEWR=8;
int dsAddress = 0x34;
int reading = 0; 
byte inByte;
int seconds;
int minutes;
int hours;
int t;
float v;
float h;
int csn_pin = 7;
EasyTransfer uart;

int Wirelesspower = 5; 

struct config_type
    {                     
      float humidity;
      float celsius;
      float ds18b20a;
      float vo;
      uint16_t lux;
      int32_t pressure;
      float mah;
      float ds18b20b;    
    };
config_type data;

void setup(){ 
  power_adc_disable();
  pinMode(DHTPOEWR, OUTPUT); 
  pinMode(13, OUTPUT); 
  digitalWrite(DHTPOEWR,HIGH);
//  digitalWrite(0,HIGH);     // Pull-UP unused input
 // digitalWrite(1,HIGH);     //8 9 13 6 10 SPI I2C
  pinMode(Wirelesspower,OUTPUT);
  digitalWrite(Wirelesspower,HIGH);
  digitalWrite(2,HIGH);
  digitalWrite(3,HIGH);
  digitalWrite(4,HIGH);
  digitalWrite(5,HIGH);
  digitalWrite(7,HIGH);
  digitalWrite(A0,HIGH);
  digitalWrite(A1,HIGH);
  digitalWrite(A2,HIGH);
  digitalWrite(A3,HIGH);
  Serial.begin(9600);
  uart.begin(details(data), &Serial);
  Wire.begin();
  delay(100);
  dps.init(MODE_ULTRA_HIGHRES, 0, true);
     data.humidity = 0;    // Intilize DATA 
     data.celsius=0;
     data.ds18b20a=0;
     data.vo=0;
     data.lux=0;
     data.pressure=0;
     data.mah=0;
     data.ds18b20b=0;
  digitalWrite(DHTPOEWR,LOW);

}

void loop(){
                                  
      digitalWrite(DHTPOEWR,HIGH);  
      readDHT2(); 
      digitalWrite(DHTPOEWR,LOW);
      
      dps.getPressure(&data.pressure);  
      
      data.vo = getdsVoltage();
      data.mah = getdsmah();
      
      digitalWrite(13, HIGH);
      digitalWrite(Wirelesspower,HIGH);
      uart.sendData();      
      digitalWrite(Wirelesspower,LOW);
      digitalWrite(13, LOW);

      loseSomeTime (30000);
     
}


static volatile byte watchdogCounter;

void watchdogInterrupts (char mode) {
    // correct for the fact that WDP3 is *not* in bit position 3!
    if (mode & bit(3))
        mode ^= bit(3) | bit(WDP3);
    // pre-calculate the WDTCSR value, can't do it inside the timed sequence
    // we only generate interrupts, no reset
    byte wdtcsr = mode >= 0 ? bit(WDIE) | mode : 0;
    MCUSR &= ~(1<<WDRF);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
    #ifndef WDTCSR
    #define WDTCSR WDTCR
    #endif
        WDTCSR |= (1<<WDCE) | (1<<WDE); // timed sequence
        WDTCSR = wdtcsr;
    }
}

/// @see http://www.nongnu.org/avr-libc/user-manual/group__avr__sleep.html
void powerDownn () {
    byte adcsraSave = ADCSRA;
    ADCSRA &= ~ bit(ADEN); // disable the ADC
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    ATOMIC_BLOCK(ATOMIC_FORCEON) {
        sleep_enable();
       // sleep_bod_disable(); // can't use this - not in my avr-libc version!
         #ifdef BODSE
        MCUCR = MCUCR | bit(BODSE) | bit(BODS); // timed sequence
        MCUCR = (MCUCR & ~ bit(BODSE)) | bit(BODS);
        #endif
    }
    sleep_cpu();
    sleep_disable();
    // re-enable what we disabled
    ADCSRA = adcsraSave;
}

byte loseSomeTime (word msecs) {
    byte ok = 1;
    word msleft = msecs;
    // only slow down for periods longer than the watchdog granularity
    while (msleft >= 16) {
        char wdp = 0; // wdp 0..9 corresponds to roughly 16..8192 ms
        // calc wdp as log2(msleft/16), i.e. loop & inc while next value is ok
        for (word m = msleft; m >= 32; m >>= 1)
            if (++wdp >= 9)
                break;
        watchdogCounter = 0;
        watchdogInterrupts(wdp);
        powerDownn();
        watchdogInterrupts(-1); // off
        // when interrupted, our best guess is that half the time has passed
        word halfms = 8 << wdp;
        msleft -= halfms;
        if (watchdogCounter == 0) {
            ok = 0; // lost some time, but got interrupted
            break;
        }
        msleft -= halfms;
    }
    // adjust the milli ticks, since we will have missed several

    extern volatile unsigned long timer0_millis;
    timer0_millis += msecs - msleft;

    return ok; // true if we lost approx the time planned
}

void watchdogEvent() {
    ++watchdogCounter;
}


void sendnrf(){
  Mirf.setTADDR((byte *)"serv1");
  
  Mirf.send((byte *)&data);
  
  while(Mirf.isSending()){
  }
  }

void readDHT2(){
  DHT22_ERROR_t errorCode;
  loseSomeTime (2050);
  errorCode = myDHT22.readData();
  if (errorCode ==  DHT_ERROR_NONE)
  {
      data.celsius = myDHT22.getTemperatureC();
      data.humidity =  myDHT22.getHumidity();
  }
}



void temp()   //Resister type tempeture reading 
{
  double a = analogRead(A3) * 5.0 / 1024.0;
  double x = (50000.0 / a) - 10000.0;
  double w = (1.0/((log(x/10000.0))/3950.0 + (1/288.15)))-273.15+10.0;//offset=10
  Serial.println(w);
} 


void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress);  
  Wire.write(address);  
  Wire.write(val);        
  Wire.endTransmission();     
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); 
  Wire.endTransmission();

  Wire.requestFrom(deviceAddress, 1); // read a byte

  while(!Wire.available()) {
    // waiting
  }

  v = Wire.read();
  return v;
}

void wakeUpNow()        
{
 
  
}

void sleepNow()         
{
  delay(100);
   Serial.println("Sleep Mode");
   delay(100);
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);   // sleep mode is set here
    sleep_enable();         
    attachInterrupt(0,wakeUpNow, LOW); 
    sleep_mode();         
                             // THE PROGRAM CONTINUES FROM HERE AFTER WAKING UP
    sleep_disable();        
    detachInterrupt(0);     
}



float getdsTemp() {
	
	Wire.beginTransmission(dsAddress);
	Wire.write(0x18);
	Wire.endTransmission();

	Wire.requestFrom(dsAddress, 2);
	if(2 <= Wire.available())    
	{ 
		reading = Wire.read();   
		reading = reading << 8;    
		reading += Wire.read();  
		reading = reading >> 5;
		reading = reading * 0.125;
                Serial.print(" Temp:");
		Serial.print(reading);    
		Serial.print("C ");
	}

	return reading;
}

float getdsVoltage(void) {
	int voltage;

	// Read Voltage Register
	Wire.beginTransmission(dsAddress);
	Wire.write(0x0C);
	Wire.endTransmission();

	Wire.requestFrom(dsAddress, 2);
	delay(4);
	if(2 <= Wire.available())     // if six bytes were received 
	{ 
		voltage = Wire.read();
		voltage = voltage << 8;
		voltage += Wire.read();
		voltage = voltage >> 5;
	  v = voltage * 4.88 / 1000;
	}
	return v;
}

float getdsCurrent(void) {
	int current;
	
	// Read Voltage Register
	Wire.beginTransmission(dsAddress);
	Wire.write(0x0E);
	Wire.endTransmission();

	Wire.requestFrom(dsAddress, 2);
	delay(4);
	if(2 <= Wire.available())     // if six bytes were received 
	{ 
	current = Wire.read();
        current = current << 8;
	current += Wire.read();
	current = current >> 3;
        double c = current * 0.625;
        return c;
        }	
}
	

float getdsmah(void) {
	int mah;
                                   
	Wire.beginTransmission(dsAddress);
	Wire.write(0x10);
	Wire.endTransmission();

	Wire.requestFrom(dsAddress, 2);
	delay(4);
	if(2 <= Wire.available())     
	{ 
		 mah = Wire.read();
                 mah = mah << 8;
                 mah += Wire.read();
                 h = mah * 0.25;
                 return h;
         }
}


int resetdsProtection(void) {
	int dsProtect;
	int dsStatus;
	
	Wire.beginTransmission(dsAddress);
	Wire.write(0x00);
	Wire.write(0x03);  //Clear OV and UV, enable charge and discharge
	//Wire.send(0x00);  //Clear OV and UV, diable charge and discharge
	Wire.endTransmission();
	delay(10);
}
void resetds() {
	Wire.beginTransmission(dsAddress);
	Wire.write(0x10);
	Wire.write(0x00);  
        Wire.write(0x00);  
	Wire.endTransmission(); 	
}


void print_bits(byte val){
  int i;
  for(i=7; i>=0; i--) 
    Serial.print(val >> i & 1, BIN);
}


