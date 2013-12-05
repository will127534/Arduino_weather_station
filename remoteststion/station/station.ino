#include <avr/sleep.h>
#include <avr/power.h>
#include <Wire.h>
#include <SPI.h>
#include <math.h> 
#include <DHT22.h>
#include <BMP085.h>
#include <nRF24L01.h>
#include <Mirf.h>
#include "nRF24L01.h"
#include <RF24.h>
#include <MirfHardwareSpiDriver.h>
#include <util/atomic.h>
BMP085 dps = BMP085();
 ISR(WDT_vect) { watchdogEvent(); }
int serialValue;
DHT22 myDHT22(9);
const unsigned char OSS = 0;  // Oversampling Setting

#define DS00OV	0x80
#define DS00UV	0x40
#define DS00COC	0x20
#define DS00DOC	0x10
#define DS00CC	0x08
#define DS00DC	0x04
#define DS00CE	0x02
#define DS00DE	0x01

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
pinMode(DHTPOEWR, OUTPUT); 
pinMode(13, OUTPUT); 
digitalWrite(DHTPOEWR,HIGH);
  Mirf.csnPin = 6;
  Mirf.cePin = 10;
  Mirf.spi = &MirfHardwareSpi;
  Mirf.init();
  Mirf.setRADDR((byte *)"clie1");
  Mirf.payload = sizeof(data);
  Mirf.config();
  
  Wire.begin();
  delay(100);
  dps.init(MODE_ULTRA_HIGHRES, 0, true);
     data.humidity = 0;
     data.celsius=0;
     data.finalTempC=0;
     data.vo=0;
     data.lux=0;
     data.pressure=0;
     data.mah=0;
     data.kPressure=0;
  digitalWrite(DHTPOEWR,LOW);

}

void loop(){
      digitalWrite(DHTPOEWR,HIGH);
      readDHT2(); //2104
      digitalWrite(DHTPOEWR,LOW);
      dps.getPressure(&data.pressure);
  //    Serial.println(data.pressure);
      
      digitalWrite(13, HIGH);
      sendnrf();
      digitalWrite(13, LOW);
       loseSomeTime (100);
  
      powerDown2();
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
  Serial.println("Finished sending");
  }
void readDHT2(){
  DHT22_ERROR_t errorCode;
  loseSomeTime (2100);
  errorCode = myDHT22.readData();
  if (errorCode ==  DHT_ERROR_NONE)
  {
      data.celsius = myDHT22.getTemperatureC();
    //  Serial.println(data.celsius);
      data.humidity =  myDHT22.getHumidity();
      //Serial.println(data.humidity);
  }
}
void powerDown2()
{
  write_register(CONFIG,read_register(CONFIG) & ~_BV(PWR_UP));
}

uint8_t write_register(uint8_t reg, uint8_t value)
{
uint8_t status;
csn(LOW);
status = SPI.transfer( W_REGISTER | ( REGISTER_MASK & reg ) );
SPI.transfer(value);
csn(HIGH);
return status;
}

uint8_t read_register(uint8_t reg)
{
  csn(LOW);
  SPI.transfer( R_REGISTER | ( REGISTER_MASK & reg ) );
  uint8_t result = SPI.transfer(0xff);

  csn(HIGH);
  return result;
}

void csn(int mode)
{
  // Minimum ideal SPI bus speed is 2x data rate
  // If we assume 2Mbs data rate and 16Mhz clock, a
  // divider of 4 is the minimum we want.
  // CLK:BUS 8Mhz:2Mhz, 16Mhz:4Mhz, or 20Mhz:5Mhz
#ifdef ARDUINO
  SPI.setBitOrder(MSBFIRST);
  SPI.setDataMode(SPI_MODE0);
  SPI.setClockDivider(SPI_CLOCK_DIV4);
#endif
  digitalWrite(csn_pin,mode);
}


double dewPoint(double celsius, double humidity)
{
        double A0= 373.15/(273.15 + celsius);
        double SUM = -7.90298 * (A0-1);
        SUM += 5.02808 * log10(A0);
        SUM += -1.3816e-7 * (pow(10, (11.344*(1-1/A0)))-1) ;
        SUM += 8.1328e-3 * (pow(10,(-3.49149*(A0-1)))-1) ;
        SUM += log10(1013.246);
        double VP = pow(10, SUM-3) * humidity;
        double T = log(VP/0.61078);   // temp var
        return (241.88 * T) / (17.558-T);
}

void temp()
{
  double a = analogRead(A3) * 5.0 / 1024.0;
  double x = (50000.0 / a) - 10000.0;
  //Serial.println(x);
  double w = (1.0/((log(x/10000.0))/3950.0 + (1/288.15)))-273.15+10.0;//offset=10
  Serial.println(w);
} 


void writeRegister(int deviceAddress, byte address, byte val) {
  Wire.beginTransmission(deviceAddress); // start transmission to device 
  Wire.write(address);       // send register address
  Wire.write(val);         // send value to write
  Wire.endTransmission();     // end transmission
}

int readRegister(int deviceAddress, byte address){

  int v;
  Wire.beginTransmission(deviceAddress);
  Wire.write(address); // register to read
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

int getdsProtection(void) {
	int dsProtect;
	int dsStatus;
	// Read Protection Register
	Wire.beginTransmission(dsAddress);
	Wire.write(0x00);
	Wire.endTransmission();
	Wire.requestFrom(dsAddress, 2);
	if(2 <= Wire.available())     // if two bytes were received 
	{ 
		dsProtect = Wire.read();
		dsStatus = Wire.read();
		//Serial.println(dsProtect,BIN);
		//Serial.println(dsStatus,BIN);

		if (dsProtect & DS00OV) { Serial.print("Over Voltage, "); }
		if (dsProtect & DS00UV) { Serial.print("Under Voltage, "); }
		if (dsProtect & DS00COC) { Serial.print("Charge Over Current, "); }
		if (dsProtect & DS00DOC) { Serial.print("Discharge Over Current, "); }
		if (dsProtect & DS00CC) { Serial.print("CC Pin state, "); }
		if (dsProtect & DS00DC) { Serial.print("DC Pin State, "); }
		if (dsProtect & DS00CE) { Serial.print("Charging Enabled, "); }
		if (dsProtect & DS00DE) { Serial.print("Discharging Enabled, "); }
		if (dsStatus & 32) { Serial.print("Sleep mode enabled, ");}
	}
	return dsProtect;
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
	// Read Protection Register
	Wire.beginTransmission(dsAddress);
	Wire.write(0x00);
	Wire.endTransmission();

	Wire.requestFrom(dsAddress, 2);
	if(2 <= Wire.available())     // if two bytes were received 
	{ 
		dsProtect = Wire.read();
		dsStatus = Wire.read();
		//Serial.println(dsProtect,BIN);    // print the reading 
		//Serial.println(dsStatus,BIN);

	}
	return dsProtect;
}
void resetds() {
	Wire.beginTransmission(dsAddress);
	Wire.write(0x10);
	Wire.write(0x00);  
        Wire.write(0x00);  
	Wire.endTransmission(); 	
}

void showall(void) { 
  Serial.println("Current BMP085 settings");
  Serial.println("==========================================================");
  Serial.print("device address                  = 0x");
  Serial.println(dps.getDevAddr(), HEX);
  Serial.print("Mode                            = ");
  switch (dps.getMode()) {
    case MODE_ULTRA_LOW_POWER: 
      Serial.println("MODE_ULTRA_LOW_POWER");
      break;
    case MODE_STANDARD: 
      Serial.println("MODE_STANDARD");
      break;    
    case MODE_HIGHRES: 
      Serial.println("MODE_HIGHRES");
      break;    
    case MODE_ULTRA_HIGHRES:     
      Serial.println("MODE_ULTRA_HIGHRES");
      break; 
  }  
}

void dumpRegisters() {
  byte ValidRegisterAddr[]={0xAA,0xAB,0xAC,0xAD,0xAE,0xAF,0xB0,0xB1,0xB2,0xB3,0xB4,0xB5,0xB6,0xB7,0xB8,0xB9,0xBA,0xBB,0xBC,0xBD,0xBE,0xBF,0xF6,0xF7,0xF8,0xF9}; 
  byte _b, i, totregisters = sizeof(ValidRegisterAddr);
  Serial.println("---dump start---");
  Serial.println("Register address|Register data");
  Serial.println("Reg.address(hex,dec) Reg.data(bin,hex,dec)");
  for (i=0;i<totregisters;i++){    
    Serial.print("0x");
    Serial.print(ValidRegisterAddr[i], HEX);
    Serial.print(",");
    Serial.print(ValidRegisterAddr[i], DEC);
    Serial.print(",");
    dps.readmem(ValidRegisterAddr[i], 1, &_b);
    Serial.print("b");
    print_bits(_b);
    Serial.print(",0x");
    Serial.print(_b,HEX);
    Serial.print(",");
    Serial.println(_b,DEC);  
  }
  Serial.println("---dump end---");
}

void print_bits(byte val){
  int i;
  for(i=7; i>=0; i--) 
    Serial.print(val >> i & 1, BIN);
}


