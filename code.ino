#include <Modbus.h>
#include <ModbusIP.h>
//above libraries custom made libraries from someone in github
//https://github.com/andresarmento/modbus-arduino
# include <Ethernet.h>
# include <SPI.h>
# include <Wire.h>
# include <VL53L0X.h>
#include <FastLED.h> // LED strip library
//libraries for voice recognition module 
#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

#define LED_PIN     48
#define NUM_LEDS    48
int x=0;
int Dist(x);

/**        
 * Connection
 * Arduino    VoiceRecognitionModule
 * 12   ------->     TX
 * 13<  ------->     RX
 */
//v3 should be connected to rx and tx pins on arduino mega only 
VR myVR(12,13);    // 12:RX 13:TX  or use pins 0 1 which are rx and tx

uint8_t records[7]; // save record
uint8_t buf[64];
int Stop=0;

#define onRecord    (0)
#define offRecord   (1) 

//v3.h

void printSignature(uint8_t *buf, int len)
{
  int i;
  for(i=0; i<len; i++){
    if(buf[i]>0x19 && buf[i]<0x7F){
      Serial.write(buf[i]);
    }
    else{
      Serial.print("[");
      Serial.print(buf[i], HEX);
      Serial.print("]");
    }
  }
}

//v3 signature
void printVR(uint8_t *buf)
{
  Serial.println("VR Index\tGroup\tRecordNum\tSignature");

  Serial.print(buf[2], DEC);
  Serial.print("\t\t");

  if(buf[0] == 0xFF){
    Serial.print("NONE");
  }
  else if(buf[0]&0x80){
    Serial.print("UG ");
    Serial.print(buf[0]&(~0x80), DEC);
  }
  else{
    Serial.print("SG ");
    Serial.print(buf[0], DEC);
  }
  Serial.print("\t");

  Serial.print(buf[1], DEC);
  Serial.print("\t\t");
  if(buf[3]>0){
    printSignature(buf+4, buf[3]);
  }
  else{
    Serial.print("NONE");
  }
  Serial.println("\r\n");
}


CRGB leds[NUM_LEDS];

int SensorVal0 , SensorVal1 , SensorVal2 , SensorVal3 , Dis1 , Dis2 , Dis3 , Dis4 , Dis5 ,Dis6 ;
const int Sensor1 = 0;
const int Sensor2 = 1;
const int Sensor3 = 2;
const int Sensor4 = 3;
const int Sensor5 = 4;
const int Sensor6 = 5;
const int Sensor7 = 6;
const int Sensor8 = 7;
const int Sensor9 = 8;
const int Sensor10 = 9;
const int Sensor17 = 16;



ModbusIP mb;

void setup () {
//led pin setup led strip
FastLED.addLeds<WS2812B, LED_PIN, GRB>(leds, NUM_LEDS);
//set all led lights to green at the start
for (int i = 0; i <= 47; i++) {
    leds[i] = CRGB ( 0, 255, 0);
}
    FastLED.show();
    delay(1000);

// initialize voice recognition module v3
myVR.begin(9600);
  
Serial.begin(115200);
Serial.println("Elechouse Voice Recognition V3 Module\r\nControl LED sample");

  
if(myVR.clear() == 0){
  Serial.println("Recognizer cleared.");
}else{
  Serial.println("Not find VoiceRecognitionModule.");
  Serial.println("Please check connection and restart Arduino.");
  while(1);
}

if(myVR.load((uint8_t)onRecord) >= 0){
  Serial.println("onRecord loaded");
}

if(myVR.load((uint8_t)offRecord) >= 0){
  Serial.println("offRecord loaded");
}
  

Wire . begin () ;
 Serial.println("1");

// pinmode for MTCH105 capacitive proximity sensor
pinMode (40 , INPUT_PULLUP ) ;
pinMode (41 , INPUT_PULLUP ) ;
pinMode (42 , INPUT_PULLUP ) ;
pinMode (43 , INPUT_PULLUP ) ;
 Serial.println("2");
// pinmode for ultrasonic sensors
pinMode (23 , OUTPUT ) ;
pinMode (22 , INPUT ) ;
pinMode (25 , OUTPUT ) ;
pinMode (24 , INPUT ) ;
pinMode (27 , OUTPUT ) ;
pinMode (26 , INPUT ) ;
pinMode (29 , OUTPUT ) ;
pinMode (28 , INPUT ) ;
pinMode (31 , OUTPUT ) ;
pinMode (30 , INPUT ) ;
pinMode (33 , OUTPUT ) ;
pinMode (32 , INPUT ) ;
 Serial.println("3");


// Modbus setup of Arduino mega
byte mac [] = {0xDE , 0xAD , 0xBE , 0xEF , 0xFE , 0xED };
byte ip [] = {194 , 94 , 86 , 6};
mb . config ( mac , ip );
Serial.println("mac and ip configured here");

 Serial.println("4");
// Defining the input registers to be written to the PLC
mb . addIreg ( Sensor1 ) ;
mb . addIreg ( Sensor2 ) ;
mb . addIreg ( Sensor3 ) ;
mb . addIreg ( Sensor4 ) ;
mb . addIreg ( Sensor5 ) ;
mb . addIreg ( Sensor6 ) ;
mb . addIreg ( Sensor7 ) ;
mb . addIreg ( Sensor8 ) ;
mb . addIreg ( Sensor9 ) ;
mb . addIreg ( Sensor10 ) ;
mb . addIreg ( Sensor17 ) ;

Serial . println ("ok1") ;
}
void loop ()
{
   
// initializing the pulseIn duration values of the ultrasonic sensors ’ echo pins
int Dur1 =0;
int Dur2 =0;
int Dur3 =0;
int Dur4 =0;
int Dur5 =0;
int Dur6 =0;

// modbus initialization every scan cycle
mb . task () ;

// voice recognition task 
int ret;
ret = myVR.recognize(buf, 50);
if(ret>0){
  switch(buf[1]){
    case onRecord:
      /** assign value 0 to stop variable */
      Stop=0;
      break;
    case offRecord:
      /**  assign value 1 to stop variable*/
      Stop=1;
      break;
    default:
      Serial.println("Record function undefined");
      break;
  }
  /** voice recognized */
  printVR(buf);
}
  
// capacitive proximity sensor reading
SensorVal0 = digitalRead (40) ;
SensorVal1 = digitalRead (41) ;
SensorVal2 = digitalRead (42) ;
SensorVal3 = digitalRead (43) ;
// ultrasonic sensor reading
digitalWrite (23 , HIGH ) ;
delayMicroseconds (10) ;
digitalWrite (23 , LOW ) ;
Dur1 = pulseIn (22 , HIGH , 25000) ;
Dis1 =( Dur1 /2) /29.1;

digitalWrite (25 , HIGH ) ;
delayMicroseconds (10) ;
digitalWrite (25 , LOW ) ;
Dur2 = pulseIn (24 , HIGH , 25000) ;
Dis2 =( Dur2 /2) /29.1;

digitalWrite (27 , HIGH ) ;
delayMicroseconds (10) ;
digitalWrite (27 , LOW ) ;
Dur3 = pulseIn (26 , HIGH , 25000) ;
Dis3 =( Dur3 /2) /29.1;

digitalWrite (29 , HIGH ) ;
delayMicroseconds (10) ;
digitalWrite (29 , LOW ) ;
Dur4 = pulseIn (28 , HIGH , 25000) ;
Dis4 =( Dur4 /2) /29.1;

digitalWrite (31 , HIGH ) ;
delayMicroseconds (10) ;
digitalWrite (31 , LOW ) ;
Dur5 = pulseIn (30 , HIGH , 25000) ;
Dis5 =( Dur5 /2) /29.1;

digitalWrite (33 , HIGH ) ;
delayMicroseconds (10) ;
digitalWrite (33 , LOW ) ;
Dur6 = pulseIn (32 , HIGH , 25000);
Dis6 =( Dur6 /2) /29.1;
int Dist[]={Dis1,Dis2,Dis6,Dis3,Dis4,Dis5};
Serial . println (Dis1) ;
Serial . println (Dis2) ;
Serial . println (Dis3) ;
Serial . println (Dis4) ;
Serial . println (Dis5) ;
Serial . println (Dis6) ;
Serial . println (Stop) ;

for(int x=0; x<6; x++) {
  if(Dist[x]<20 && Dist[x]!=0) {//for red lights turn on 
    for(int j=8*x; j<=(8*(x+1))-1; j++) {
      leds[j] = CRGB ( 255, 0, 0);
    }
  } 
    
    else if(Dist[x]>=20 && Dist[x]<40) {//for yellow lights on the strip
      for(int j=8*x; j<=(8*(x+1))-1; j++){
        leds[j] = CRGB ( 255, 165, 0);
      }
   }
    else {
      for (int j=8*x; j<=(8*(x+1))-1; j++) {//set all led lights to green at the start
        leds[j] = CRGB ( 0, 255, 0);
      }
    }   
   FastLED.show();
   delay(1000); 
     
}

// print ok to acknowledge no error ! for debug purpose !
Serial . println ("ok") ;
// Modbus values write [ Reading of ToF sensors written directly to the PLC ]
mb . Ireg ( Sensor4 , SensorVal0 ) ;
mb . Ireg ( Sensor3 , SensorVal1 ) ;
mb . Ireg ( Sensor2 , SensorVal2 ) ;
mb . Ireg ( Sensor1 , SensorVal3 ) ;
mb . Ireg ( Sensor10 , Dis1 ) ;
mb . Ireg ( Sensor7 , Dis2 ) ;
mb . Ireg ( Sensor5 , Dis3 ) ;
mb . Ireg ( Sensor8 , Dis4 ) ;
mb . Ireg ( Sensor9 , Dis5 ) ;
mb . Ireg ( Sensor6 , Dis6 ) ;
mb . Ireg ( Sensor17, Stop) ;
}
