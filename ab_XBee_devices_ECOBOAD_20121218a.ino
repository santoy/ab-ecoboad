 /* 
  ==================================================
  Active Buildings
  Device-side (XBee router) Program for Arduino
  with Network communication with XBee
  *** This code is for Printed Circuit Board and
  Breadboard versions. Select digital pins for the
  correct board.
  - Light, Sound, Temperature/Humidity sensors
  - Potentiometer
  - Voltage (battery) Meter
  - RFID reader
  Author: Yasu Santo 2012 (yasu@santo.com)
  ==================================================
  DHT.cpp - DHT sensor library
  https://github.com/ringerc/Arduino-DHT22
  ==================================================
*/


#include <DHT22.h>
#include <XBee.h>
#include <SoftwareSerial.h>

const int xbeeInterval = 20000; // 20 seconds
const byte id = 0;            // Device ID  
#define devicePayLoad 27      // (number of sensors x 2) + RFID (5 bytes) + 2. "+2" for device ID and sensor number.

// analog pins
#define soundPin 0            // sound sensor
#define lightPin 1            // light sensor
#define potPin 2              // potentiometer 
#define vmPin 3               // voltage (battery) monitor pin

//* // digital pins for Printed Circuit Board version 1
boolean thisIsPcb = true;
byte soundVccPin, soundGndPin, dhtVccPin;
#define txPin 2               // unused tx connection pin to RFID (not for this PCB)
#define signalRedPin 3        // signal Red LED pin
////#define unused2 4         // unused pin
#define signalYellowPin 5     // signal Yellow LED pin
#define signalGreenPin 6      // signal Green LED pin
////#define unused3 7         // unused pin
#define resetPin 8            // reset pin for RFID
#define rxPin 9               // serial data from RFID (not for this PCB)
#define dhtPin 10             // dht - temperature and humidity sensor
////#define unused4 11        // XBEE reset pin
////#define unused5 12        // unused pin
////#define unused6 13        // unused pin
byte unusedPins = 5;
byte pinUnused[] = {4,7,11,12,13};
// */

/* // digital pins for QUT made Printed Circuit Board
boolean thisIsPcb = true;
byte soundVccPin, soundGndPin, dhtVccPin;
#define signalRedPin 2        // signal Red LED pin
#define signalYellowPin 3     // signal Yellow LED pin
#define signalGreenPin 4      // signal Green LED pin
#define txPin 5               // unused tx connection pin to RFID
//#define unused1 6           // unused pin
//#define unused2 7           // unused pin
#define resetPin 8            // reset pin for RFID
#define rxPin 9               // serial data from RFID
#define dhtPin 10             // dht - temperature and humidity sensor
//#define unused3 11          // XBEE reset pin
//#define unused4 12          // unused pin
//#define unused5 13          // unused pin
byte unusedPins = 5;
byte pinUnused[] = {6,7,11,12,13};
// */

/* // digital pins for Breadboard version
boolean thisIsPcb = false;
#define rxPin 2               // serial data from RFID
#define txPin 3               // serial data to RFID (not used)
#define resetPin 4            // reset pin for RFID
#define signalRedPin 5        // signal Red LED pin
#define signalYellowPin 6     // signal Yellow LED pin
#define signalGreenPin 7      // signal Green LED pin
#define soundVccPin 8         // Vcc pin of Sound Sensor
#define soundGndPin 9         // GND pin of Sound Sensor
#define dhtPin 10             // dht - temperature and humidity sensor
#define dhtVccPin 11          // dht - Vcc Pin
//#define unused1 12          // unused pin
//#define unused2 13          // unused pin
byte unusedPin = 2;
byte pinUnused[] = {12,13};
// */

// Setup a DHT instance and initialise
// Connect a 4.7K resistor between VCC and the data pin (strong pullup)
DHT22 myDHT22(dhtPin);
DHT22_ERROR_t errorCode;
unsigned long dhtTimer=0;
const int dhtInterval=2500;

// sensor variables
int ax=0;                     // accelerometer x-axis analog value
int ay=0;                     // accelerometer y-axis analog value
int az=0;                     // accelerometer y-axis analog value
int light=0;                  // LDR analog value
int sound=0;                  // sound sensor analog value
int temp=0;                   // temperature sensor temperature value
int humid=0;                  // humidity sensor humidity value
int pot=0;                    // potentiometer value
int vm=0;                     // voltage (battery) value
// all variables below are for keeping the previous sensor values
int pAx=0;
int pAy=0;
int pAz=0;
int pLight=0;
int pSound=0;
int pTemp=0;
int pHumid=0;
int pPot=0;
int pVm=0;

// XBee wireless communication
XBee xbee = XBee();
uint8_t payload[devicePayLoad]; // (number of sensors x 2) + 2. "+2" for location ID and sensor number.
//XBeeAddress64 addr64 = XBeeAddress64(0x0013A200, 0x402D82CA);
XBeeAddress64 addr64 = XBeeAddress64(0x00, 0x00);
ZBTxRequest zbTx = ZBTxRequest(addr64, payload, sizeof(payload));
ZBTxStatusResponse txStatus = ZBTxStatusResponse();
unsigned long xbeeTimer=0;

//* RFID
SoftwareSerial rfid( rxPin, txPin ); // 2=rxPin, 3=txPin
byte checksum = 0;
byte tagBytes[6]; // "Unique" tags are only 5 bytes but we need an extra byte for the checksum
char tagValue[13];
//*/

void setup(){
  //Serial.begin(9600);	  // Debugging only
  //XBeeNetwork.begin(9600);    // NewSoftSerial Serial Port Speed
  xbee.begin(9600);
  rfid.begin(9600); // Serial for connection to RFID module
  delay(2000);
  for(int x=0; x<devicePayLoad; x++) payload[x] = 0;
  
  pinMode(signalRedPin,1); digitalWrite(signalRedPin,0);
  pinMode(signalYellowPin,1); digitalWrite(signalYellowPin,0);
  pinMode(signalGreenPin,1); digitalWrite(signalGreenPin,0);
  pinMode(resetPin, 1); digitalWrite(resetPin, 0);
  
  for(int i=0; i<unusedPins; i++){
    pinMode(pinUnused[i],INPUT_PULLUP);
  }
  
  if(!thisIsPcb){
    pinMode(dhtVccPin,1); digitalWrite(dhtVccPin,1);
    pinMode(soundGndPin,1); digitalWrite(soundGndPin,0);
    pinMode(soundVccPin,1); digitalWrite(soundVccPin,1);
  }
}

void loop(){
  // store previous sensor values
  pAx = ax;
  pAy = ay;
  pAz = az;
  pLight = light;
  pSound = sound;
  pTemp = temp;
  pHumid = humid;
  pPot = pot;
  pVm = vm;
  // check all sensors
  ax = 1024; //max(analogRead(axPin),pAx); 
  ay = 1024; //max(analogRead(ayPin),pAy); 
  az = 1024; //max(analogRead(azPin),pAz); 
  light = max(getLight(lightPin),pLight);
  sound = max(getSound(soundPin),pSound);
  temp = max((myDHT22.getTemperatureC()+40)*10,pTemp);
  humid = max((myDHT22.getHumidity())*10,pHumid);
  pot = analogRead(potPin);
  vm = analogRead(vmPin);
  
  //checkRFID();
  
  if(millis() > dhtTimer + dhtInterval){
    dhtTimer += dhtInterval;
    errorCode = myDHT22.readData();
  }
  
  if(millis() > xbeeTimer + xbeeInterval){
    xbeeTimer += xbeeInterval;
    // checkRFID();
    sendXbeeData();
  }

  //printMessages(1);
  analogWrite(signalGreenPin,100);
  delay(5);
  digitalWrite(signalGreenPin,0);
  delay(495);
}

void sendXbeeData(){
  // Bytes to be sent to XBee Master.
  // Processing Application needs to mirror these sequence of bytes.
  payload[0] = id;  // id of this device's location
  payload[1] = 10;  // number of sensors 
  // sensor bytes below (typically 2 bytes per sensor).
  payload[2] = light & 127;      // low 7 bits of light sensor value
  payload[3] = (light>>7) & 15;  // next 4 higher bits of light sensor value
  payload[4] = sound & 127;      // low 7 bits of sound sensor value
  payload[5] = (sound>>7) & 15;  // next 4 higher bits of sound sensor value
  payload[6] = temp & 127;       // low 7 bits of temperature sensor value
  payload[7] = (temp>>7) & 15;   // next 4 higher bits of temperature sensor value
  payload[8] = humid & 127;      // low 7 bits of humidity sensor value
  payload[9] = (humid>>7) & 15;  // next 4 higher bits of humidity sensor value
  payload[10] = ax & 127;        // low 7 bits of X axis: accelerometer value
  payload[11] = (ax>>7) & 15  ;  // next 4 higher bits of X axis: accelerometer value
  payload[12] = ay & 127;        // low 7 bits of Y axis: accelerometer value
  payload[13] = (ay>>7) & 15;    // next 4 higher bits of Y axis: accelerometer value
  payload[14] = az & 127;        // low 7 bits of Z axis: accelerometer value
  payload[15] = (az>>7) &   15;  // next 4 higher bits of Z axis: accelerometer value
  payload[16] = 0; // there is no sensor
  payload[17] = 8; // there is no sensor
  payload[18] = pot & 127;       // low 7 bits of potentiometer value
  payload[19] = (pot>>7) & 15;   // next 4 higher bits of potentiometer value
  payload[20] = vm & 127;        // low 7 bits of voltage monitor value
  payload[21] = (vm>>7) & 15;    // high 4 bits of voltage monitor value
  payload[22] = tagBytes[0];     // 1st byte of RFID
  payload[23] = tagBytes[1];     // 2nd byte of RFID
  payload[24] = tagBytes[2];     // 3rd byte of RFID
  payload[25] = tagBytes[3];     // 4th byte of RFID
  payload[26] = tagBytes[4];     // 5th byte of RFID
  
  ax=0;
  ay=0;
  az=0;
  light=0;
  sound=0;
  temp=0;
  humid=0; 
  pot=0;
  vm=0;
        
  xbee.send(zbTx);
  
  if (xbee.readPacket(500)) {
    // got a response within 500ms!

    // should be a znet tx status                   
    if (xbee.getResponse().getApiId() == ZB_TX_STATUS_RESPONSE) {
      xbee.getResponse().getZBTxStatusResponse(txStatus);
                
      // get the delivery status, the fifth byte
      if (txStatus.getDeliveryStatus() == SUCCESS) {
        // success.  time to celebrate
        for(int b=0; b<1; b++){
          analogWrite(signalYellowPin,100);
          delay(5);
          digitalWrite(signalYellowPin,0);
          delay(95);
        }
      } else {
        // the remote XBee did not receive our packet. is it powered on?
      }
    }      
  } else {
    // local XBee did not provide a timely TX Status Response -- should not happen
//    for(int b=0; b<1; b++){
//      digitalWrite(signalGreenPin,1);
//      delay(250);
//      digitalWrite(signalGreenPin,0);
//      delay(250);
//    }
  }
}

int getSound(int spin)
{
  int samples=200;
  int swave[samples];
  for(int s=0; s<samples; s++)
  {
    swave[s] = analogRead(spin);
    //delayMicroseconds(10);
  }
  int myMax=0;
  int myMin=1023;
  for(int x=0; x<samples; x++)
  {
    myMax=max(myMax,swave[x]);
    myMin=min(myMin,swave[x]);
  }
  int volume = myMax-myMin;
  return volume;
}

int getLight(int lpin)
{
  int samples=200;
  int lwave[samples];
  for(int s=0; s<samples; s++)
  {
    lwave[s] = analogRead(lpin);
    //delay(1);
  }
  int myMax=0;
  int myMin=1023;
  for(int x=0; x<samples; x++)
  {
    myMax=max(myMax,lwave[x]);
    myMin=min(myMin,lwave[x]);
  }
  int brightness = ((myMax-myMin)/2)+myMin;
  return brightness;
}

void checkRFID()
{ 
  digitalWrite(resetPin,1);
  delay(1000);
  digitalWrite(resetPin,0);
  
  if(rfid.available() >= 13)
  {
    if(rfid.read() == 2) // Check for header
    {
      for(int i=0;i<12;i++)
      {
        tagValue[i] = char(rfid.read());

        // Check if this is a header or stop byte before the 10 digit reading is complete
        if((tagValue[i] == 0x0D)||(tagValue[i] == 0x0A)||(tagValue[i] == 0x03)||(tagValue[i] == 0x02)) {
          break; // Stop reading
        }
      }
      
      while(rfid.available()>0) rfid.read();
      
      for(int i=0;i<6;i++)
      {
        // Ascii/Hex conversion:
        if ((tagValue[i*2] >= '0') && (tagValue[i*2] <= '9')) tagBytes[i] = (tagValue[i*2] - '0')<<4;
        if ((tagValue[i*2] >= 'A') && (tagValue[i*2] <= 'F')) tagBytes[i] = (10 + tagValue[i*2] - 'A')<<4;
        
        if ((tagValue[(i*2)+1] >= '0') && (tagValue[(i*2)+1] <= '9')) tagBytes[i] += (tagValue[(i*2)+1] - '0');
        if ((tagValue[(i*2)+1] >= 'A') && (tagValue[(i*2)+1] <= 'F')) tagBytes[i] += (10 + tagValue[(i*2)+1] - 'A');
        
        if(i != 5) checksum ^= tagBytes[i];
      }
      digitalWrite(signalYellowPin,1);
      delay(50);
      digitalWrite(signalYellowPin,0);
    }
  } 
  else 
  {
    for(int x=0; x<6; x++) tagBytes[x]=0;
  }
  digitalWrite(signalRedPin,1);
  delay(50);
  digitalWrite(signalRedPin,0);
}

void printMessages(int m){
  if(m==1){
    Serial.print(" x:");
    Serial.print(ax);
    Serial.print(" y:");
    Serial.print(ay);
    Serial.print(" z:");
    Serial.print(az);
    Serial.print(" L:");
    Serial.print(light);
    Serial.print(" S:");
    Serial.print(sound);
    Serial.print(" T:");
    Serial.print((temp/10.0)-40);
    Serial.print(" H:");
    Serial.print((humid/10.0));
    
    Serial.println("");
  }
}

