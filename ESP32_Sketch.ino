/* 
2019-01-25 WW Initial Create
*/

#include <string>
#include <sstream>
#include <iostream>
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <DHT.h>
//#include <Adafruit_Sensor.h>
#include <U8x8lib.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include <Wire.h>
#include "MAX30105.h"
#include "spo2_algorithm.h"


MAX30105 particleSensor;

#define MAX_BRIGHTNESS 255
#if defined(__AVR_ATmega328P__) || defined(__AVR_ATmega168__)
//Arduino Uno doesn't have enough SRAM to store 100 samples of IR led data and red led data in 32-bit format
//To solve this problem, 16-bit MSB of the sampled data will be truncated. Samples become 16-bit data.
uint16_t irBuffer[100]; //infrared LED sensor data
uint16_t redBuffer[100];  //red LED sensor data
#else
uint32_t irBuffer[100]; //infrared LED sensor data
uint32_t redBuffer[100];  //red LED sensor data
#endif
int32_t bufferLength; //data length
int32_t spo2; //SPO2 value
int8_t validSPO2; //indicator to show if the SPO2 calculation is valid
int32_t heartRate; //heart rate value
int8_t validHeartRate; //indicator to show if the heart rate calculation is valid

byte pulseLED = 11; //Must be on PWM pin
byte readLED = 13; //Blinks with each data read



#define envService BLEUUID((uint16_t)0x181A) 
BLECharacteristic envHumiCharacteristic(BLEUUID((uint16_t)0x2A6F), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
BLECharacteristic envTempCharacteristic(BLEUUID((uint16_t)0x2A6E), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
//envHumiCharacteristic.addDescriptor(new BLE2902());
//envTempCharacteristic.addDescriptor(new BLE2902());

#define spo2Service BLEUUID((uint16_t)0x1822) 
BLECharacteristic spo2PlxCharacteristic(BLEUUID((uint16_t)0x2A5F), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
//spo2PlxCharacteristic.addDescriptor(new BLE2902());

#define thermoService BLEUUID((uint16_t)0x1809) 
BLECharacteristic thermoCharacteristic(BLEUUID((uint16_t)0x2A1C), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);
//thermoCharacteristic.addDescriptor(new BLE2902());

bool deviceConnected = false;
bool oldDeviceConnected = false;
bool deviceReset = false;
uint32_t value = 1;
BLEServer* pServer = NULL;

class MyServerCallbacks: public BLEServerCallbacks {
    void onConnect(BLEServer* pServer) {
      deviceConnected = true;
      BLEDevice::startAdvertising();
    };

    void onDisconnect(BLEServer* pServer) {
      deviceConnected = false;
    }
};


// OLED Assignment
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// Assign sensors
// DHT type (DHT11) and pin (17)
DHT dht(17,DHT11);
// Digital Skin Temperature
// Data wire is conntec to the Arduino digital pin 5
#define ONE_WIRE_BUS 5

// Setup a oneWire instance to communicate with any OneWire devices
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature sensor 
DallasTemperature skinTemp(&oneWire);

long previousMillis = 0;  // last time the sensors were checked, in ms

void setup() {
  
  Serial.begin(9600);    // initialize serial communication
  
  // Set up the OLED screen
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clearLine(0);
  u8x8.setCursor(0, 0);
  u8x8.print("Sampling");  
  skinTemp.begin();

  BLEDevice::init("IoTHealth");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Services
  BLEService *pEnv = pServer->createService(envService);
  pEnv->addCharacteristic(&envHumiCharacteristic);
  pEnv->addCharacteristic(&envTempCharacteristic);
  
  BLEService *pSpo2 = pServer->createService(spo2Service);
  pSpo2->addCharacteristic(&spo2PlxCharacteristic);
  
  BLEService *pThermo = pServer->createService(thermoService);
  pThermo->addCharacteristic(&thermoCharacteristic);
  

  pServer->getAdvertising()->addServiceUUID(envService);
  pServer->getAdvertising()->addServiceUUID(spo2Service);
  pServer->getAdvertising()->addServiceUUID(thermoService);
  
  pEnv->start();
  pSpo2->start();
  pThermo->start();

  // Start advertising
  pServer->getAdvertising()->start(); 
  Serial.println("Waiting a client connection to notify...");



    pinMode(pulseLED, OUTPUT);
  pinMode(readLED, OUTPUT);

  // Initialize sensor
  if (!particleSensor.begin(Wire, I2C_SPEED_FAST)) //Use default I2C port, 400kHz speed
  {
    Serial.println(F("MAX30105 was not found. Please check wiring/power."));
    while (1);
  }

  byte ledBrightness = 60; //Options: 0=Off to 255=50mA
  byte sampleAverage = 4; //Options: 1, 2, 4, 8, 16, 32
  byte ledMode = 2; //Options: 1 = Red only, 2 = Red + IR, 3 = Red + IR + Green
  byte sampleRate = 100; //Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
  int pulseWidth = 411; //Options: 69, 118, 215, 411
  int adcRange = 4096; //Options: 2048, 4096, 8192, 16384

  particleSensor.setup(ledBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange); //Configure sensor with these

    bufferLength = 25; //buffer length of 100 stores 4 seconds of samples running at 25sps

  //read the first 100 samples, and determine the signal range
  for (byte i = 0 ; i < bufferLength ; i++)
  {
    while (particleSensor.available() == false) //do we have new data?
      particleSensor.check(); //Check the sensor for new data

    redBuffer[i] = particleSensor.getRed();
    irBuffer[i] = particleSensor.getIR();
    particleSensor.nextSample(); //We're finished with this sample so move to next sample

    Serial.print(F("red="));
    Serial.print(redBuffer[i], DEC);
    Serial.print(F(", ir="));
    Serial.println(irBuffer[i], DEC);
  }

  //calculate heart rate and SpO2 after first 25 samples 
  maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);
  
}

void loop() {

  if (deviceConnected) {
      u8x8.setCursor(0, 0);
      switch (value) {
        case 1:
        u8x8.print("Sending |");
        value +=1;
        break;
        case 2:
        u8x8.print("Sending /");
        value +=1;
        break;
        case 3:
        u8x8.print("Sending -");
        value +=1;
        break;
        case 4:
        u8x8.print("Sending \\");
        value =1;
        break;
      }
    
  }

  if (!deviceConnected) {
      u8x8.setCursor(0, 0);
      switch (value) {
        case 1:
        u8x8.print("Waiting |");
        value +=1;
        break;
        case 2:
        u8x8.print("Waiting /");
        value +=1;
        break;
        case 3:
        u8x8.print("Waiting -");
        value +=1;
        break;
        case 4:
        u8x8.print("Waiting \\");
        value =1;
        break;
      }
  }

      long currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateSensors();
      }


  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
      Serial.print("Disconnected"); 
  }
  
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
  }
  
}

void updateSensors() {
  skinTemp.requestTemperatures();
  delay(100);

  // Environment
  int t = dht.readTemperature(true);
  int h = dht.readHumidity(true);
  if (t > 150) t = 0;
  if (h > 99) h = 0;
  
  Serial.print("Temperature: ");
  Serial.println(t);
  Serial.print("Humidity: ");
  Serial.println(h);
  u8x8.setCursor(0, 1);
  u8x8.printf("Env Temp: %dF  ", t);    
  u8x8.setCursor(0, 2);
  u8x8.printf("Env Hum:  %d%%  ", h);

  // Pulse Oximeter  
  Serial.print("SO2: ");
  Serial.println(spo2);
  Serial.print("Pulse: ");
  Serial.println(heartRate);
  u8x8.setCursor(0, 3);
  u8x8.printf("O2:  %d%%  ", spo2);
  u8x8.setCursor(0, 4);
  u8x8.printf("Pulse:  %d% BPM  ", heartRate);
  
  // Health Thermometer
  int ht = 0;
  skinTemp.requestTemperatures();
  ht = skinTemp.getTempFByIndex(0);

  if (ht < 50) ht = 0;
  Serial.print("Skin Temp: ");
  Serial.println(ht);  
  u8x8.setCursor(0, 5);
  u8x8.printf("Skin Temp:  %d%  ", ht);  

  
  // Update BLE Characteristics

  std::string sht;
  std::stringstream ssht;
  ssht << ht;
  sht = ssht.str();

  std::string sh;
  std::stringstream ssh;
  ssh << h;
  sh = ssh.str();
  
  std::string st;
  std::stringstream sst;
  sst << t;
  st = sst.str(); 

  std::string sspo2;
  std::stringstream ssspo2;
  ssspo2 << spo2;
  sspo2 = ssspo2.str();

    
  envHumiCharacteristic.setValue(sh);
  envHumiCharacteristic.notify();  // and update the environment humidity characteristic
  //envTempCharacteristic.setValue((uint8_t*)&t,2);
  envTempCharacteristic.setValue(st);
  envTempCharacteristic.notify();  // and update the environment humidity characteristic
  spo2PlxCharacteristic.setValue(sspo2);
  spo2PlxCharacteristic.notify();  // and update the environment humidity characteristic
  //thermoCharacteristic.setValue((char*)&ht,2);
  thermoCharacteristic.setValue(sht);
  thermoCharacteristic.notify();  // and update the environment humidity characteristic




    //dumping the first sample in the memory and shift the rest up to the top
    for (byte i = 1; i < bufferLength; i++)
    {
      redBuffer[i - 1] = redBuffer[i];
      irBuffer[i - 1] = irBuffer[i];
    }

    //take 1 sample before calculating the heart rate.
    for (byte i = bufferLength - 1; i < bufferLength; i++)
    {
      while (particleSensor.available() == false) //do we have new data?
        particleSensor.check(); //Check the sensor for new data

      digitalWrite(readLED, !digitalRead(readLED)); //Blink onboard LED with every data read

      redBuffer[i] = particleSensor.getRed();
      irBuffer[i] = particleSensor.getIR();
      particleSensor.nextSample(); //We're finished with this sample so move to next sample

      //send samples and calculation result to terminal program through UART
      Serial.print(F("red="));
      Serial.print(redBuffer[i], DEC);
      Serial.print(F(", ir="));
      Serial.print(irBuffer[i], DEC);

      Serial.print(F(", HR="));
      Serial.print(heartRate, DEC);

      Serial.print(F(", HRvalid="));
      Serial.print(validHeartRate, DEC);

      Serial.print(F(", SPO2="));
      Serial.print(spo2, DEC);

      Serial.print(F(", SPO2Valid="));
      Serial.println(validSPO2, DEC);
    }

    //After gathering new sample recalculate HR and SP02
    maxim_heart_rate_and_oxygen_saturation(irBuffer, bufferLength, redBuffer, &spo2, &validSPO2, &heartRate, &validHeartRate);

}
