/* 
2019-01-25 WW Initial Create
*/


#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>
#include <DHT.h>
#include <Adafruit_Sensor.h>
#include <U8x8lib.h>

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

// Assign environmental sensor type (DHT11) and pin (17)
DHT sensor(17,DHT11);

long previousMillis = 0;  // last time the sensors were checked, in ms

void setup() {
  
  Serial.begin(9600);    // initialize serial communication
  
  // Set up the OLED screen
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  u8x8.clearLine(0);
  u8x8.setCursor(0, 0);
  u8x8.print("Ready");  

  BLEDevice::init("IoTHealth");
  pServer = BLEDevice::createServer();
  pServer->setCallbacks(new MyServerCallbacks());

  // Create the BLE Services
  BLEService *pEnv = pServer->createService(envService);
  pEnv->addCharacteristic(&envHumiCharacteristic);
  pEnv->addCharacteristic(&envTempCharacteristic);
  
  BLEService *pSpo2 = pServer->createService(spo2Service);
  pEnv->addCharacteristic(&spo2PlxCharacteristic);
  
  BLEService *pThermo = pServer->createService(thermoService);
  pEnv->addCharacteristic(&thermoCharacteristic);
  

  pServer->getAdvertising()->addServiceUUID(envService);
  pServer->getAdvertising()->addServiceUUID(spo2Service);
  pServer->getAdvertising()->addServiceUUID(thermoService);
  
  pEnv->start();
  pSpo2->start();
  pThermo->start();

  // Start advertising
  pServer->getAdvertising()->start();

  

 
  Serial.println("Waiting a client connection to notify...");
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

  // Environment
  int t = sensor.readTemperature(true);
  int h = sensor.readHumidity(true);
  if (t > 150) t = 0;
  if (h > 99) h = 0;
  
  Serial.print("Temperature: ");
  Serial.println(t);
  Serial.print("Humidity: ");
  Serial.println(h);
  u8x8.setCursor(0, 1);
  u8x8.printf("Env Temp: %dF", t);    
  u8x8.setCursor(0, 2);
  u8x8.printf("Env Hum:  %d%%", h);

  // Pulse Oximeter
  int spo2 = 98;
  int pulse = 89;
  
  Serial.print("SO2: ");
  Serial.println(spo2);
  Serial.print("Pulse: ");
  Serial.println(pulse);
  u8x8.setCursor(0, 3);
  u8x8.printf("SPO2:  %d%%", spo2);
  u8x8.setCursor(0, 4);
  u8x8.printf("Pulse:  %d% BPM", pulse);
  
  // Health Thermometer
  int ht = 94;
  Serial.print("Skin Temp: ");
  Serial.println(ht);  
  u8x8.setCursor(0, 5);
  u8x8.printf("Skin Temp:  %d%", ht);  

  
  // Update BLE Characteristics

  
  envHumiCharacteristic.setValue((uint8_t*)&h,2);
  envHumiCharacteristic.notify();  // and update the environment humidity characteristic
  envTempCharacteristic.setValue((uint8_t*)&t,2);
  envTempCharacteristic.notify();  // and update the environment humidity characteristic
  spo2PlxCharacteristic.setValue((uint8_t*)&spo2,2);
  spo2PlxCharacteristic.notify();  // and update the environment humidity characteristic
  thermoCharacteristic.setValue((uint8_t*)&ht,2);
  thermoCharacteristic.notify();  // and update the environment humidity characteristic
    

}
