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
BLECharacteristic envTempCharacteristic(BLEUUID((uint16_t)0x2A20), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

#define spo2Service BLEUUID((uint16_t)0x1822) 
BLECharacteristic spo2PlxCharacteristic(BLEUUID((uint16_t)0x2A5F), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

#define thermoService BLEUUID((uint16_t)0x1809) 
BLECharacteristic thermoCharacteristic(BLEUUID((uint16_t)0x2A1C), BLECharacteristic::PROPERTY_READ | BLECharacteristic::PROPERTY_NOTIFY);

bool deviceConnected = false;
bool oldDeviceConnected = false;
uint32_t value = 0;
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
  Serial.println("Starting BLE work!");

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
  
  // Set up the OLED screen
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE devicecur
     The name can be changed but maybe be truncated based on space left in advertisement packet */


  Serial.println("Bluetooth device active, waiting for connections...");
  u8x8.clearLine(0);
  u8x8.setCursor(0, 0);
  u8x8.print("Waiting for BLE Device");  
}

void loop() {

    while (deviceConnected) {
      long currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateSensors();
      }
    }

  // disconnecting
  if (!deviceConnected && oldDeviceConnected) {
      delay(500); // give the bluetooth stack the chance to get things ready
      pServer->startAdvertising(); // restart advertising
      Serial.println("start advertising");
      oldDeviceConnected = deviceConnected;
      Serial.print("Disconnected");
      u8x8.clearLine(0);
      u8x8.setCursor(0, 0);
      u8x8.print("Disconnected, waiting for BLE Device"); 
  }
  
  // connecting
  if (deviceConnected && !oldDeviceConnected) {
      oldDeviceConnected = deviceConnected;
      u8x8.clearLine(0);
      u8x8.setCursor(0, 0);
      u8x8.print("Connected"); 
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

  u8x8.setCursor(0, 3);
  u8x8.printf("SPO2:  %d%%", spo2);
  u8x8.setCursor(0, 4);
  u8x8.printf("Pulse:  %d% BPM", pulse);
  
  // Health Thermometer
  int ht = 94;
  
  u8x8.setCursor(0, 5);
  u8x8.printf("Skin Temp:  %d%", ht);  

  
  // Update BLE Characteristics

  
  envHumiCharacteristic.setValue(0, (char)h);
  envHumiCharacteristic.notify();  // and update the environment humidity characteristic
  envTempCharacteristic.setValue(0, (char)t);
  envTempCharacteristic.notify();  // and update the environment humidity characteristic
  spo2PlxCharacteristic.setValue(0, (char)spo2);
  spo2PlxCharacteristic.notify();  // and update the environment humidity characteristic
  thermoCharacteristic.setValue(0, (char)ht);
  thermoCharacteristic.notify();  // and update the environment humidity characteristic
    

}
