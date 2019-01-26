/* 

2019-01-25 WW Initial Create

*/


#include "CurieBLE.h"
#include "DHT.h"
#include "Adafruit_Sensor.h"
#include "U8x8lib.h"

// OLED Assignment
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);

// Assign environmental sensor type (DHT11) and pin (17)
DHT sensor(17,DHT11);

BLEPeripheral blePeripheral;       // BLE Peripheral Device (the board you're programming)

long previousMillis = 0;  // last time the heart rate was checked, in ms

// ******* Environmental Sensing ******* 
BLEService environmentService("181A"); // BLE Environmental Sensing

// BLE Humidity Measurement Characteristic
BLECharacteristic envHumidityChar("2A6F",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                              // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications
                          
BLECharacteristic envTemperatureChar("2A20",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                              // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications


// ******* Pulse Oximeter ******* 
BLEService spo2Service("1822"); // BLE Pulse Oximeter Service
BLECharacteristic PLXContChar("2A5F",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                              // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications

                              
// ******* Health Thermometer ******* 
BLEService healthThermometerService("1809");// BLE Health Thermometer
BLECharacteristic healthTempChar("2A1C",  // standard 16-bit characteristic UUID
    BLERead | BLENotify, 2);  // remote clients will be able to get notifications if this characteristic changes
                              // the characteristic is 2 bytes long as the first field needs to be "Flags" as per BLE specifications


void setup() {
  
  Serial.begin(9600);    // initialize serial communication
  
  // Set up the OLED screen
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  

  /* Set a local name for the BLE device
     This name will appear in advertising packets
     and can be used by remote devices to identify this BLE devicecur
     The name can be changed but maybe be truncated based on space left in advertisement packet */
  blePeripheral.setLocalName("SASIoTHealth");

  // Environmental
  blePeripheral.setAdvertisedServiceUuid( environmentService.uuid());  // add the service UUID
  blePeripheral.addAttribute(environmentService);   // Add the BLE Environmental Sensing service
  blePeripheral.addAttribute(envHumidityChar); // add the Humidity characteristic
  blePeripheral.addAttribute(envTemperatureChar); // add the Temperature characteristic

  // Pulse Oximeter
  blePeripheral.setAdvertisedServiceUuid(spo2Service.uuid());  // add the service UUID
  blePeripheral.addAttribute(spo2Service);   // Add the Pulse Oximeter service
  blePeripheral.addAttribute(PLXContChar); // add the PLX Continuous Measurement characteristic  

  // Health Thermometer
  blePeripheral.setAdvertisedServiceUuid(healthThermometerService.uuid());  // add the service UUID
  blePeripheral.addAttribute(healthThermometerService);   // Add the Health Thermometer service
  blePeripheral.addAttribute(healthTempChar); // add the Temperature Measurement characteristic  

  /* Now activate the BLE device.  It will start continuously transmitting BLE
     advertising packets and will be visible to remote BLE central devices
     until it receives a new connection */
  blePeripheral.begin();
  Serial.println("Bluetooth device active, waiting for connections...");
  u8x8.setCursor(0, 0);
  u8x8.print("Waiting for BLE Device");  
}

void loop() {

  int t = sensor.readTemperature(true);
  int h = sensor.readHumidity(true);
  if (t > 150) t = 0;
  if (h > 99) h = 0;
  
  Serial.println(t);
  Serial.println(h);

  u8x8.setCursor(0, 1);
  u8x8.printf("Temp: %dF", t);    
  u8x8.setCursor(0, 2);
  u8x8.printf("Hum:  %d%%", h); 
  
  // listen for BLE peripherals to connect:
  BLECentral central = blePeripheral.central();

  // if a central is connected to peripheral:
  if (central) {
    Serial.print("Connected to central: ");
    // print the central's MAC address:
    Serial.println(central.address());
    // turn on the LED to indicate the connection:
    u8x8.clearLine(0);
    u8x8.setCursor(0, 0);
    u8x8.print("Connected to " + central.address()); 

    // check the heart rate measurement every 200ms
    // as long as the central is still connected:
    while (central.connected()) {
      long currentMillis = millis();
      // if 200ms have passed, check the heart rate measurement:
      if (currentMillis - previousMillis >= 200) {
        previousMillis = currentMillis;
        updateSensors();
      }
    }

    
    // when the central disconnects, turn off the LED:
    
    Serial.print("Disconnected from central: ");
    Serial.println(central.address());
    u8x8.clearLine(0);
    u8x8.setCursor(0, 0);
    u8x8.print("Disconnected"); 
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
  const unsigned char envHumidityCharArray[2] = { 0, (char)h };
  envHumidityChar.setValue(envHumidityCharArray, 2);  // and update the environment humidity characteristic
  const unsigned char envTemperatureCharArray[2] = { 0, (char)t };
  envTemperatureChar.setValue(envTemperatureCharArray, 2);  // and update the environment temperature characteristic
  const unsigned char PLXContCharArray[2] = { 0, (char) };
  envTemperatureChar.setValue(PLXContCharArray, 2);  // and update the PLX Continuous characteristic
  const unsigned char envhealthTempCharArray[5] = { 1, (char)ht};
  envTemperatureChar.setValue(envhealthTempCharArray, 2);  // and update the health temperature characteristic    

}
