/*
Werkend model peripheral voor riem moet volgende modules/ hardware bedienen

-accelerometer/ gyro
-hx711 load cell
-aan uit knop
-battery power


Versie:

1.0 3/5/2024: werkend met ble central voor 1 accelerometer data path x


  */



#include <ArduinoBLE.h>
#include "Arduino_BMI270_BMM150.h"

// Definieer de BLE service en de karakteristiek
BLEService accelerometerService("180D");
BLEFloatCharacteristic accelerometerDataCharacteristic("2A37", BLERead | BLENotify);

void setup() {
  Serial.begin(9600);

  // Initialiseer de BLE omgeving
  if (!BLE.begin()) {
    Serial.println("BLE couldn't be started!");
    while (1);
  }

  // Initialiseer de LSM9DS1 accelerometer
  if (!IMU.begin()) {
    Serial.println("Failed to initialize IMU!");
    while (1);
  }

  // Adverteer de service
  BLE.setLocalName("Accelerometer");
  BLE.setAdvertisedService(accelerometerService);

  // Voeg de karakteristiek toe aan de service
  accelerometerService.addCharacteristic(accelerometerDataCharacteristic);

  // Begin met adverteren
  BLE.addService(accelerometerService);
  accelerometerDataCharacteristic.setValue(0); // Beginwaarde
  BLE.advertise();

  Serial.println("BLE peripheral is actief, wacht op verbinding...");
}

void loop() {
  // Wacht op een verbinding
  BLEDevice central = BLE.central();

  // Als er een verbinding is
  if (central) {
    Serial.print("Verbonden met: ");
    Serial.println(central.address());

    // Zolang de centrale verbonden is, blijf de accelerometergegevens sturen
    while (central.connected()) {
      // Lees de accelerometergegevens
      float x, y, z;
      if (IMU.accelerationAvailable()) {
        IMU.readAcceleration(x, y, z);
        
        // Stel de waarden in op de BLE karakteristiek
        accelerometerDataCharacteristic.setValue(x);
        accelerometerDataCharacteristic.valueUpdated();
        //accelerometerDataCharacteristic.notify();
        Serial.println(x);

        delay(100); // Wacht een korte tijd voordat je de volgende update verzendt
      }
    }

    // De centrale heeft de verbinding verbroken
    Serial.print("Verbinding verbroken met: ");
    Serial.println(central.address());
  }
}


