#include <ArduinoBLE.h>

BLEService device_service("19B10000-E8F2-537E-4F6C-D104768A1214"); // BLE Service

// BLE position Characteristic - custom 128-bit UUID, read and notify.
BLEUnsignedIntCharacteristic positionCharacteristic("19B10001-E8F2-537E-4F6C-D104768A1214", BLERead | BLENotify);

// pin to use for the built-in LED
const int ledPin = LED_BUILTIN; 

// Variables
int16_t pos_giunto =0;
int i;

//Time variables
unsigned long previous_timer;
unsigned long timer;

void setup() {
  
  // set LED pin to output mode
  pinMode(ledPin, OUTPUT);

  // begin initialization
  if (!BLE.begin()) {
    while (1){
      //If BLE fails to start, lonk blink
      digitalWrite(ledPin, HIGH); 
      delay(1000);
      digitalWrite(ledPin, LOW);
      delay(1000);
    }
  }
  else {
    digitalWrite(ledPin, HIGH);
  }
  //set connection interval to be the fastest possible (every 7.5ms)
  BLE.setConnectionInterval(6, 6);
  // set advertised local name and service UUID:
  BLE.setLocalName("Device_0");
  BLE.setAdvertisedService(device_service);
  
  // add the characteristic to the service
  device_service.addCharacteristic(positionCharacteristic);
  
  // add service
  BLE.addService(device_service);

  // set the initial value for the characeristic:
  positionCharacteristic.writeValue((uint16_t)0xAAAA);

  // start advertising
  BLE.advertise();

  //timer for sending new data every 7.5ms
  timer=7500;
}

void loop() {
  // listen for BLE peripherals to connect:
  // Fast blink when looking for central
  digitalWrite(ledPin, HIGH); 
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(100);
  digitalWrite(ledPin, HIGH); 
  delay(100);
  digitalWrite(ledPin, LOW);
  delay(100);
  BLEDevice central = BLE.central();

  // if a central is connected to peripheral:
  if (central) {
    //Fixed on led when connected
    digitalWrite(ledPin, HIGH);

    //Inizialization
    previous_timer=micros();
    pos_giunto=0;

    // while the central is still connected to peripheral:
    while (central.connected()) {    
      //Increase position variable of 1 every 7.5ms and send over bluetooth
      //At 2500 restart from 0;
      if (pos_giunto==2500) {
        pos_giunto=0;
      }
      if(micros() - previous_timer >= timer){
        pos_giunto++;
        positionCharacteristic.writeValue(pos_giunto);
        previous_timer = micros();
      }             
    }
    //Central disconnected, turn off led
    digitalWrite(ledPin, LOW);
  }
}
