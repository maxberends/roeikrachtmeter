#include <ArduinoBLE.h>

//Defining time interval and number of devices
#define BLE_MAX_PERIPHERALS 4
#define BLE_SCAN_INTERVALL 10000
#define BLE_SCAN_new_devices 10000

// BLE variables
BLEDevice peripherals[BLE_MAX_PERIPHERALS];
BLECharacteristic positionCharacteristics[BLE_MAX_PERIPHERALS];

// variables 
uint16_t position[BLE_MAX_PERIPHERALS];
bool peripheralsConnected[BLE_MAX_PERIPHERALS] = { 0 };
bool peripheralsToConnect[BLE_MAX_PERIPHERALS] = { 0 };
bool ok = true; 
int peripheralCounter = 0;

//Time variable
unsigned long control_time;


void setup() {
  Serial.begin(9600);
  while (!Serial);
  
  // initialize the BLE hardware
  if (!BLE.begin()) {
    Serial.println("starting BLE failed!");
    while (1);
  }
  Serial.println("BLE started");
  peripheralCounter = 0;
}

void loop() {
    // start scanning for peripherals
  BLE.scanForUuid("19B10000-E8F2-537E-4F6C-D104768A1214");
  Serial.println("Scan ongoing");
  unsigned long startMillis = millis();

  //Until timeout or max devices found
  while ( millis() - startMillis < BLE_SCAN_INTERVALL && peripheralCounter < BLE_MAX_PERIPHERALS ) {
    BLEDevice peripheral = BLE.available();

    if ( peripheral ) {
      //If device has name of interest, is not already in the list to be connected or in the list of connected devices
      if ( peripheral.localName() == "Device_0" && !peripheralsToConnect[0] && !peripheralsConnected[0]) {
        peripherals[0] = peripheral;
        peripheralCounter++;
        peripheralsToConnect[0]=true;
      }
      if ( peripheral.localName() == "Device_1" && !peripheralsToConnect[1] && !peripheralsConnected[1]) {
        peripherals[1] = peripheral;
        peripheralCounter++;
        peripheralsToConnect[1]=true;
      }
      if ( peripheral.localName() == "Device_2" && !peripheralsToConnect[2] && !peripheralsConnected[2]) {
        peripherals[2] = peripheral;
        peripheralCounter++;
        peripheralsToConnect[2]=true;
      }
      if ( peripheral.localName() == "Device_3" && !peripheralsToConnect[3] && !peripheralsConnected[3]) {
        peripherals[3] = peripheral;
        peripheralCounter++;
        peripheralsToConnect[3]=true;
      }
    }
  }
  Serial.print("Device found: ");
  Serial.println(peripheralCounter);
  
  BLE.stopScan();

  //Connecting to all devices found which are not already connected
  for ( int i = 0; i < BLE_MAX_PERIPHERALS; i++ ) {
    
    if(peripheralsToConnect[i]){
      
      peripherals[i].connect();
      peripherals[i].discoverAttributes();
      BLECharacteristic positionCharacteristic = peripherals[i].characteristic("19B10001-E8F2-537E-4F6C-D104768A1214");
     
      if ( positionCharacteristic ){
        positionCharacteristics[i] = positionCharacteristic;
        positionCharacteristics[i].subscribe();
      }
     
      peripheralsConnected[i]=true;
      peripheralsToConnect[i]=false;
    }
  }
  
  control_time=millis();
  ok=true;
  while (ok) {
    if(peripheralCounter < BLE_MAX_PERIPHERALS) {
      //if not all devices connected, redo search after BLE_SCAN_new_devices time
      if(millis()-control_time>BLE_SCAN_new_devices)  {
        ok=false;
        Serial.println("Looking for other devices");
      }
    }
    for ( int i = 0; i < BLE_MAX_PERIPHERALS; i++ ) { 
      //If the i_th device is supposed to be connected
      if(peripheralsConnected[i]) { 
        //Check if it disconnected in the meantime
        if(!peripherals[i].connected()) {
          ok=false;
          peripheralsConnected[i]=false;
          peripheralCounter--;
          Serial.print("Device ");
          Serial.print(i);
          Serial.println(" disconnected.");
        }
        else {
          //if it did not disconnect, check for updates in position characteristic
          if (positionCharacteristics[i].valueUpdated()) {
            positionCharacteristics[i].readValue(position[i]);
            Serial.print("Posizione_");
            Serial.print(i);
            Serial.print(": ");
            Serial.println(position[i]);               
          }
        }        
      }
    }
  } 
  //Something disconnected or we reached limit time, redo scan to reconnect to devices 
}

