/*
Werkend model central voor coxbox moet volgende modules/ hardware bedienen

-accelerometer/ gyro
-SDD display
-rotary encoder
-SD card logger
-aan uit knop
-battery power


Versie:

1.0 3/5/2024: werkend met ble peripheral voor 1 accelerometer data path x


  */



#include <ArduinoBLE.h>

union dat{
  unsigned char asdf[4];
  float zxcv;
};

float getData(const unsigned char data[], int length) {
  dat dat;
  for (int i = 0; i < length; i++) {
    dat.asdf[i] = data[i]; 
    }
  return dat.zxcv;
}

void printcsv(BLECharacteristic c1){//, BLECharacteristic c2, BLECharacteristic c3, BLECharacteristic c4, BLECharacteristic c5, BLECharacteristic c6, BLECharacteristic c7, BLECharacteristic c8, BLECharacteristic c9){
  c1.read();

  float f1=getData(c1.value(), c1.valueLength());

  Serial.print(f1);
  Serial.print('\n');
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  if(!BLE.begin()) {
    Serial.println("Starting BLE Failed!");
    while(1);
  }
  BLE.scan();
}

void loop() {
  // put your main code here, to run repeatedly:
  BLEDevice peripheral = BLE.available();

  if(peripheral){
    if(peripheral.localName()=="Accelerometer"){
      BLE.stopScan();
      if(peripheral.connect()){
        Serial.println("Connect1");
      }
      else{
        return;
      }
      if(peripheral.discoverAttributes()){
        Serial.println("Connect2");
      }
      else{
        return;
      }
      BLEService acc=peripheral.service("180D");
      BLECharacteristic accx=acc.characteristic("2A37");

      while(true){
//        accx.read();
//        float f1=getData(accx.value(),accx.valueLength());
//        Serial.print(f1);
//        Serial.print(',');
//        accy.read();
//        float f2=getData(accy.value(),accy.valueLength());
//        Serial.print(f2);
//        Serial.print(',');
//        accz.read();
//        float f3=getData(accz.value(),accz.valueLength());
//        Serial.println(f3);
        if(peripheral.connected()){
          printcsv(accx);//,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz);
        }
        else{
          peripheral.disconnect();
          return;
        }
      }
    }
  }
  BLE.scan();
}
