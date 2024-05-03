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

void printcsv(BLECharacteristic c1, BLECharacteristic c2, BLECharacteristic c3, BLECharacteristic c4, BLECharacteristic c5, BLECharacteristic c6, BLECharacteristic c7, BLECharacteristic c8, BLECharacteristic c9){
  c1.read();
  c2.read();
  c3.read();
  c4.read();
  c5.read();
  c6.read();
  c7.read();
  c8.read();
  c9.read(); 
  float f1=getData(c1.value(), c1.valueLength());
  float f2=getData(c2.value(), c2.valueLength());
  float f3=getData(c3.value(), c3.valueLength());
  float f4=getData(c4.value(), c4.valueLength());
  float f5=getData(c5.value(), c5.valueLength());
  float f6=getData(c6.value(), c6.valueLength());
  float f7=getData(c7.value(), c7.valueLength());
  float f8=getData(c8.value(), c8.valueLength());
  float f9=getData(c9.value(), c9.valueLength());
  Serial.print(f1);
  Serial.print(',');
  Serial.print(f2);
  Serial.print(',');
  Serial.print(f3);
  Serial.print(',');
  Serial.print(f4);
  Serial.print(',');
  Serial.print(f5);
  Serial.print(',');
  Serial.print(f6);
  Serial.print(',');
  Serial.print(f7);
  Serial.print(',');
  Serial.print(f8);
  Serial.print(',');
  Serial.print(f9);
  Serial.print('\n');
}

void setup() {
  Serial.begin(115200);

  if(!BLE.begin()) {
    Serial.println("Starting BLE Failed!");
    while(1);
  }
  BLE.scan();
}

void loop() {
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
      BLECharacteristic accx=acc.characteristic("2001");
      BLECharacteristic accy=acc.characteristic("2002");
      BLECharacteristic accz=acc.characteristic("2A37");
      BLEService gyro=peripheral.service("1002");
      BLECharacteristic gyrox=gyro.characteristic("2011");
      BLECharacteristic gyroy=gyro.characteristic("2012");
      BLECharacteristic gyroz=gyro.characteristic("2013");
      BLEService mag=peripheral.service("1003");
      BLECharacteristic magx=mag.characteristic("2021");
      BLECharacteristic magy=mag.characteristic("2022");
      BLECharacteristic magz=mag.characteristic("2023");
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
          printcsv(accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz);
        }
        else{
          peripheral.disconnect();
          return;
        }
      }
    }
  }
  BLE.scan();
  Serial.println("rescan");
}

