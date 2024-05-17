

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "HX711.h"
#include <esp_now.h>
#include <WiFi.h>
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
//#include "soc/rtc.h"
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);


//float gyroX = 0;
unsigned long sTime;
unsigned long rTime;
float driveTime;
float releaseTime;
float drRatio;
float strokeRate;
bool stroke;
int catchAngle;
int releaseAngle;
unsigned long powerTime;
unsigned long postpowerTime = 0;
int sampleTime = 100;
float Frower;
int outboardL = 3500;
int inboardL = 1200;
float drivePower[20];
float strokePower;
float angleXvalue;
float accelXvalue;  //
int powerTimeAccel;
int postpowerTimeAccel;
float preGyroX;
float postGyroX;
float accelXcalc;
float gyroXvalue;
int x = 0;
int strokeXvar = 0;
int catchAngle;
int releaseAngle;


const int touchCal = 14; //calibration call
int touchValue;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("Hello World!");

}

void loop() {

    
    touchValuevoid();


    while (gyroX()  > 0) {  // stroke loop
      
      sTime = millis();
      stroke == true;
      releaseTime = sTime - rTime;
      if (strokeXvar == 0) {
        Serial.println("stroke");
        catchAngle = angleX();
      }
      powercalculation();
      gyroX();
      strokeXvar += 1;
    }
    
    while (gyroX() <= 0) { // release loop

      rTime = millis();
      stroke == false;
      driveTime = rTime - sTime;
      gyroX();
      if (strokeXvar =>3) {
        releaseAngle = angleX();
      }
      
    }

    printvalues();

    strokeXvar = 0;
  

}

float angleX() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angleXvalue = euler.x();
  return angleXvalue; // angleX;
}

float accelX() {
  powerTimeAccel = millis();
  preGyroX = gyroX();
  if ((postpowerTimeAccel - powerTimeAccel) == sampleTime) {
    postGyroX = gyroX();
    accelXcalc = ((postGyroX - preGyroX)/ sampleTime);
  }
  postpowerTimeAccel = powerTimeAccel;
  return accelXcalc;
}

float gyroX() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyroXvalue = (gyro.x() * outboardL);  // baansnelheid
  return gyroXvalue; //gyroXvalue;
}

float power() {
  float powerRower = 10;
  return powerRower;
}

void powercalculation() {
    // P = (outboardlength * Fblade) / inboardlength  Fblade  = hx711 in kg
    // F(P) = m.a == P * accelX in Newton
    // P = dW / dt = F.v in W watt = J/s = Nm/s
    //gyroXvalue = gyroX();
    //accelXvalue = accelX();
    powerTime = millis();
    Frower = ((outboardL/1000) * power() / (inboardL/1000)) * accelX(); //Newton
    //P = Frower * gyroX() // dt
    if ((postpowerTime - powerTime) == sampleTime) {
      drivePower[x] = Frower;
      strokePower += (Frower * gyroX());
      x += 1;
      postpowerTime = powerTime;
    }

}

void printvalues() { // esp now message emulator
    Serial.print("Stroketime: "); Serial.print(stroke); Serial.print("\t"); Serial.println(driveTime);
    Serial.print("Releasetime: "); Serial.print(stroke); Serial.print("\t"); Serial.println(releaseTime);
    drRatio = driveTime/releaseTime;
    Serial.print("DRratio: "); Serial.println(drRatio, 2); 
    //strokeDuration = (driveTime + releaseTime);
    strokeRate = (60/((driveTime + releaseTime)/1000));
    Serial.print("Stroke rate: \t"); Serial.println(strokeRate, 1);
    Serial.print("Catch Anlge: \t"); Serial.println(catchAngle);
    Serial.print("Release Angle: \t"); Serial.println(releaseAngle);
    //Serial.print("Frower in Newton: \t"); Serial.println(Frower, 1);
    Serial.print("Stroke in Watt: \t"); Serial.println(strokePower, 1);
    for (x = 0; x += 1; x < 10) {
      Serial.print("Drive Power #: "); Serial.println(x); Serial.print("\t"); Serial.println(drivePower[x], 1);
    }
}
