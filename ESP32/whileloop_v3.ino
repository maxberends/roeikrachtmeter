

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
float drivePower[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
float strokePower;
float angleXvalue;
float accelXvalue;  //
int powerTimeAccel;
int postpowerTimeAccel = 0;
float preGyroX;
float postGyroX;
float accelXcalc;
float gyroXvalue;
int x = 0;
int y = 0;
int strokeXvar = 0;


const int touchCal = 14; //calibration call
int touchValue;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);

  Serial.println("Hello World!");

  if(!bno.begin())
  {
    // There was a problem detecting the BNO055 ... check your connections 
    Serial.print("BNO055 not detected ");
    while(1);
  }
  delay(1000);
  /* Display the current temperature */
  int8_t temp = bno.getTemp();
  bno.setExtCrystalUse(true);

  Serial.println("BNO initialized");

}

void loop() {
    powerTimeAccel = millis();
    
    

    while (gyroX()  > 0) {  // stroke loop
  
      
      sTime = millis();
      stroke == true;
      releaseTime = sTime - rTime;
      if (strokeXvar == 0) {
        Serial.print("stroke  \t"); Serial.println(angleX());
        catchAngle = angleX();
      }
      
      //if (millis()%500>100) {
      if ((powerTimeAccel - postpowerTimeAccel) >= sampleTime) {
        postGyroX = gyroX();
        accelXcalc = ((postGyroX - preGyroX)/ sampleTime);
        Frower = ((outboardL/1000) * power() / (inboardL/1000)) * accelXcalc;
        drivePower[x] = Frower;
        strokePower += (Frower * postGyroX);
        x = x + 1;
        //Serial.println(preGyroX);
        //Serial.println(postGyroX);
        //Serial.println(accelXcalc);
        preGyroX = postGyroX;
        postpowerTimeAccel = powerTimeAccel;
        //Serial.print("&&&&&:  \t"); Serial.println(drivePower[3]);
      }
      //powercalculation();
      //gyroX();
      strokeXvar += 1;

    }
    
    while (gyroX() <= 0) { // release loop

      rTime = millis();
      stroke == false;
      driveTime = rTime - sTime;
      if (strokeXvar > 1) {
        releaseAngle = angleX();
        Serial.print("release  \t"); Serial.println(releaseAngle);
        strokeXvar = 0;
        x = 0;
        }
      
        
      }
      printvalues();

      strokeXvar = 0;
      
    }


// FUNCTIONS //

float gyroX() {
  imu::Vector<3> gyro = bno.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  gyroXvalue = (gyro.z() * (outboardL/1000));  // baansnelheid
  return gyroXvalue; //gyroXvalue;
}

float angleX() {
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);
  angleXvalue = euler.z();
  return angleXvalue; // angleX;
}

float accelX() {
  //postpowerTimeAccel = powerTimeAccel;
  preGyroX = gyroX();
  if (millis()%500>100) {
    //Serial.println("right");
    postGyroX = gyroX();
    
  }
  /*
  while (postpowerTimeAccel - powerTimeAccel < sampleTime) {
    if ((postpowerTimeAccel - powerTimeAccel) >= sampleTime) {
      postGyroX = gyroX();
      accelXcalc = ((postGyroX - preGyroX)/ sampleTime);
      
    }
    postpowerTimeAccel = powerTimeAccel;
  }
  */
  accelXcalc = ((postGyroX - preGyroX)/ sampleTime);
  //Serial.println(preGyroX);
  //Serial.println(postGyroX);
  return accelXcalc;
}

void powercalculation() {
    // P = (outboardlength * Fblade) / inboardlength  Fblade  = hx711 in kg
    // F(P) = m.a == P * accelX in Newton
    // P = dW / dt = F.v in W watt = J/s = Nm/s
    //gyroXvalue = gyroX();
    //accelXvalue = accelX();
    powerTime = millis();
    Frower = ((outboardL/1000) * power() / (inboardL/1000)) * accelXcalc; //Newton
    //P = Frower * gyroX() // dt
    if ((powerTime - postpowerTime) >= sampleTime) {
      drivePower[x] = Frower;
      strokePower += (Frower * gyroX());
      x = x + 1;
      postpowerTime = powerTime;
    }
    x = 0;

}

float power() {
  float powerRower = 10;
  return powerRower;
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
    for (x = 0; x < 10; x++) {
      float drivePrint = drivePower[x];
      Serial.print("Drive Power #: "); Serial.print(x); Serial.print("\t"); Serial.println(drivePrint, 1);
    }
    Serial.print("Frower: \t"); Serial.println(Frower);
    Serial.print("Power: \t"); Serial.println(power());
    Serial.print("Power: \t"); Serial.println(accelXcalc);
}
