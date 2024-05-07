/*
 * 
 * LET OP DE FILE: libesp32.a IN: /Users/berends/Documents/Arduino/hardware/espressif/esp32/tools/sdk/lib
 * IS VERVANGEN DOOR EEN ANDERE VERSIE WAARDOOR ESP32 OP 80MHZ LOOPT
 * 
 * 
 * 
 */



#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include "HX711.h"
#if defined(ESP8266)|| defined(ESP32) || defined(AVR)
#include <EEPROM.h>
#endif
//#include "soc/rtc.h"


// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 33;
const int LOADCELL_SCK_PIN = 32;
const int calVal_eepromAdress = 0; //2
float oldCalibrationValue = 0.00; //2

String incomingByte = ""; float preCalibration = 0.00; float calibrationFactor = 0.00;


HX711 scale;

const int tarebuttonPin = 12;
const int calbuttonupPin = 14;
const int calbuttondownPin = 13;
int buttonState = 0;
int buttonStatecu = 0;
int buttonStatecd = 0;



/* ESP32 CONNECTIONS LOLIN32

   BNO055
   ===========
   0x29
   Connect SCL to 22
   Connect SDA to 21
   Connect VDD to 3.3V DC
   Connect GROUND to common ground

   HX711
   ===========
   Connect SCL to 32
   Connect SDA to 33
   Connect VDD to 3.3V DC
   Connect GROUND to common ground   

   TARE/CALIB BUTTONS
   ===========
   TARE TO 12
   CALIBRATION UP 14
   CALIBRATION DOWN 13
   3.3V VIA BUTTON AND LED/REISTANCE TO GROUND

   arduino ide via WEMOS LOLIN32, 80Mhz, 115200

   History
   =======
  2018/5/23
*/

/* Set the delay between fresh samples */
#define BNO055_SAMPLERATE_DELAY_MS (100)

// Check I2C device address and correct line below (by default address is 0x29 or 0x28)
//                                   id, address
Adafruit_BNO055 bno = Adafruit_BNO055(-1, 0x29, &Wire);

// HX711 POWERMETER VARIABLES

//float calibration_factor = -9; //-7050 worked for my 440lb max scale setup
float as = 0;
int MeasurementsToAverage = 16;
float AveragePower = 0;
float totalp = 0;
int w = 1;
float pas = 20;
unsigned long currentmillis, elapsedtime, totalmillis, strokemillis, elapsedtimestroke;
float pastotal = 0;

const int numReadings = 5;
int strokeRate[numReadings];      // the readings from the analog input
int readIndex = 0;              // the index of the current reading
int strokeRateTotal = 0;                  // the running total
int strokeaverage = 0;  
float SPower[numReadings];
float powerTotal = 0;
float poweraverage = 0;

/**************************************************************************/
/*
    Arduino setup function (automatically called at startup)
*/
/**************************************************************************/
void setup()
{
  //rtc_clk_cpu_freq_set(RTC_CPU_FREQ_80M);
  
  Wire.begin(21, 22, 400000); // (SDA, SCL) (21, 22) are default on ESP32, 400 kHz I2C bus speed
  delay(5000);
  EEPROM.begin(100);
  
  Serial.begin(115200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");


/*
  pinMode(tarebuttonPin, INPUT);
  pinMode(calbuttonupPin, INPUT);
  pinMode(calbuttondownPin, INPUT);
*/

  delay(2000);
  Serial.println("HX711 Demo");

  Serial.println("Initializing the scale");
  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // By omitting the gain factor parameter, the library
  // default "128" (Channel A) is used here.
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  Serial.println("Before setting up the scale:");
  Serial.print("read: \t\t");
  Serial.println(scale.read());			// print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));  	// print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));		// print the average of 5 readings from the ADC minus the tare weight (not set yet)

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);	// print the average of 5 readings from the ADC minus tare weight (not set) divided
						// by the SCALE parameter (not set yet)

  scale.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();				        // reset the scale to 0

  Serial.println("After setting up the scale:");

  Serial.print("read: \t\t");
  Serial.println(scale.read());                 // print a raw reading from the ADC

  Serial.print("read average: \t\t");
  Serial.println(scale.read_average(20));       // print the average of 20 readings from the ADC

  Serial.print("get value: \t\t");
  Serial.println(scale.get_value(5));		// print the average of 5 readings from the ADC minus the tare weight, set with tare()

  Serial.print("get units: \t\t");
  Serial.println(scale.get_units(5), 1);        // print the average of 5 readings from the ADC minus tare weight, divided
						// by the SCALE parameter set with set_scale

  Serial.println("Readings:");



  // Initialise the sensor 
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

 
}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop()
{



  //tarebutton();

  //scale.set_scale(calibration_factor); //Adjust to this calibration factor

  calibration();

  float pas = GetPower();

    Serial.print(pas);
    Serial.print("  ");
    Serial.print(calibrationFactor);
    Serial.print("    ");
    
  delay(100);
  
  // Possible vector values can be:
  // - VECTOR_ACCELEROMETER - m/s^2
  // - VECTOR_MAGNETOMETER  - uT
  // - VECTOR_GYROSCOPE     - rad/s
  // - VECTOR_EULER         - degrees
  // - VECTOR_LINEARACCEL   - m/s^2
  // - VECTOR_GRAVITY       - m/s^2
  imu::Vector<3> euler = bno.getVector(Adafruit_BNO055::VECTOR_EULER);

  /* Display the floating point data */
  //Serial.print("X: ");
  Serial.println(euler.x());

/*
  float deg = euler.x();
  deg = deg + 10;
  Serial.println(deg);
  */

  /*
  Serial.print(" Y: ");
  Serial.print(euler.y());
  Serial.print(" Z: ");
  Serial.print(euler.z());
  Serial.print("\t\t");
  */
  
  /*
  // Quaternion data
  imu::Quaternion quat = bno.getQuat();
  Serial.print("qW: ");
  Serial.print(quat.w(), 4);
  Serial.print(" qX: ");
  Serial.print(quat.y(), 4);
  Serial.print(" qY: ");
  Serial.print(quat.x(), 4);
  Serial.print(" qZ: ");
  Serial.print(quat.z(), 4);
  Serial.print("\t\t");
  */
  
  
  
  /* Display calibration status for each sensor. */
 /* uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  Serial.print("CALIBRATION: Sys=");
  Serial.print(system, DEC);
  Serial.print(" Gyro=");
  Serial.print(gyro, DEC);
  Serial.print(" Accel=");
  Serial.print(accel, DEC);
  Serial.print(" Mag=");
  Serial.println(mag, DEC);
  */

  delay(BNO055_SAMPLERATE_DELAY_MS);
}



float GetPower() 
{
    float as = (scale.get_units());
   /*
    if (as <= 1)
    {
      as = 0;
    }
    */
    return as;
}


/*
void tarebutton()
{
    buttonState = digitalRead(tarebuttonPin);

    if (buttonState == HIGH) {
      scale.tare();
  }
}

void calibration() 
{   
    buttonState = digitalRead(calbuttonupPin);

    if (buttonState == HIGH) {
      calibration_factor = calibration_factor + 10;
      scale.set_scale(calibration_factor);
      
    }

    //buttonStatecu == 0;

    buttonState = digitalRead(calbuttondownPin);

    if (buttonState == HIGH) {
      calibration_factor = calibration_factor - 10;
      scale.set_scale(calibration_factor);
      
    }
   // buttonStatecd == 0;
}
*/
void calibration() {

  if (Serial.available() > 0) {
    incomingByte = char(Serial.read());

    if (incomingByte == "c") {
      Serial.println(incomingByte);
      Serial.println("\tCALIBRATION");
      scale.set_scale();
      delay(50);
      scale.tare();
      Serial.println("Put the known weight 1kg on");
      delay(5000);
      Serial.println("Thank you");
      preCalibration = scale.get_units(20);
      Serial.println(preCalibration);
      delay(5000);
      calibrationFactor = (preCalibration/1000);
      Serial.println("The calibration factor: ");
      Serial.println(calibrationFactor);
      scale.set_scale(calibrationFactor);  
      delay(5000);
      EEPROM.put(calVal_eepromAdress, calibrationFactor);
      scale.tare();		

    }

    else if (incomingByte == "u"){
      //EEPROM.begin();
      float x;
      EEPROM.get(calVal_eepromAdress, oldCalibrationValue);
        Serial.print("Value ");
        Serial.println(oldCalibrationValue);
        Serial.print(" saved to EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        delay(5000);

        Serial.println("UPDATING...");
        EEPROM.put(calVal_eepromAdress, calibrationFactor);
        //EEPROM.commit();
        delay(100);
        EEPROM.get(calVal_eepromAdress, x);
        Serial.print("Value ");
        Serial.println(x);
        Serial.print("EEPROM address: ");
        Serial.println(calVal_eepromAdress);
        delay(5000);
    }
    else if ((incomingByte != "c") && (incomingByte != "u")){
      Serial.println("stopping");
    }
    else {
      
    }
  }
}
