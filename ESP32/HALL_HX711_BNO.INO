/*
 * 
 * LET OP DE FILE: libesp32.a IN: /Users/berends/Documents/Arduino/hardware/espressif/esp32/tools/sdk/lib
 * IS VERVANGEN DOOR EEN ANDERE VERSIE WAARDOOR ESP32 OP 80MHZ LOOPT
 * 
 * 
 * 
 */

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

String incomingByte = ""; 
float preCalibration = 0.00; 
float calibrationFactor = 0.00;


HX711 scale;

const int touchTare = 12;
const int touchCal = 14; //calibration call
const int threshold = 20;
int touchValue;
const int ledCal = 4;
unsigned long previousMillis;   // variable for comparing millis counter
bool ledState = false; 


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

// HALL EFFECT
int nulling = 0;


// HALL EFFECT HANDLER
void int_handler()
{
	value = digitalRead(Q_OUTPUT_PIN);   

	if(value == LOW)
    {
      nulling = 1;
      Serial.println(nulling);
		//Serial.println("B Field ON");
      //  digitalWrite(LED_BUILTIN, HIGH);
	}
    else if(value == HIGH)
    {
        nulling = 0;
        Serial.println(nulling);
      //  Serial.println("B Field OFF");
      //  digitalWrite(LED_BUILTIN, LOW);
	}
}

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

  delay(200);
  Serial.println("HX711 Demo");
  Serial.println("Initializing the scale");
  // Initialize library with data output pin, clock input pin and gain factor.
  // Channel selection is made by passing the appropriate gain:
  // - With a gain factor of 64 or 128, channel A is selected
  // - With a gain factor of 32, channel B is selected
  // By omitting the gain factor parameter, the library
  // default "128" (Channel A) is used here.
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  scale.set_scale(2280.f);                      // this value is obtained by calibrating the scale with known weights; see the README for details
  scale.tare();				        // reset the scale to 0
  delay(200);
  Serial.println("Orientation Sensor Raw Data Test"); Serial.println("");
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

  pinMode(ledCal, OUTPUT);

  // SETTING INTERUPT HANDLER HALL EFFECT SENSOR
  pinMode(Q_OUTPUT_PIN, INPUT_PULLUP); // mode can be set as INPUT, OUTPU, INPUT_PULLUP
  attachInterrupt(digitalPinToInterrupt(Q_OUTPUT_PIN), int_handler, FALLING); //CHANGE, FALLING, RISING, LOW, HIGH
 

}

/**************************************************************************/
/*
    Arduino loop function, called once 'setup' is complete (your own code
    should go here)
*/
/**************************************************************************/
void loop()
{
  //calibration();


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
  
  
  BNO055calibration();

  delay(BNO055_SAMPLERATE_DELAY_MS);

  calbutton();
  tarebutton();
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

void tarebutton()
{
    touchValue = touchRead(touchTare);
    
    if (touchValue < threshold)  {
      Serial.println("Stay clear");
      delay(5000);
      scale.tare();
      Serial.println("Tare done");
      delay(1000);
  }
}

void calbutton() {
    touchValue = touchRead(touchCal);
    
    if (touchValue < threshold) {
      Serial.println("\tCALIBRATION");
      Serial.print(touchValue);
      delay(2000);
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
    else {

    }
}

void BNO055calibration() {
  /* Display calibration status for each sensor. */
 uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  if (system == 3 && gyro == 3 && mag == 3) {
    //Serial.println("Cali! 3");
    digitalWrite(ledCal, HIGH);
  }

  else {
    if (millis() - previousMillis >= 1000)  // check that 1000ms is pass
    {
      if (ledState == false) {              // check the leddState and toggle it. 
        ledState = true;
      }
      else {
        ledState = false;
      }
      digitalWrite(ledCal, ledState);          // Set LED state to ledState 
      previousMillis = millis();            // set previousMillis to current millis
    }
  }
}






