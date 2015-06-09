// A simple data logger for the analog pins
#define CHIP_SELECT     4 // SD chip select pin
//#define LOG_INTERVAL  100 // mills between entries
//#define SENSOR_COUNT     3 // number of analog pins to log
#define ECHO_TO_SERIAL   1 // echo data to serial port
#define WAIT_TO_START    1 // Wait for serial input in setup()
#define SYNC_INTERVAL 10 // mills between calls to sync()



//put all the includes here, organized
//SD Card
#include <Fat16.h>
#include <Fat16util.h> // use functions to print strings from flash memory



//IMU 
#include <Wire.h>
#include "I2Cdev.h"
#include "RTIMUSettings.h"
#include "RTIMU.h"
#include "RTPressure.h"
#include "CalLib.h"
#include <EEPROM.h>


//servo
#include <Servo.h>



// other global declared things, again KEEP ORDER
//SD card
SdCard card;
Fat16 file;
uint32_t syncTime = 0;     // time of last sync()
uint32_t logTime  = 0;     // time data was logged

//IMU 
RTIMU *imu;                                           // the IMU object
RTPressure *pressure;                                 // the pressure object
RTIMUSettings settings;                               // the settings object
//  DISPLAY_INTERVAL sets the rate at which results are displayed
#define DISPLAY_INTERVAL  300                         // interval between pose displays
//  SERIAL_PORT_SPEED defines the speed to use for the debug serial port
#define  SERIAL_PORT_SPEED  115200
//unsigned long lastDisplay;
//unsigned long lastRate;
//int sampleCount;



// Servo
Servo servoParachute;

//general other stuffs

float altInitial   = 0;
float accellNet    = 0;
float launchTime   = 0;
float temperature;
float altCurrent = 0;
float altNet = 0;

int launched = 0;
int parachuteDeploy = 0;

// store error strings in flash to save RAM
#define error(s) error_P(PSTR(s))
//------------------------------------------------------------------------------
void error_P(const char* str)
{
  PgmPrint("error: ");
  SerialPrintln_P(str);
  if (card.errorCode) 
  {
      PgmPrint("SD error: ");
      Serial.println(card.errorCode, HEX);
  } // end of if
  while(1);
} // end of error_P
//------------------------------------------------------------------------------


void setup(void) 
{
    int errcode;

  Serial.begin(SERIAL_PORT_SPEED);
  Serial.println("REBOOT");
  Wire.begin();

  
  // SD Card stuffs
  initalize_SD();
  
  // IMU Card stuffs
  imu = RTIMU::createIMU(&settings);                        // create the imu object
  pressure = RTPressure::createPressure(&settings);         // create the pressure sensor
  if (pressure == 0) {
        Serial.println("No pressure sensor has been configured - terminating"); 
        while (1) ;
    }
    
    Serial.print("ArduinoIMU10 starting using IMU "); Serial.print(imu->IMUName());
    Serial.print(", pressure sensor "); Serial.println(pressure->pressureName());
    if ((errcode = imu->IMUInit()) < 0) {
        Serial.print("Failed to init IMU: "); Serial.println(errcode);
    }
  
    if ((errcode = pressure->pressureInit()) < 0) {
        Serial.print("Failed to init pressure sensor: "); Serial.println(errcode);
    }
  
    if (imu->getCalibrationValid())
        Serial.println("Using compass calibration");
    else
        Serial.println("No valid compass calibration data");

//    lastDisplay = lastRate = millis();
//    sampleCount = 0;

  //initalize_IMU();
  Serial.print("checking the initalization: ");
  Serial.println(pressure->initP());
  
  //servo
  servoParachute.attach(7);


} // end of setup


//------------------------------------------------------------------------------
//    String data = "";


void loop() 
{  
//    unsigned long now = millis();
//    unsigned long delta;
    float latestPressure = 0.0;
    float latestTemperature = 0.0;
    int loopCount = 1;
    float alt = 0;
    float data[12];
  
    while (imu->IMURead()) {                                // get the latest data if ready yet
        // this flushes remaining data in case we are falling behind
        if (++loopCount >= 10)
            continue;

//        sampleCount++;
//        if ((delta = now - lastRate) >= 1000)
{
//            Serial.print("Sample rate: "); Serial.print(sampleCount);
//            if (imu->IMUGyroBiasValid())
//                Serial.println(", gyro bias valid");
//            else
//                Serial.println(", calculating gyro bias");
        
//            sampleCount = 0;
//            lastRate = now;
        }
//       if ((now - lastDisplay) >= DISPLAY_INTERVAL) 
        {
//           lastDisplay = now;
          if (pressure->pressureRead(latestPressure, latestTemperature)) 
           {
             alt = pressure->altitude();
           }
           
           //create the data string with all the measurements
           imu_to_float_array(data, latestPressure, latestTemperature, alt);
        }
      }  
  log_Data(data);
}
  
  void imu_to_float_array(float data[], float &p, float &t, float &a)
  {
     data[0] = (imu->getGyro()).x(); 
     data[1] = (imu->getGyro()).y();
     data[2] = (imu->getGyro()).z();
     data[3] = (imu->getAccel()).x(); 
     data[4] = (imu->getAccel()).y();
     data[5] = (imu->getAccel()).z();
     data[6] = (imu->getCompass()).x(); 
     data[7] = (imu->getCompass()).y();
     data[8] = (imu->getCompass()).z();
     data[9] = p; 
     data[10]= t;
     data[11]= a;
  }// end of imu_to_float_array
  
  
  
 

void log_Data(float data[])
{
  logTime = millis();
  // log time to file
  file.print(logTime);  
#if ECHO_TO_SERIAL
  Serial.print(logTime);
#endif //ECHO_TO_SERIAL
      
  // add sensor data
  for (int i = 0; i < 12; i++)
  {
     file.write(',');
     file.print(data[i]);
#if ECHO_TO_SERIAL
       Serial.print(',');
       Serial.print(data[i]);
#endif // echo to serial       
  }// end of save/print data

    file.println();
#if ECHO_TO_SERIAL
    Serial.println();
#endif //ECHO_TO_SERIAL
  
  if (file.writeError) error("write data");
  
  //don't sync too often - requires 2048 bytes of I/O to SD card
  if ((millis() - syncTime) <  SYNC_INTERVAL) return;
  syncTime = millis();
  if (!file.sync()) error("sync");
  
} // end of log_Data()






//I just wanted to remove it from the setup to make the code look cleaner
void initalize_SD()
{
  // initialize the SD card
    if (!card.begin(CHIP_SELECT)) error("card.begin");  
    // initialize a FAT16 volume
    if (!Fat16::init(&card)) error("Fat16::init");  
    // create a new file
    char name[] = "LAUNCH00.CSV"; 
    for (uint8_t i = 0; i < 100; i++) 
    {
      name[6] = i/10 + '0';
      name[7] = i%10 + '0';
      // O_CREAT - create the file if it does not exist
      // O_EXCL - fail if the file exists
      // O_WRITE - open for write only
      if (file.open(name, O_CREAT | O_EXCL | O_WRITE))break;
    } // end of create file
  
    if (!file.isOpen()) error ("create");
    PgmPrint("Logging to: ");
    
    Serial.println(name);
    
    // write data header  
    
    // clear write error
    file.writeError = false;
    file.print("millis");
  #if ECHO_TO_SERIAL 
    Serial.print("millis");
  #endif //ECHO_TO_SERIAL
    char header[] = ", Gyro x, Gyro y, Gyro z, Accel x, Accel y, Accel z, Compass x, Compass y, Compass z, Pressure, Temperature, Altitude";
    file.println(header);
  #if ECHO_TO_SERIAL
    Serial.println(header);
  #endif //ECHO_TO_SERIAL
//    file.println();  
//  #if ECHO_TO_SERIAL
//    Serial.println();
//  #endif  //ECHO_TO_SERIAL

    if (file.writeError || !file.sync()) 
    {
      error("write header");
    } // end of write header error
} // end of initalize sd


