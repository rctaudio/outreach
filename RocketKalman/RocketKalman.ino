/*
-----------------inside_throw_1--------------------------------

float sdevProcess = 0.1;
float sdevMeasurement = 1;

 P[0][0] = 58; P[0][1] = 100; P[0][2] = 87;
  P[1][0] = 100; P[1][1] = 275; P[1][2] = 326; 
  P[2][0] = 087; P[2][1] = 326; P[2][2] = 608;




---------------inside_throw_2----------------------------------------
float sdevProcess = 0.1;
float sdevMeasurement = 1;

 P[0][0] = .058; P[0][1] = .100; P[0][2] = .087;
  P[1][0] = .100; P[1][1] = .275; P[1][2] = .326; 
  P[2][0] = .087; P[2][1] = .326; P[2][2] = .608;
  
----------------inside_throw_3------------------------------------

float sdevProcess = 0.1;
float sdevMeasurement = .5;

 P[0][0] = .058; P[0][1] = .100; P[0][2] = .087;
  P[1][0] = .100; P[1][1] = .275; P[1][2] = .326; 
  P[2][0] = .087; P[2][1] = .326; P[2][2] = .608;
  
-----------------inside_throw_4------------------------

float sdevProcess = 0.1;
float sdevMeasurement = .2;

  P[0][0] = .058; P[0][1] = .100; P[0][2] = .087;
  P[1][0] = .100; P[1][1] = .275; P[1][2] = .326; 
  P[2][0] = .087; P[2][1] = .326; P[2][2] = .608;
 
-----------------inside_throw_5------------------

float sdevProcess = 0.1;
float sdevMeasurement = .2;

  P[0][0] = 1000; P[0][1] = 0; P[0][2] = 0;
  P[1][0] = 0; P[1][1] = 1000; P[1][2] = 0; 
  P[2][0] = 0; P[2][1] = 0; P[2][2] = 1000;

----------------inside_throw_6--------------

float sdevProcess = 0.2;
float sdevMeasurement = 1;
  
    P[0][0] = 1000; P[0][1] = 0; P[0][2] = 0;
  P[1][0] = 0; P[1][1] = 1000; P[1][2] = 0; 
  P[2][0] = 0; P[2][1] = 0; P[2][2] = 1000;
  
  
-----------------inside_throw_01---------------------

  float sdevProcess = 0.5;
float sdevMeasurement = .05;

    P[0][0] = 1000; P[0][1] = 0; P[0][2] = 0;
  P[1][0] = 0; P[1][1] = 1000; P[1][2] = 0; 
  P[2][0] = 0; P[2][1] = 0; P[2][2] = 1000;
  
  
-----------------inside_throw_02---------------------

  float sdevProcess = 0.5;
float sdevMeasurement = .02;

    P[0][0] = 1000; P[0][1] = 0; P[0][2] = 0;
  P[1][0] = 0; P[1][1] = 1000; P[1][2] = 0; 
  P[2][0] = 0; P[2][1] = 0; P[2][2] = 1000;
  
  -----------------rocket launch 1---------------------

  float sdevProcess = 0.005; 
float sdevMeasurement = .05;

    P[0][0] = 1000; P[0][1] = 0; P[0][2] = 0;
  P[1][0] = 0; P[1][1] = 1000; P[1][2] = 0; 
  P[2][0] = 0; P[2][1] = 0; P[2][2] = 1000;
  
  
  
  
  

*/

// these includes allow us to use someone elses code.  This way we dont have to figure out every little detail
// , instead we can focus on what we want to do.

#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <MatrixMath.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP085_U.h>



/* Assign a unique ID to the sensors */
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

//variables for hardware : sd card reader and leds

const int chipSelect = 4;
const int ledBlue = 5;
const int ledGreen = 6;
const int ledRed = 7;

//variables used for the maths that make our graphs pretty 
// we are using a kalman filter which is a REALLY awesome filter
// based on probability.
float Z[1][1];
float X_prior[3][1];
float X_est[3][1];
float X_old[3][1];
float KalmanGain[3][1];
float t_old = 0;
float dt = 0;
float F[3][3];
float Q[3][3];
float P[3][3];
float temp33_1[3][3];
float temp33_2[3][3];
float temp31[3][1];
float temp11[1][1];
float sdevProcess = 0.005;
float sdevMeasurement = .05;



/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!bmp.begin(BMP085_MODE_ULTRALOWPOWER))
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  } // end of if
} // end of initSensors()

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  // tell the arduion that we want to use these pins as outputs, i.e. as switches
  pinMode(ledBlue, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);

  //flash all the LED's to give us visual feedback that the system is booting up
  digitalWrite(ledBlue, HIGH);  
  digitalWrite(ledGreen, HIGH);  
  digitalWrite(ledRed, HIGH);  
  delay(500);              
  digitalWrite(ledBlue, LOW);  
  digitalWrite(ledGreen, LOW);  
  digitalWrite(ledRed, LOW);
  delay(1000); 
  digitalWrite(ledBlue, HIGH);  
  digitalWrite(ledGreen, HIGH);  
  digitalWrite(ledRed, HIGH);
  
  //enable the Serial(magnifying glass button) to help us debug our program if needed
  Serial.begin(115200);
  Serial.println(F("Surfing on a rocket")); Serial.println("");
  
  /* Initialise the sensors */
  initSensors();
  sensors_event_t bmp_event;
  bmp.getEvent(&bmp_event);
  
  /* Get ambient temperature in C */
  float temperature;
  bmp.getTemperature(&temperature);
  
  /* Convert atmospheric pressure, SLP and temp to altitude    */
  X_old[0][0] = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
                                        
  //initalize some needed values for our Kalman filter
  t_old = millis()/1000.0;
  X_old[1][0] = 0;
  X_old[2][0] = 0;
  P[0][0] = 1000; P[0][1] = 0; P[0][2] = 0;
  P[1][0] = 0; P[1][1] = 1000; P[1][2] = 0; 
  P[2][0] = 0; P[2][1] = 0; P[2][2] = 1000;
  
  Serial.print("Initializing SD card...");
  // see if the card is present and can be initialized:
  if (!SD.begin(chipSelect)) 
  {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  } // end of if
  Serial.println("card initialized.");
  
} // end of setup()

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/



void loop(void)
{
  //create a CUSTOM variable to store the bmp (barometric pressure) values in
  sensors_event_t bmp_event;
  
  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    float temperature;
    bmp.getTemperature(&temperature);
    
    /* Convert atmospheric pressure, SLP and temp to altitude    */
    Z[0][0] = bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
    
    
    //more maths involved with the Kalman filter stuffs
    dt = millis()/1000.0-t_old;
    t_old = millis()/1000.0;
    
    F[0][0] = 1; F[0][1] = dt; F[0][2] = pow(dt,2)/2.0;   
    F[1][0] = 0; F[1][1] = 1; F[1][2] = dt;   
    F[2][0] = 0; F[2][1] = 0; F[2][2] = 1;   
    
    Q[0][0] = pow(dt,6)/9.0*pow(sdevProcess,2);
    Q[0][1] = pow(dt,5)/6.0*pow(sdevProcess,2);
    Q[0][2] = pow(dt,4)/3.0*pow(sdevProcess,2);
    Q[1][0] = pow(dt,5)/6.0*pow(sdevProcess,2);
    Q[1][1] = pow(dt,4)/4.0*pow(sdevProcess,2);
    Q[1][2] = pow(dt,3)/2.0*pow(sdevProcess,2);
    Q[2][0] = pow(dt,4)/3.0*pow(sdevProcess,2);
    Q[2][1] = pow(dt,3)/2.0*pow(sdevProcess,2);
    Q[2][2] = pow(dt,2)/1.0*pow(sdevProcess,2);

    //more Maths
    //Predict
    Matrix.Multiply((float*)F,(float*)X_old,3,3,1,(float*)X_prior); // Predict State

    //Matrix.Print((float*)P,3,3,"P initial:");
    Matrix.Transpose((float*)F,3,3,(float*)temp33_1); // Predict Covar
    //Matrix.Print((float*)temp33_1,3,3,"F':");
    Matrix.Multiply((float*)P,(float*)temp33_1,3,3,3,(float*)temp33_2);
    //Matrix.Print((float*)temp33_2,3,3,"PF':");
    Matrix.Multiply((float*)F,(float*)temp33_2,3,3,3,(float*)temp33_1);
    //Matrix.Print((float*)temp33_1,3,3,"FPF':");
    Matrix.Add((float*)temp33_1,(float*)Q,3,3,(float*)P);
    //Matrix.Print((float*)P,3,3,"P Priori:");
    
    //Even more maths
    //Update
    temp11[0][0] = P[0][0]+pow(sdevMeasurement,2);  // Innovation Covar
    
    temp11[0][0] = 1.0/temp11[0][0];                // Optimal Kalman Gain
    temp31[0][0] = temp11[0][0]; temp31[1][0] = 0; temp31[2][0] = 0; 
    Matrix.Multiply((float*)P,(float*)temp31,3,3,1,(float*)KalmanGain);

    temp11[0][0] = Z[0][0] - X_prior[0][0];         // State Estimate
    Matrix.Multiply((float*)KalmanGain,(float*)temp11,3,1,1,(float*)temp31);
    Matrix.Add((float*)X_prior,(float*)temp31,3,1,(float*)X_est); 
    
    temp33_1[0][0] = 1-KalmanGain[0][0]; temp33_1[0][1] = 0; temp33_1[0][2] = 0; // Covar Est
    temp33_1[1][0] = -KalmanGain[1][0];  temp33_1[1][1] = 1; temp33_1[1][2] = 0;
    temp33_1[2][0] = -KalmanGain[2][0];  temp33_1[2][1] = 0; temp33_1[2][2] = 1;
    Matrix.Copy((float*)P,3,3,(float*)temp33_2);
    Matrix.Multiply((float*)temp33_1,(float*)temp33_2,3,3,3,(float*)P);
    
    Matrix.Print((float*)X_est,3,1,"X_est:");

    Matrix.Copy((float*)X_est,3,1,(float*)X_old);
        
    // LOGGING to the sd card now, so far we have made variables
    // to store data in and now we will write those variables to 
    // a file so we can look at them later     
     
    File dataFile = SD.open("datalog.csv", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) 
    {
      digitalWrite(ledBlue, LOW);  digitalWrite(ledGreen, LOW);  digitalWrite(ledRed, LOW);
      dataFile.print(millis()/1000.0);
      dataFile.print(",");
      dataFile.print(Z[0][0]);
      dataFile.print(",");
      dataFile.print(X_est[0][0]);
      dataFile.print(",");
      dataFile.print(X_est[1][0]);
      dataFile.print(",");
      dataFile.print(X_est[2][0]);
      dataFile.println(",");
      dataFile.close();
    } // end of if datafile
    // if the file isn't open, pop up an error:
    else 
    {
      digitalWrite(ledBlue, HIGH);  digitalWrite(ledGreen, HIGH);  digitalWrite(ledRed, HIGH);
      Serial.println("error opening datalog.txt");
    } // end of else
  } // end of if bmp_pressure
  
  delay(10);
} // end of loop
