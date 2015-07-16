// process noise     = .02
// measurement noise = .009

/***********************************************************************
*  
*                       GLOBAL STUFFS 
*    
*  This is where the things go that you need access to from EVERY
*  PART of your code.  This would be your "Cell phone" that you take with
*  you everywhere that has all the info you need
*/
/***********************************************************************/


// these includes allow us to use someone elses code.  This way we dont have to figure out every little detail
// , instead we can focus on what we want to do.
#include <Wire.h>                   // used to communicate with the IMU
#include <SPI.h>                    // used to communicate with the sd card 
#include <SD.h>                     // tells the arduino how to use the sd card
#include "MatrixMath.h"             // maths to make graphs pretty
#include "Adafruit_Sensor.h"        // tells the arduino how to use a sensor (general)
#include "Adafruit_BMP085_U.h"      // tells the arduino how to use the pressure sensor (specific)
#include <Servo.h>                  // tells the arduino how to use a servo

//IMU Stuffs
Adafruit_BMP085_Unified   bmp   = Adafruit_BMP085_Unified(18001); // assign a unique ID to the sensor
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;            // sets a reference value for sea level to calculate our height.

// LED Stuffs.  "const" means constant, this keyword will prevent you from accidently
const int ledBlue  = 10;       // reassigning this variable in your code
const int ledRed   = 8;
const int ledGreen = 9;     


// SD card Stuffs: 
const int chipSelect = 4;

// Servo Stuffs
Servo servoParachute;
const int servo = 6;

// Logic Stuffs, These are things used to figure out WHEN to deploy the parachute
float altInitial     = 0;         // set a reference height of where the rocket is BEFORE it is launched
float altCurrent     = 0;
float velocity       = 0;
float accel          = 0;
int launched         = 0;         // this is our "flag" that will tell the arduino it is moving
int parachuteDeploy  = 0;         // this is our "flag" that will tell the arduino its time to SLOW DOWN 
int average          = 100;       // used to average the altInitial reading, to provide a good starting point
float launchTime     = 0.0;         // time that we launched
float altDeployTime  = 0.0;
float failSafe       = 0.0;
float altMax         = 0.0;


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
float sdevProcess = 0.02;
float sdevMeasurement = .009;

//--------------------------END OF GLOBAL STUFFS--------------------


/**************************************************************************/
/*!
                      void setup
       this is where we put all our code that needs to be SETUP first.  
       If we need to INITALIZE any PINS, we set that here.  If we need to
       provide VISUAL feedback that our system is running correctly, we set that here.
       If we want to use the SERIAL terminal to see the data on the arduino, we set that here
       ... If we want to do something ONCE we put it in here.

       Do not declare varaibles in here, or else they will disappear after the 
       brackets close.
*/
/**************************************************************************/
void setup(void)
{
  // LED Stuffs
  pinMode(ledBlue, OUTPUT);
  pinMode(ledGreen, OUTPUT);
  pinMode(ledRed, OUTPUT);
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

  //Servo Stuffs
  servoParachute.attach(servo);
  servoParachute.write(120);
  
  //enable the Serial(magnifying glass button) to help us debug our program if needed
  Serial.begin(115200);
  Serial.println(F("Surfing on a rocket")); Serial.println("");
  
  // Sensor setup Stuffs
  initSensors();
  sensors_event_t bmp_event;
  int counter = 0;
  while(true)
  {
    bmp.getEvent(&bmp_event);
    if(bmp_event.pressure)
    {
      float temperature;
      bmp.getTemperature(&temperature); // get the ambient temperature as a reference
      altInitial += bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature);
      counter++;
      if (counter > average) break;     
    }// end of if(bmp_event.pressure)
  }// end of while(true)
  altInitial /= average;

  //Kalman filter maths
  X_old[0][0] = altInitial;
  t_old = millis()/1000.0;
  X_old[1][0] = 0;
  X_old[2][0] = 0;
  P[0][0] = 1000; P[0][1] = 0; P[0][2] = 0;
  P[1][0] = 0; P[1][1] = 1000; P[1][2] = 0; 
  P[2][0] = 0; P[2][1] = 0; P[2][2] = 1000;
  
  // Sd card Stuffs
  pinMode(10,OUTPUT); // set the default chipselect pin to output, for stability
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

//---------------------end of setups-----------------------

/****************************************************************
*             
*                          void loop
*      This is our "main" program.  This is what will be ran
*      over and over again.
* 
* 
* 
/*****************************************************************/

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
        
    //----------------------kalman filter / pretty maths---------------------------------
    //REALLY COOL STUFF, but higher level maths/ probability dont have to understand it.
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

    //Predict
    Matrix.Multiply((float*)F,(float*)X_old,3,3,1,(float*)X_prior); // Predict State
    Matrix.Transpose((float*)F,3,3,(float*)temp33_1); // Predict Covar
    Matrix.Multiply((float*)P,(float*)temp33_1,3,3,3,(float*)temp33_2);
    Matrix.Multiply((float*)F,(float*)temp33_2,3,3,3,(float*)temp33_1);
    Matrix.Add((float*)temp33_1,(float*)Q,3,3,(float*)P);
    
    
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

    //---------------------end of kalman filter / pretty maths


    //--------------------------------------------logics go here----------------------------------------
  
    //have we launched? if my height has changed by more than 5 meters, then I think we have.
    // i'm also using an absolute value since there is a pressure spike at launch
    if ( ( ( ( altInitial - X_est[0][0] ) > 4 ) || ( ( altInitial - X_est[0][0] ) < -4 ) ) && ( launched != 1 )  )
    {
      launched = 1;
      launchTime = millis()/1000.0;    
    } // end of if we have launched

    // are we at the apex of our parabola?  if so then maybe deploy parachut
    if ( X_est[0][0] > altMax) altMax = X_est[0][0];
    //if we have launched, AND our current position is higher than our start, AND we are more than 5 meters away from our largest height
    // AND alt deployTime has NOT happened yet
    if ( ( launched > 0 ) &&  ( X_est[0][0] > altInitial ) && ( ( altMax - X_est[0][0] ) > 4 ) && ( altDeployTime < 0.5 ) )
    {
      parachuteDeploy = 1;
      altDeployTime = millis()/1000.0;
    } // deploy parachute becuase we are on the downslope of our position parabola

    // what about time?
    // past launches have taken about 3-4 seconds to hit the apex before coming down so lets shoot for 4.5 seconds
    if ( ( launched > 0 ) && ( ( ( millis() / 1000.0 ) - launchTime ) > 4.5 ) )
    {
      parachuteDeploy = 1;
      failSafe = millis()/1000.0;
    } // end of time failsafe

    // come back and add flags for velocity and acceleration, if wanted
 
    //-----------------------------------------end of logics---------------------------------------

    //----------------------------------------sd card logging---------------------------------------
        
    // LOGGING to the sd card now, so far we have made variables
    // to store data in and now we will write those variables to 
    // a file so we can look at them later     
     
    File dataFile = SD.open("datalog.csv", FILE_WRITE);

    // if the file is available, write to it:
    if (dataFile) 
    {
      if(launched != 1) digitalWrite(ledGreen, LOW);
      dataFile.print(millis()/1000.0);
      dataFile.print(",");
      dataFile.print(Z[0][0]);
      dataFile.print(",");
      dataFile.print(X_est[0][0]);
      dataFile.print(",");
      dataFile.print(X_est[1][0]);
      dataFile.print(",");
      dataFile.print(X_est[2][0]);
      dataFile.print(",");
      dataFile.print(launchTime);
      dataFile.print(",");
      dataFile.print(failSafe);
      dataFile.print(",");
      dataFile.print(altDeployTime);
      dataFile.print(",");
      dataFile.println(launched);
      dataFile.close();
    } // end of if datafile
    // if the file isn't open, pop up an error:
    else 
    {
      digitalWrite(ledBlue, HIGH);
      digitalWrite(ledGreen, HIGH);
      digitalWrite(ledRed, HIGH);
      Serial.println("error opening datalog.txt");
    } // end of else
  } // end of if bmp_pressure

  //---------------------end of sd logging------------------------

  //---------------time to do some stuffs with the logics made above---------------------

  if (launched == 1) 
  {
    digitalWrite(ledRed,LOW);
    digitalWrite(ledGreen, HIGH);
  } // end of if
  else
    digitalWrite(ledRed,HIGH);

  if ( parachuteDeploy == 1)
  {
    servoParachute.write(0);
    digitalWrite(ledBlue,LOW); 
  } // end of if parachutedeploy
  else
  {
    servoParachute.write(120);
    digitalWrite(ledBlue,HIGH);
  }// end of else
  delay(10);

  //----------------------end of time to do stuff with logics------------------------
} // end of loop

//------------------------------------end of main loop---------------------------




/**************************************************************************

                     Other functions
    This is where we put other "functions" we will use. we usually use functions to make 
    code more readable, and easier to understand.  Instead of having a big block of code
    that has all the details of making coffee, we could make a function void coffee() that
    has all the code so that we just have "call" in our main program.


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

