#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM303_U.h>
#include <Adafruit_BMP085_U.h>
#include <Adafruit_10DOF.h>
#include <Adafruit_L3GD20_U.h>

#include <MatrixMath.h>


#include <SPI.h>
#include <SD.h>
int CS = 10;


/* Assign a unique ID to the sensors */
Adafruit_10DOF                dof   = Adafruit_10DOF();
Adafruit_LSM303_Accel_Unified accel = Adafruit_LSM303_Accel_Unified(30301);
Adafruit_LSM303_Mag_Unified   mag   = Adafruit_LSM303_Mag_Unified(30302);
Adafruit_BMP085_Unified       bmp   = Adafruit_BMP085_Unified(18001);

/* Update this with the correct SLP for accurate altitude measurements */
float seaLevelPressure = SENSORS_PRESSURE_SEALEVELHPA;

float initialAlt = 0.0;

/**************************************************************************/
/*!
    @brief  Initialises all the sensors used by this example
*/
/**************************************************************************/
void initSensors()
{
  if(!accel.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println(F("Ooops, no LSM303 detected ... Check your wiring!"));
    while(1);
  }
  if(!mag.begin())
  {
    /* There was a problem detecting the LSM303 ... check your connections */
    Serial.println("Ooops, no LSM303 detected ... Check your wiring!");
    while(1);
  }
  if(!bmp.begin())
  {
    /* There was a problem detecting the BMP180 ... check your connections */
    Serial.println("Ooops, no BMP180 detected ... Check your wiring!");
    while(1);
  }
}

//Kalman main variables
//define main variables

int u = 0;                                                                                  //acceleration magnitude
int Q[3] = {0,0,0};                                                                         //initalized state of the sensor [position; velocity; acceleration]
float Q_estimate[3];
float Sensor_pressure_noise_mag = .2;                                                       //process noise: the variability in how fast the Quail is speeding up (stdv of acceleration: meters/sec^2)
float arduino_noise_mag = 1.0;                                                              //measurement noise: How mask-blinded is the Ninja (stdv of location, in meters)
float Ez = arduino_noise_mag*arduino_noise_mag;                                             //Ez convert the measurement noise (stdv) into covariance matrix
float Ex[3][3];
float blah[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 1}};                                             //Ex convert the process noise (stdv) into covariance matrix
float spnm[1] = {Sensor_pressure_noise_mag*Sensor_pressure_noise_mag};
float P[3][3];                                                                         //estimate of initial Quail position variance (covariance matrix)
  

/**************************************************************************/
/*!

*/
/**************************************************************************/
void setup(void)
{
  Serial.begin(115200);
  Serial.println(F("Adafruit 10 DOF Pitch/Roll/Heading Example")); Serial.println("");
  int lim = 50; // number of times to average starting height over
  /* Initialise the sensors */
  initSensors();
  sensors_event_t bmp_event;
  int i = 0;
  while(i <lim)
  {
    bmp.getEvent(&bmp_event);
    if (bmp_event.pressure)
    {
      i++;
      /* Get ambient temperature in C */
      float temperature;
      bmp.getTemperature(&temperature);
      /* Convert atmospheric pressure, SLP and temp to altitude */
      initialAlt +=  bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature); 
    } // end of if
  
  }// end of while
  initialAlt /= lim;
  
  Serial.print("Initializing SD card...");

  // see if the card is present and can be initialized:
  if (!SD.begin(CS)) {
    Serial.println("Card failed, or not present");
    // don't do anything more:
    return;
  }
  Serial.println("card initialized.");
  
  String dataString = "Altitude,DeltaT,Temperature,Initial_Height,Relative_Height,Ardy_Kalman";
  // open the file. note that only one file can be open at a time,
  // so you have to close this one before opening another.
  File dataFile = SD.open("datalog.csv", FILE_WRITE);
  // if the file is available, write to it:
  if (dataFile) 
  {
    dataFile.println(dataString);
    dataFile.close();
    // print to the serial port too:
    Serial.println(dataString);
  }
  // if the file isn't open, pop up an error:
  else 
    Serial.println("error opening datalog.txt");
  
 
 //some Kalman setup stuff
 float temp[3];
 Matrix.Transpose((float*)Q, 3, 1, (float*)temp);
 Matrix.Copy((float*)temp, 1, 3, (float*)Q);
 Matrix.Copy((float*)Q,1,3,(float*)Q_estimate);                //x_estimate of initial location of where the sensor is 
 Matrix.Multiply((float*)spnm , (float*)blah, 1, 3, 3, (float*)Ex);
 Matrix.Copy((float*)Ex, 3,3, (float*)P);

 
} // end of setup

/**************************************************************************/
/*!
    @brief  Constantly check the roll/pitch/heading/altitude/temperature
*/
/**************************************************************************/



void loop(void)
{
  // sting for datalog
  String dataString = "";
  long time;                    // current time
  int deltaT;                   // time delta 
  float alt;                    // current alt
    
  sensors_event_t bmp_event;
  bmp.getEvent(&bmp_event);    // try to get a current reading
  while(!bmp_event.pressure)   // if reading is not good keep trying
    bmp.getEvent(&bmp_event);
     
  long prevTime = millis();    // good reading, now get time ( to be used as previous reading
  float temperature;          
  bmp.getTemperature(&temperature);   

  /* Calculate the altitude using the barometric pressure sensor */
  bmp.getEvent(&bmp_event);
  if (bmp_event.pressure)
  {
    /* Get ambient temperature in C */
    time = millis();
    bmp.getTemperature(&temperature);
    deltaT = time - prevTime;
    prevTime = time;
        
    /* Convert atmospheric pressure, SLP and temp to altitude */
    alt = (bmp.pressureToAltitude(seaLevelPressure, bmp_event.pressure, temperature)); 
    
    ///// Kalman Filter stuffs ////////
    
    float A[3][3] = {{1, deltaT, ((deltaT*deltaT)/2)}, {0, 1, deltaT}, {0, 0, 1}};  //state transistion matrix: expected height of the sensor (state prediction)
    float B[1] = {0};                                                           //input control matrix: there is no control, so this is zero?     
    float C[3] = {1, 0, 0};                                                       //measurement matrix: the expected measurement given the predicted state
    
    float Q_loc_meas[1] = {alt};
    float Q_loc_estimate[1] = {0};     //Quail position estimate
    float vel_estimate[1] = {0};       //Quail velocity estimate
    float acc_estimate[1] = {0};       //Quail ackbar estimate
    float P_estimate[3][3];
    Matrix.Copy((float*)P,3, 3, (float*)P_estimate);
    float P_mag_estimate[1] = {0};
    float predic_state[1] = {0};
//    float predic_var[3] = {0,0,0};   //probally dont need
    
    
    
    // Predict next state of the quail with the last state and predicted motion.
    float temp1[3];
    Matrix.Multiply((float*)A, (float*)Q_estimate, 3, 3, 1, (float*)temp1) ;
    float temp2[3];
    Matrix.Multiply((float*)B, (float*)u, 3, 3, 1,(float*)temp2 );
    Matrix.Add((float*)temp1, (float*)temp2, 3, 1, (float*)Q_estimate);
    
    //predict next covariance
    //P = A * P * A' + Ex;

    float temp3[3][3];
    float temp4[3][3];
    Matrix.Transpose((float*)A, 3, 3, (float*)temp3);
    Matrix.Multiply((float*)A, (float*)P, 3, 3, 3, (float*)temp4);
    Matrix.Multiply((float*)temp4, (float*)temp3, 3, 3, 3, (float*)P);  
    Matrix.Add((float*)P, (float*)Ex, 3, 3, (float*)temp3);
    Matrix.Copy((float*)temp3, 3, 3, (float*)P);
    
//    predic_var = [predic_var; P] ;   // dont think I need to add this....
    
    // predicted Ninja measurement covariance
    
    // Kalman Gain
    //K = P*C'*inv(C*P*C'+Ez);
    Matrix.Transpose((float*)C,1, 3, (float
    
    
    // Update the state estimate.
    
//    Q_estimate = Q_estimate + K * (Q_loc_meas(t) - C * Q_estimate);
    
    // update covariance estimation.
    
//    P =  (eye(3)-K*C)*P;
    
    //Store for plotting
    
//    Q_loc_estimate = [Q_loc_estimate; Q_estimate(1)];
//    vel_estimate = [vel_estimate; Q_estimate(2)];
//    acc_estimate = [acc_estimate; Q_estimate(3)];
//    P_mag_estimate = [P_mag_estimate; P(1)];

    
   
    dataString += alt;
    dataString += ",";
    dataString += deltaT;
    dataString += ",";
    dataString += temperature;
    dataString += ",";
    dataString += initialAlt;
    dataString += ",";
    dataString += (alt - initialAlt);
    dataString += ",";
    
    // open the file. note that only one file can be open at a time,
    // so you have to close this one before opening another.
    File dataFile = SD.open("datalog.csv", FILE_WRITE);
    // if the file is available, write to it:
    if (dataFile) 
    {
      dataFile.println(dataString);
      dataFile.close();
      // print to the serial port too:
      Serial.println(dataString);
      
    } // end of if
    // if the file isn't open, pop up an error:
    else 
      Serial.println("error opening datalog.txt");  
  }// end of if  
  
  Matrix.Copy((float*)Q,3,1,(float*)Q_estimate);                //x_estimate of initial location of where the sensor is 

} // end of loop


