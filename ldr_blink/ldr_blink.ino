/*-------------------------
 * Use an LDR to blink an led
 * 
 * HARDWARE SETUP
 * refer to picture handout
 * 
 *
 * this should turn an led on when you cover the sensor
 * 
 */



// create GLOBAL variables for the pins that the LDR and LED use
int ldr = 0;
int led = 13;
int ldr_initial = 0; // creating a global variable for the inital value of the LDR so that we can check to see if the sensor has change
int balance = 10; // number of times to measure the starting value of the ldr sensor.  used to get a good starting point. AKA take the average of readings

void setup()
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(led, OUTPUT);               // set the LED pin to an output, so we can use it as a switch

  //average the "Starting value" of the ldr over a number of readings
  for(int i = 0; i < balance; i++)
  {
    ldr_initial += analogRead(ldr);
  }// end of for
  ldr_initial /= balance;
} // end of setup

void loop()
{
  //first we need to read a value from the ldr
  int sensor = analogRead(ldr);

  // then we need to see if it has changed, but the sesnor is noisy, so it doesnt always read the same value, it flucuates, so we are going to set a "threshold"
  if( (ldr_initial - sensor ) > 20 )
  {
    //in here the sensor has reached the threshold so lets turn the LED on
    digitalWrite(led, HIGH);
  } // end of if
  else
  {
    // in here the sensor has not reached the threshold so lets make sure the LED is off
    digitalWrite(led,LOW);
  }// end of else

  
  Serial.print("ldr initial: ");
  Serial.print(ldr_initial);
  Serial.print("      ldr: ");
  Serial.println(sensor);
  
  


} // end of loop()
