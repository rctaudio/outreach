/*----------------------------------------------------
          HARDWARE SETUP
RED    = 5v
BLACK  = GND 
WHITE  = Sensor - Analog pin 0

          SOFTWARE SETUP

1.  open arduino -> file -> examples -> firmata -> StandardFirmata -> upload to arduino
2.  open processing and click play
3.  test output and make sure it changes.  If it does not change you will need to change the variable CHANGEME
    


----------------------------------------------------
*/
int CHANGEME = 0;  // If no output change this to 1....2...3... in that order.

import processing.serial.*;
import cc.arduino.*;

int sensor = 0;      // pin number for sensor data
int read = 0;        // value that is read from sensor
int width =0;        // global variable for storing screen width.  
int height =0;       // global variable for storing screen height.
int xPos = 0;        // global variable for current x position for drawing graph
int[] yPos;          // global array for storing corresponding y value of graph


Arduino arduino;     // create new arduino object, for reading analog data.

void setup()
{
  initArduino();
  initScreen();
  
  yPos = new int[displayWidth];  //create an array for each xvalue of screen
}

void draw()
{

  //clear screen to allow new data to be drawn
  background(0);
  
  // read analog sensor "convert" to digital signal
  read = (arduino.analogRead(sensor));
//  if (read >1000) read = 500;
//  else read = 0;
  
  drawEllipse();
  drawText();  
  drawGraph();
} // end of draw()

//--------------------------------//
//                                //
//  initalize screen              //
// should be called in setup      //
//                                //
//--------------------------------//

void initScreen()
{
  width = displayWidth -50;
  height = displayHeight -100;
  size(width,height);
  frameRate(30);
  stroke(255);

}// end of initalize screen

//--------------------------------//
//                                //
//  initalize arduino             //
//  should be called in setup     //
//                                //
//--------------------------------//

void initArduino()
{
  arduino = new Arduino(this , Arduino.list()[CHANGEME], 57600); 
  arduino.pinMode(sensor, Arduino.INPUT);
}// end of initArduino

//--------------------------------//
//                                //
//             drawText           //
//  should be called when ever    //
//  you want to display data      //
//  as text                       //
//--------------------------------//

void drawText()
{
  textSize(32);
  text("Analog Data: " + read,50,50);
  
} // end of drawText

//--------------------------------//
//                                //
//       drawEllipse()            //
//  should be called when ever    //
//  you want to display data as   //
//  an ellipse                    //
//--------------------------------//

void drawEllipse()
{
  ellipseMode(CENTER);
  fill(255);
  ellipse(width/2,300,read, read);
  
}// end of drawEllipse()

//--------------------------------//
//                                //
//       drawGraph()              //
//  should be called when ever    //
//  you want to display data as   //
//  a graph                       //
//--------------------------------//

void drawGraph()
{
  noFill();
  yPos[xPos] = height - read;
  beginShape();
  curveVertex(0,yPos[0]);
  for (int i = 0; i< xPos; i++)
  {
    curveVertex(i,yPos[i]);
  }// end of for
  
  curveVertex(xPos, yPos[xPos]);
  endShape();

   //reset graph back to the beginning when it hits the end of the screen
   if (xPos >= width) 
   {
     xPos = 0;
     background(0); 
   } // end of if
   else
   {
     xPos++;
   } // end of else
}// end of drawGraph()
