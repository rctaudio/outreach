// Setup sensor on same ANALOG pin as defined below.
//run standard Firmata script on arduino FIRST 
// then run this script to gather data

import processing.serial.*;
import cc.arduino.*;

int sensor = 0;      // pin number for sensor data
float read = 0;        // value that is read from sensor
int width =0;        // global variable for storing screen width.  
int height =0;       // global variable for storing screen height.
int xPos = 0;        // global variable for current x position for drawing graph
int[] yPos;          // global array for storing corresponding y value of graph


Arduino arduino;     // create new arduino object, for reading analog data.
Serial myPort;       // create object from Serial class
String val;          // Data received from the serial port
int trigPin = 2;
int echoPin = 4;

void setup()
{
  initArduino();
  initScreen();
//  initSerial();
  
  yPos = new int[displayWidth];  //create an array for each xvalue of screen
  
 


}

void draw()
{

  //clear screen to allow new data to be drawn
  background(0);
  
  // read analog sensor and flip scale
  read = (arduino.analogRead(sensor))-600;
  if ((read >100) && (read <400))
  read =read*5 -400;
//  read*=3;

//  if (myPort.available() > 0)
//  {
//    read = float(myPort.readString());
//    read /= 200;
//    read *= 1023;
////      println(read);
////      if (read ==0)
////      read = 1023;
//  }
//  
//  long duration;
//  float inches, cm;
// 

  
  
  drawEllipse();
  drawText();  
  drawGraph();
} // end of draw()


int get_data()
{
  if (myPort.available() > 0)
    return myPort.read();
  return 1023;
} // end of get data()

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
//  initalize Serial              //
//  should be called in setup     //
//                                //
//--------------------------------//

void initSerial()
{
  String portName = Serial.list()[0];
  myPort = new Serial(this,portName, 115200);
  
}// end of initSerial()
  

//--------------------------------//
//                                //
//  initalize arduino             //
//  should be called in setup     //
//                                //
//--------------------------------//

void initArduino()
{
  arduino = new Arduino(this , Arduino.list()[0], 57600); 
//  arduino.pinMode(sensor, Arduino.INPUT);
  
//  arduino.pinMode(trigPin, Arduino.OUTPUT);
//  arduino.pinMode(echoPin, Arduino.INPUT);
  
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
  yPos[xPos] = height - (int)read;
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
