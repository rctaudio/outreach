// Setup sensor on same ANALOG pin as defined below.
//run standard Firmata script on arduino FIRST 
// then run this script to gather data



import processing.serial.*;
import cc.arduino.*;

int sensor = 0;
int read = 0;
int width =0;
int height =0;
int xPos = 0;
int[] yPos;


Arduino arduino;

void setup()
{
  arduino = new Arduino(this , Arduino.list()[0], 57600);
  arduino.pinMode(sensor, Arduino.INPUT);
  width = displayWidth -50;
  height = displayHeight -100;
  size(width,height);
  frameRate(30);
  stroke(255);
  yPos = new int[5000];
}

void draw()
{

  //read data and prepare circle
  background(0);
  read = 1024 - arduino.analogRead(sensor);
//  println(read);
  ellipseMode(CENTER);
  fill(255);
  ellipse(width/2,300,read/2, read/2);
  
  // draw text
  textSize(32);
  text("Analog Data: " + read/2,50,50);
  
  
  
  //draw graph
  noFill();

  yPos[xPos] = height - read/2;
  beginShape();
  curveVertex(0,yPos[0]);
  for (int i = 0; i< xPos; i++)
  {
    curveVertex(i,yPos[i]);
  }
  curveVertex(xPos, yPos[xPos]);
  endShape();

   //reset graph back to the beginning when it hits the end of the screen
   if (xPos >= width) 
   {
     xPos = 0;
     background(0); 
   } 
   else
   {
     xPos++;
   }
}
