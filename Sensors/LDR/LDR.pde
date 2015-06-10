// Setup sensor on same ANALOG pin as defined below.
//run standard Firmata script on arduino FIRST 
// then run this script to gather data



import processing.serial.*;
import cc.arduino.*;

int sensor = 0;
int read = 0;
int width =0;
int height =0;

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
}

void draw()
{

  
  background(0);
  read = 1024 - arduino.analogRead(sensor);
  println(read);
  ellipseMode(CENTER);
  ellipse(width/2,height/2,read, read);

}
