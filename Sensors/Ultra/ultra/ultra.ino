#define trigPin 2
#define echoPin 4

//plug green wire into digital pin 2
// plug white wire into digital pin 4
 
void setup() {
  Serial.begin (57600);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}
 
void loop() {
  long distance;
  float cm = 0.0;
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); 
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10); 
  digitalWrite(trigPin, LOW);
  distance = pulseIn(echoPin, HIGH,10000);
  cm = microsecondsToCentimeters(distance);
  
  distance = pulseIn(echoPin, HIGH,10000);
  cm += microsecondsToCentimeters(distance);
  
 
  
    Serial.print(cm/2);
  
  delay(90);
}


float microsecondsToCentimeters(long microseconds)
{
  // The speed of sound is 340 m/s or 29 microseconds per centimeter.
  // The ping travels out and back, so to find the distance of the
  // object we take half of the distance travelled.
  return microseconds / 29.0 / 2.0;
}
