int sensorPin = 3;    // select the input pin for the potentiometer
int ledPin = 13;      // select the pin for the LED
int sensorValue = 0;  // variable to store the value coming from the sensor

void setup() {
 pinMode(ledPin, OUTPUT); 
 pinMode(sensorPin, INPUT);
  Serial.begin(9600); 
}

void loop() {
 
  sensorValue = analogRead(sensorPin);    
 delay(sensorValue);
 Serial.println(sensorValue);  
}

