
int led = 13;
int ldr = 0;



void setup() 
{
  Serial.begin(115200);
  // put your setup code here, to run once:
  pinMode(led,OUTPUT);

}

void loop() 
{
  // put your main code here, to run repeatedly:
  int wait = analogRead(ldr);
  Serial.println(wait);

  wait = map(wait, 0, 200, 0, 7xhff00);
  

  digitalWrite(led , HIGH);
  delay(wait);
  digitalWrite(led , LOW);
  delay(wait);


  Serial.print("Sensor value:   ");
  Serial.println(wait);

}
