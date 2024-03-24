/* Pinout Definition */
#define triggerPin A2 
#define echoPin  A6

/* UltrasonicSensor */
int ultrasonicSensorTimeout = 0;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.write("Hello World\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
  ultrasonicSensor();
}


void ultrasonicSensor(){
  if(0!= ultrasonicSensorTimeout){
    ultrasonicSensorTimeout--;
  }
  else{
    ultrasonicSensorTimeout = 1000;
    ultrasonicSensor();
    float duration, distance;
    pinMode(triggerPin, OUTPUT);
    digitalWrite(triggerPin, LOW);
    delayMicroseconds(2);
    digitalWrite(triggerPin, HIGH);
    delayMicroseconds(10);
    digitalWrite(triggerPin, LOW);
    pinMode(echoPin, INPUT);
    duration = pulseIn(echoPin, HIGH);
    distance = (duration*.0343)/2;
    Serial.print("Distance : ");
    Serial.print(distance);
    Serial.print("cm");
    Serial.println();
  }
}