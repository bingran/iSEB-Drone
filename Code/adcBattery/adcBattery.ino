/* Pinout Definition */
#define batteryVoltagePin A3

/* adcVoltage*/
long adcTimer = 1000;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  Serial.write("Hello World\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  if(millis() - adcTimer > 1000) // print data every second
  {
    float voltage = analogRead(batteryVoltagePin);
    Serial.print("voltage : ");
    Serial.print(((voltage)*5/1023)*12/2);     
    Serial.println("v");
    adcTimer = millis();
  }
}
