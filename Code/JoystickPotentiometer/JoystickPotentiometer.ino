/* Pinout Definition */
#define rightYPin   A3
#define rightXPin   A2
#define leftYPin    A1
#define leftXePin   A0

/* status of the potentiometer */
float flLeftX = 0;
float flLeftY = 0;
float flRightX = 0;
float flRightY = 0;

/* Period of checking the status of the potentiometer */
int timeout = 500;

void setup() {
  /* Serial */
  Serial.begin(115200);
  Serial.println("Hello World!");
}

void loop() {
  // put your main code here, to run repeatedly:
   delay(1);
  if(0 != timeout)
  {
    timeout--;
    if(0 == timeout)
    {
      timeout = 1000;
      flLeftX = analogRead(leftXePin);
      flLeftY = analogRead(leftYPin);
      flRightX = analogRead(rightXPin);
      flRightY = analogRead(rightYPin);
      Serial.print((flLeftX));     
      Serial.print(" | ");
      Serial.print((flLeftY));     
      Serial.print(" | ");
      Serial.print((flRightX));     
      Serial.print(" | ");
      Serial.println((flRightY));  
    }
  } 
}
