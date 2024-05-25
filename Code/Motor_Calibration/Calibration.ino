/*ESC calibration sketch; author: ELECTRONOOBS */
#include <Servo.h>

#define MAX_PULSE_LENGTH 2000
#define MIN_PULSE_LENGTH 1000

#define pwmMotor4 9
#define pwmMotor1 6
#define pwmMotor3 5
#define pwmMotor2 3

int DELAY = 1000;

Servo mot1, mot2, mot3, mot4;

void setup() {
  Serial.begin(9600);
  
  Serial.println("How to calibrate motor driver");
  Serial.println("Power off the motor driver");
  Serial.println("Send maximum speed before power on");
  Serial.println("Turn on power source and wait for 2 second");
  Serial.println("Motor driver will enter configuration mode");
  Serial.println("Send minimum speed");
  Serial.println("Calibration will be done.");
  Serial.println("Send any value between 1000 to 20000 to start the motor.");
  Serial.println("1000 is motor stop and 2000 is full throttle");
  
  
  Serial.println("Calibration start.");
  Serial.println("Turn off power source and press any key.");
  // Wait for input
  while (!Serial.available());
  Serial.read();
 
  Serial.println("Sending maximum speed before power on");
  Serial.println("Turn on power source and press any key.");
  
  // Wait for input
  while (!Serial.available());
  Serial.read();  

  Serial.println("Wait for 2 second and motor driver will enter configuration mode");
  mot1.attach(pwmMotor4);
  mot2.attach(pwmMotor1);
  mot3.attach(pwmMotor3);
  mot4.attach(pwmMotor2);

  mot1.writeMicroseconds(MAX_PULSE_LENGTH);
  mot2.writeMicroseconds(MAX_PULSE_LENGTH);
  mot3.writeMicroseconds(MAX_PULSE_LENGTH);
  mot4.writeMicroseconds(MAX_PULSE_LENGTH);
  
  delay(1000);

  Serial.println("Press any key to continue");
  
  // Wait for input
  while (!Serial.available());
  Serial.read();
  
  delay(1000);
  // Send min output

  Serial.println("Sending minimum speed");
  
  mot1.writeMicroseconds(MIN_PULSE_LENGTH);
  mot2.writeMicroseconds(MIN_PULSE_LENGTH);
  mot3.writeMicroseconds(MIN_PULSE_LENGTH);
  mot4.writeMicroseconds(MIN_PULSE_LENGTH);
  
  Serial.println("Now, type a values between 1000 and 2000 and press enter");
  Serial.println("and the motor will start rotating.");
  Serial.println("Send 1000 to stop the motor and 2000 for full throttle");

}

void loop() {
   
  if (Serial.available() > 0)
  {
    int DELAY = Serial.parseInt();
    if (MIN_PULSE_LENGTH <= DELAY && MAX_PULSE_LENGTH >= DELAY)
    {
      mot1.writeMicroseconds(DELAY);
      mot2.writeMicroseconds(DELAY);
      mot3.writeMicroseconds(DELAY);
      mot4.writeMicroseconds(DELAY);
      float SPEED = (DELAY-1000)/10;
      Serial.print("Motor speed:  "); 
      Serial.print(SPEED);
      Serial.println("%"); 
    } 
	else
	{
		Serial.println("Wrong input"); 
		Serial.println("Type a values between 1000 and 2000 and press enter");
	}
  }
}