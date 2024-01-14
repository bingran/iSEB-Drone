#include "Wire.h"
#include <MPU6050_light.h>

byte bMPU6050  = 0;
MPU6050 mpu(Wire);
long mpu6050timer = 0;

void setup() {
  
  Wire.begin();
  /* Serial */
  Serial.begin(115200);
  Serial.println("Hello World!");
  bMPU6050 = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(bMPU6050);
  if(0x00 == bMPU6050)
  {
    Serial.println(F("Calculating offsets, do not move MPU6050"));
  }
  delay(500); 
  if(0x00 == bMPU6050)
  {
    mpu.calcOffsets(true,true); // gyro and accelero
    Serial.println("offset Done!\n");
  }
}

void loop() {
  if(0x00 == bMPU6050)
  {
    if(millis() - mpu6050timer > 1000)
    { 
      mpu.update();
      Serial.print(F("TEMPERATURE: "));Serial.println(mpu.getTemp());
      Serial.print(F("ACCELERO  X: "));Serial.print(mpu.getAccX());
      Serial.print(" | Y: ");Serial.print(mpu.getAccY());
      Serial.print(" | Z: ");Serial.println(mpu.getAccZ());
    
      Serial.print(F("GYRO      X: "));Serial.print(mpu.getGyroX());
      Serial.print(" | Y: ");Serial.print(mpu.getGyroY());
      Serial.print(" | Z: ");Serial.println(mpu.getGyroZ());
    
      Serial.print(F("ACC ANGLE X: "));Serial.print(mpu.getAccAngleX());
      Serial.print(" | Y: ");Serial.println(mpu.getAccAngleY());
      
      Serial.print(F("ANGLE     X: "));Serial.print(mpu.getAngleX());
      Serial.print(" | Y: ");Serial.print(mpu.getAngleY());
      Serial.print(" | Z: ");Serial.println(mpu.getAngleZ());
      mpu6050timer = millis();
    }
  }
}