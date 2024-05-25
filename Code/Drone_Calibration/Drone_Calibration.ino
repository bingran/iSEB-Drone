///////////////////////////////////////////////////////////////////////////////////////
//Terms of use
///////////////////////////////////////////////////////////////////////////////////////
//THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
//IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
//FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
//AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
//LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
//OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
//THE SOFTWARE.
//
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you 
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////



///////////////////////////////////////////////////////////////////////////////////////
//The program will start in calibration mode.
//Send the following characters / numbers via the serial monitor to change the mode
//
//r = print receiver signals.
//a = print quadcopter angles.
//1 = check rotation / vibrations for motor 1 (right front CCW).
//2 = check rotation / vibrations for motor 2 (right rear CW).
//3 = check rotation / vibrations for motor 3 (left rear CCW).
//4 = check rotation / vibrations for motor 4 (left front CW).
//5 = check vibrations for all motors together.


#include <Wire.h>                                    //Include the Wire.h library so we can communicate with the gyro.
#include <EEPROM.h>                                  //Include the EEPROM.h library so we can store information onto the EEPROM
#include "RF24.h"
#include <Servo.h>

/* Pinout Definition */
#define rfCSNPin 10
#define pwmMotor4 9
#define buzzerPin 8
#define gpsTx 7
#define pwmMotor1 6
#define pwmMotor3 5
#define gpsRx 4
#define pwmMotor2 3

#define rfCEPin A0
#define sdCSPin A1

//Declaring global variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte eeprom_data[36], start, data;
boolean new_function_request,first_angle;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int counter_channel_1, counter_channel_2, counter_channel_3, counter_channel_4;
int receiver_input[5];
int loop_counter, gyro_address, vibration_counter;
int temperature;
long acc_x, acc_y, acc_z, acc_total_vector[20], acc_av_vector, vibration_total_result;
unsigned long timer,timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long zero_timer, timer_1, timer_2, timer_3, timer_4, current_time;

int acc_axis[4], gyro_axis[4];
double gyro_pitch, gyro_roll, gyro_yaw;
float angle_roll = 0;
float angle_roll_acc = 0;
float angle_pitch = 0;
float angle_pitch_acc = 0;

int cal_int;
double gyro_axis_cal[4];


/* MPU6050 */
#define MPU6050_ADDR                  0x68
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1c
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b

#define MPU6050_GYRO_OUT_REGISTER     0x43
#define MPU6050_ACCEL_OUT_REGISTER    0x3B

#define MPU6050_CAL_NUM               2000

// Motor
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 5000 // Maximum pulse length in µs

/* motor */
uint32_t double_lift = 0;
uint32_t double_yaw = 0;
uint32_t double_roll = 0;
uint32_t double_pitch = 0;
Servo mot1, mot2, mot3, mot4;
double esc_1 = MIN_PULSE_LENGTH;
double esc_2 = MIN_PULSE_LENGTH;
double esc_3 = MIN_PULSE_LENGTH;
double esc_4 = MIN_PULSE_LENGTH;
unsigned long motortimer = 0;

/* RF */
RF24 radio(rfCEPin,rfCSNPin);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
byte bRadio = false;
char radioMessage[6];  // only using 6 characters for TX & ACK payloads
int radioTimeout = 500;
uint8_t rfAddress[][6] = { "AAAAAA", "555555" };
uint8_t u8Yaw  = 0; /* left X */
uint8_t u8Lift  = 0; /* left Y */
uint8_t u8Roll = 0; /* Right X */
uint8_t u8Pitch = 0; /* RIght Y */
bool bRightButton = false;
bool bLeftButton = false;

//Setup routine
void setup(){

  Wire.begin();             //Start the I2C as master
  
  Serial.begin(115200);   
  Serial.println("Hello World!");   
  Wire.begin();   //Start the I2C as master.
  TWBR = 12;   //Set the I2C clock speed to 400kHz.      

  // //Arduino Uno pins default to inputs, so they don't need to be explicitly declared as inputs.
  mot1.attach(pwmMotor1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot2.attach(pwmMotor2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot3.attach(pwmMotor3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot4.attach(pwmMotor4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  /* set to minimum to stop the motor and also avoid beep sound */
  mot1.writeMicroseconds(MIN_PULSE_LENGTH); 
  mot2.writeMicroseconds(MIN_PULSE_LENGTH);
  mot3.writeMicroseconds(MIN_PULSE_LENGTH);
  mot4.writeMicroseconds(MIN_PULSE_LENGTH);                                                           // set PCINT3 (digital input 11)to trigger an interrupt on state change.

  for(data = 0; data <= 35; data++)eeprom_data[data] = EEPROM.read(data);               //Read EEPROM for faster data access
                                               
  set_gyro_registers();                                                                 //Set the specific gyro registers.

  wait_for_receiver(); /* set up rf module */                                                                 //Wait until the receiver is active.
  zero_timer = micros();                                                                //Set the zero_timer for the first loop.

  Serial.println("Press 'r' :Reading receiver signals.");
  Serial.println("Press 'a' :Print the quadcopter angles.");
  Serial.println("Press '1' :Test motor 1 (right front CCW.)");
  Serial.println("Press '2' :Test motor 2 (right rear CW.)");
  Serial.println("Press '3' :Test motor 3 (left rear CCW.)");
  Serial.println("Press '4' :Test motor 4 (left front CW.)");
  Serial.println("Press '5' :Test all motors together");
  
  while(Serial.available())data = Serial.read();                                        //Empty the serial buffer.
  data = 0;                                                                             //Set the data variable back to zero.
}

//Main program loop
void loop(){
  checkRFSignal(false);
  while(zero_timer + 4000 > micros());                                                  //Start the pulse after 4000 micro seconds.
  zero_timer = micros();                                                                //Reset the zero timer.

  if(Serial.available() > 0){
    data = Serial.read();                                                               //Read the incomming byte.
    delay(100);                                                                         //Wait for any other bytes to come in
    while(Serial.available() > 0)loop_counter = Serial.read();                          //Empty the Serial buffer.
    new_function_request = true;                                                        //Set the new request flag.
    loop_counter = 0;                                                                   //Reset the loop_counter variable.
    cal_int = 0;                                                                        //Reset the cal_int variable to undo the calibration.
    start = 0;                                                                          //Set start to 0.
    first_angle = false;                                                                //Set first_angle to false.
    //Confirm the choice on the serial monitor.
    if(data == 'r')Serial.println("Reading receiver signals.");
    if(data == 'a')Serial.println("Print the quadcopter angles.");
    if(data == 'a')Serial.println("Gyro calibration starts in 2 seconds (don't move the quadcopter).");
    if(data == '1')Serial.println("Test motor 1 (right front CCW.)");
    if(data == '2')Serial.println("Test motor 2 (right rear CW.)");
    if(data == '3')Serial.println("Test motor 3 (left rear CCW.)");
    if(data == '4')Serial.println("Test motor 4 (left front CW.)");
    if(data == '5')Serial.println("Test all motors together");

    //Let's create a small delay so the message stays visible for 2.5 seconds.
    //We don't want the ESC's to beep and have to send a 1000us pulse to the ESC's.
    for(vibration_counter = 0; vibration_counter < 625; vibration_counter++){           //Do this loop 625 times
      delay(3);                                                                         //Wait 3000us.
      esc_1 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc_2 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc_3 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc_4 = 1000;                                                                     //Set the pulse for ESC 1 to 1000us.
      esc_pulse_output();                                                               //Send the ESC control pulses.
    }
    vibration_counter = 0;                                                              //Reset the vibration_counter variable.
  }
  
  if(receiver_input_channel_2 < 1100)new_function_request = false;                      //If the throttle is in the lowest position set the request flag to false.

  ////////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a 'r' print the receiver signals.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 'r'){ 
    loop_counter ++;                                                                    //Increase the loop_counter variable.
    receiver_input_channel_1 = receiver_input_channel_1;                                //Convert the actual receiver signals for pitch to the standard 1000 - 2000us.
    receiver_input_channel_2 = receiver_input_channel_2;                                //Convert the actual receiver signals for roll to the standard 1000 - 2000us.
    receiver_input_channel_3 = receiver_input_channel_3;                                //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
    receiver_input_channel_4 = receiver_input_channel_4;                                //Convert the actual receiver signals for yaw to the standard 1000 - 2000us.

    if(loop_counter == 125){                                                            //Print the receiver values when the loop_counter variable equals 250.
      print_signals();                                                                  //Print the receiver values on the serial monitor.
      loop_counter = 0;                                                                 //Reset the loop_counter variable.
    }

    //For starting the motors: throttle low and yaw left (step 1).
    if(receiver_input_channel_3 < 1050 && receiver_input_channel_4 < 1050)start = 1;
    //When yaw stick is back in the center position start the motors (step 2).
    if(start == 1 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1450)start = 2;
    //Stopping the motors: throttle low and yaw right.
    if(start == 2 && receiver_input_channel_3 < 1050 && receiver_input_channel_4 > 1950)start = 0;

    esc_1 = 1000;                                                                       //Set the pulse for ESC 1 to 1000us.
    esc_2 = 1000;                                                                       //Set the pulse for ESC 1 to 1000us.
    esc_3 = 1000;                                                                       //Set the pulse for ESC 1 to 1000us.
    esc_4 = 1000;                                                                       //Set the pulse for ESC 1 to 1000us.
    esc_pulse_output();                                                                 //Send the ESC control pulses.
  }

  ///////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a '1, 2, 3, 4 or 5 test the motors.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == '1' || data == '2' || data == '3' || data == '4' || data == '5'){          //If motor 1, 2, 3 or 4 is selected by the user.
    loop_counter ++;                                                                    //Add 1 to the loop_counter variable.
    if(new_function_request == true && loop_counter == 250){                            //Wait for the throttle to be set to 0.
      Serial.print("Set throttle to 1000 (low). It's now set to: ");                    //Print message on the serial monitor.
      Serial.println(receiver_input_channel_2);                                         //Print the actual throttle position.
      loop_counter = 0;                                                                 //Reset the loop_counter variable.
    }
    if(new_function_request == false){                                                  //When the throttle was in the lowest position do this.                       //Convert the actual receiver signals for throttle to the standard 1000 - 2000us.
      if(data == '1' || data == '5')esc_1 = receiver_input_channel_2;                   //If motor 1 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc_1 = 1000;                                                                //If motor 1 is not requested set the pulse for the ESC to 1000us (off).
      if(data == '2' || data == '5')esc_2 = receiver_input_channel_2;                   //If motor 2 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc_2 = 1000;                                                                //If motor 2 is not requested set the pulse for the ESC to 1000us (off).
      if(data == '3' || data == '5')esc_3 = receiver_input_channel_2;                   //If motor 3 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc_3 = 1000;                                                                //If motor 3 is not requested set the pulse for the ESC to 1000us (off).
      if(data == '4' || data == '5')esc_4 = receiver_input_channel_2;                   //If motor 4 is requested set the pulse for motor 1 equal to the throttle channel.
      else esc_4 = 1000;                                                                //If motor 4 is not requested set the pulse for the ESC to 1000us (off).

      esc_pulse_output();                                                               //Send the ESC control pulses.
 
      //For balancing the propellors it's possible to use the accelerometer to measure the vibrations.
      if(eeprom_data[31] == 1){                                                         //The MPU-6050 is installed
        Wire.beginTransmission(0x68);                                           //Start communication with the gyro.
        Wire.write(0x3B);                                                               //Start reading @ register 43h and auto increment with every read.
        Wire.endTransmission();                                                         //End the transmission.
        Wire.requestFrom(0x68,6);                                               //Request 6 bytes from the gyro.
        while(Wire.available() < 6);                                                    //Wait until the 6 bytes are received.
        acc_x = Wire.read()<<8|Wire.read();                                             //Add the low and high byte to the acc_x variable.
        acc_y = Wire.read()<<8|Wire.read();                                             //Add the low and high byte to the acc_y variable.
        acc_z = Wire.read()<<8|Wire.read();                                             //Add the low and high byte to the acc_z variable.

        acc_total_vector[0] = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));          //Calculate the total accelerometer vector.

        acc_av_vector = acc_total_vector[0];                                            //Copy the total vector to the accelerometer average vector variable.

        for(start = 16; start > 0; start--){                                            //Do this loop 16 times to create an array of accelrometer vectors.
          acc_total_vector[start] = acc_total_vector[start - 1];                        //Shift every variable one position up in the array.
          acc_av_vector += acc_total_vector[start];                                     //Add the array value to the acc_av_vector variable.
        }

        acc_av_vector /= 17;                                                            //Divide the acc_av_vector by 17 to get the avarage total accelerometer vector.

        if(vibration_counter < 20){                                                     //If the vibration_counter is less than 20 do this.
          vibration_counter ++;                                                         //Increment the vibration_counter variable.
          vibration_total_result += abs(acc_total_vector[0] - acc_av_vector);           //Add the absolute difference between the avarage vector and current vector to the vibration_total_result variable.
        }
        else{
          vibration_counter = 0;                                                        //If the vibration_counter is equal or larger than 20 do this.
          Serial.println(vibration_total_result/50);                                    //Print the total accelerometer vector divided by 50 on the serial monitor.
          vibration_total_result = 0;                                                   //Reset the vibration_total_result variable.
        }
      }
    }
  }
  ///////////////////////////////////////////////////////////////////////////////////////////
  //When user sends a 'a' display the quadcopter angles.
  ////////////////////////////////////////////////////////////////////////////////////////////
  if(data == 'a'){

    if(cal_int != 2000){
      Serial.print("Calibrating the gyro");
      //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
      for (cal_int = 0; cal_int < 2000 ; cal_int ++){                                   //Take 2000 readings for calibration.
        if(cal_int % 100 == 0)Serial.print(F("."));                //Print dot to indicate calibration.
        gyro_signalen();                                                                //Read the gyro output.
        gyro_axis_cal[1] += gyro_axis[1];                                               //Ad roll value to gyro_roll_cal.
        gyro_axis_cal[2] += gyro_axis[2];                                               //Ad pitch value to gyro_pitch_cal.
        gyro_axis_cal[3] += gyro_axis[3];                                               //Ad yaw value to gyro_yaw_cal.
        delay(3);                                                                       //Wait 3 milliseconds before the next loop.
      }
      Serial.println(" ");
      //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
      gyro_axis_cal[1] /= 2000;                                                         //Divide the roll total by 2000.
      gyro_axis_cal[2] /= 2000;                                                         //Divide the pitch total by 2000.
      gyro_axis_cal[3] /= 2000; 
      
      Serial.print("gyro_axis_cal: ");
      Serial.print(gyro_axis_cal[0]);
      Serial.print(" ");
      Serial.print(gyro_axis_cal[1]);
      Serial.print("  ");
      Serial.println(gyro_axis_cal[2]);                                                        //Divide the yaw total by 2000.
    }
    else{
      delayMicroseconds(1000);     
      //Let's get the current gyro data.
      gyro_signalen();

      //Gyro angle calculations
      //0.0000611 = 1 / (250Hz / 65.5)
      angle_pitch += gyro_pitch * 0.0000611;                                           //Calculate the traveled pitch angle and add this to the angle_pitch variable.
      angle_roll += gyro_roll * 0.0000611;                                             //Calculate the traveled roll angle and add this to the angle_roll variable.

      //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) The Arduino sin function is in radians
      angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the roll angle to the pitch angel.
      angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                         //If the IMU has yawed transfer the pitch angle to the roll angel.

      //Accelerometer angle calculations
      acc_total_vector[0] = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));           //Calculate the total accelerometer vector.

      //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
      angle_pitch_acc = asin((float)acc_y/acc_total_vector[0])* 57.296;                //Calculate the pitch angle.
      angle_roll_acc = asin((float)acc_x/acc_total_vector[0])* -57.296;                //Calculate the roll angle.
      
      if(!first_angle){
        angle_pitch = angle_pitch_acc;                                                 //Set the pitch angle to the accelerometer angle.
        angle_roll = angle_roll_acc;                                                   //Set the roll angle to the accelerometer angle.
        first_angle = true;
      }
      else{
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;                 //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;                    //Correct the drift of the gyro roll angle with the accelerometer roll angle.
      }

      //We can't print all the data at once. This takes to long and the angular readings will be off.
      
      if(loop_counter == 0)Serial.print("Pitch: ");
      if(loop_counter == 1)Serial.print(angle_pitch ,0);
      if(loop_counter == 2)Serial.print(" Roll: ");
      if(loop_counter == 3)Serial.print(angle_roll ,0);
      if(loop_counter == 4)Serial.print(" Yaw: ");
      if(loop_counter == 5)Serial.print(gyro_yaw / 65.5 ,0);
      if(loop_counter == 6)Serial.print(" Yaw: ");
      if(loop_counter == 7)Serial.println(gyro_yaw / 65.5 ,0);
      if(loop_counter == 8)Serial.print(" acc_x: ");
      if(loop_counter == 9)Serial.print(acc_x);
      if(loop_counter ==10)Serial.print(" acc_y: ");
      if(loop_counter ==11)Serial.print(acc_y);
      if(loop_counter ==12)Serial.print(" acc_z: ");
      if(loop_counter ==13)Serial.println(acc_z);

      loop_counter ++;
      if(loop_counter == 60)loop_counter = 0;      
    }
  }
}

//Checck if the receiver values are valid within 10 seconds
void wait_for_receiver(){
 if (!radio.begin()) {
    bRadio = false;
    Serial.println(F("radio hardware is not responding!!"));
  }
  else
  {
    bRadio = true;
    
    // Set the PA Level low to try preventing power supply related problems
    // because these examples are likely run with nodes in close proximity to
    // each other.
    radio.setPALevel(RF24_PA_MAX);  // RF24_PA_MAX is default.
 
    // to use ACK payloads, we need to enable dynamic payload lengths (for all nodes)
    radio.enableDynamicPayloads();  // ACK payloads are dynamically sized

    // Acknowledgement packets have no payloads by default. We need to enable
    // this feature for all nodes (TX & RX) to use ACK payloads.
    radio.enableAckPayload();

    // set the TX address of the RX node into the TX pipe
    radio.openWritingPipe(rfAddress[1]);  // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1,rfAddress[0]);  // using pipe 1
    memset(radioMessage,0x00,6);
    radio.writeAckPayload(1, &radioMessage, sizeof(radioMessage));
    radio.startListening();  // put radio in RX mode
  }
  // byte bReceived = 0;
  // timer = millis() + 3000;
  // while(timer > millis()){
  
  //   if (true == checkRFSignal(true))
  //   {                     
  //     bReceived = 1;
  //     break;
  //   }
  //   delay(500);
  //   Serial.print(F("."));
  // }
  // if(bReceived == 0){
  //   Serial.println(F("."));
  //   Serial.println(F("No valid receiver signals found!!! (ERROR 1)"));
  // }
}

void print_signals(){
  Serial.print("Start:");
  Serial.print(start);

  Serial.print("  Roll:");
  if(receiver_input_channel_3 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_3 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_3);

  Serial.print("  Pitch:");
  if(receiver_input_channel_4 - 1480 < 0)Serial.print("^^^");
  else if(receiver_input_channel_4 - 1520 > 0)Serial.print("vvv");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_4);

  Serial.print("  Throttle:");
  if(receiver_input_channel_2 - 1480 < 0)Serial.print("vvv");
  else if(receiver_input_channel_2 - 1520 > 0)Serial.print("^^^");
  else Serial.print("-+-");
  Serial.print(receiver_input_channel_2);

  Serial.print("  Yaw:");
  if(receiver_input_channel_1 - 1480 < 0)Serial.print("<<<");
  else if(receiver_input_channel_1 - 1520 > 0)Serial.print(">>>");
  else Serial.print("-+-");
  Serial.println(receiver_input_channel_1);
}

void esc_pulse_output(){
  mot1.writeMicroseconds(esc_1); 
  mot2.writeMicroseconds(esc_2);
  mot3.writeMicroseconds(esc_3);
  mot4.writeMicroseconds(esc_4); 
}

void set_gyro_registers(){
  //Setup the MPU-6050
  Wire.beginTransmission(0x68);                        //Start communication with the address found during search.
  Wire.write(0x6B);                                            //We want to write to the PWR_MGMT_1 register (6B hex)
  Wire.write(0x00);                                            //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                      //End the transmission with the gyro.

  Wire.beginTransmission(0x68);                        //Start communication with the address found during search.
  Wire.write(0x1B);                                            //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                      //End the transmission with the gyro

  Wire.beginTransmission(0x68);                        //Start communication with the address found during search.
  Wire.write(0x1C);                                            //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                            //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();                                      //End the transmission with the gyro

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(0x68);                                //Start communication with the address found during search
  Wire.write(0x1B);                                            //Start reading @ register 0x1B
  Wire.endTransmission();                                      //End the transmission
  Wire.requestFrom(0x68, 1);                                   //Request 1 bytes from the gyro
  while(Wire.available() < 1);                                 //Wait until the 6 bytes are received
  if(Wire.read() != 0x08){                                     //Check if the value is 0x08
    digitalWrite(12,HIGH);                                     //Turn on the warning led
    while(1)delay(10);                                         //Stay in this loop for ever
  }

  Wire.beginTransmission(0x68);                                //Start communication with the address found during search
  Wire.write(0x1A);                                            //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                            //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                      //End the transmission with the gyro    
}

void gyro_signalen(){
  //Read the MPU-6050
  Wire.beginTransmission(0x68);                                //Start communication with the gyro.
  Wire.write(0x3B);                                            //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                      //End the transmission.
  Wire.requestFrom(0x68,14);                                   //Request 14 bytes from the gyro.
  while(Wire.available() < 14);                                //Wait until the 14 bytes are received.
  acc_axis[1] = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the acc_x variable.
  acc_axis[2] = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the acc_y variable.
  acc_axis[3] = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the acc_z variable.
  temperature = Wire.read()<<8|Wire.read();                    //Add the low and high byte to the temperature variable.
  gyro_axis[1] = Wire.read()<<8|Wire.read();                   //Read high and low part of the angular data.
  gyro_axis[2] = Wire.read()<<8|Wire.read();                   //Read high and low part of the angular data.
  gyro_axis[3] = Wire.read()<<8|Wire.read();                   //Read high and low part of the angular data.

  if(cal_int == 2000){
    gyro_axis[1] -= gyro_axis_cal[1];                            //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                            //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                            //Only compensate after the calibration.
  }

  gyro_roll   = gyro_axis[0];
  gyro_pitch  = gyro_axis[1];
  gyro_yaw    = gyro_axis[2];
  acc_x       = acc_axis[0];
  acc_y       = acc_axis[1];
  acc_z       = acc_axis[2];
}

bool checkRFSignal(bool printout)
{
  uint8_t pipe;
  if (radio.available(&pipe))
  {                     
    // is there a payload? get the pipe number that recieved it
    // load the payload for the first received transmission on pipe 0
    // radio.writeAckPayload(1,&radioMessage,6);
    uint8_t bytes = radio.getDynamicPayloadSize();  // get the size of the payload
    char received[6];
    radio.read(received, sizeof(received));  // get incoming ACK payload
    u8Yaw = received[0];
    u8Lift = received[1];
    u8Roll = received[2];
    u8Pitch = received[3];
    double_yaw   =  ((uint32_t)u8Yaw   * 900 / 255 )+ 1050 ; 
    double_lift  =  ((uint32_t)u8Lift  * 900 / 255 )+ 1050 ; 
    double_roll  =  ((uint32_t)u8Roll  * 900 / 255 )+ 1050 ; 
    double_pitch =  ((uint32_t)u8Pitch * 900 / 255 )+ 1050 ; 
    receiver_input_channel_1 = double_yaw;
    receiver_input_channel_2 = double_lift;
    receiver_input_channel_3 = double_roll;
    receiver_input_channel_4 = double_pitch;
    if(true == printout)
    {
      Serial.println(" ");  
      Serial.print(F("Recieved on signal "));
      Serial.print(double_yaw);     
      Serial.print(" | ");
      Serial.print(double_lift);     
      Serial.print(" | ");
      Serial.print(double_roll);     
      Serial.print(" | ");
      Serial.println(double_pitch);  
    }
    radio.flush_rx();
    return true;
  }
  return false;
}





