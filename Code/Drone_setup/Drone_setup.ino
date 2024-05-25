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
///////////////////////////////////////////////////////////////////////////////////////
//Safety note
///////////////////////////////////////////////////////////////////////////////////////
//Always remove the propellers and stay away from the motors unless you 
//are 100% certain of what you are doing.
///////////////////////////////////////////////////////////////////////////////////////

#include <Wire.h>               //Include the Wire.h library so we can communicate with the gyro
#include <EEPROM.h>             //Include the EEPROM.h library so we can store information onto the EEPROM
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

//Declaring Global Variables
byte last_channel_1, last_channel_2, last_channel_3, last_channel_4;
byte lowByte, highByte, type, gyro_address, error, clockspeed_ok;
byte channel_1_assign, channel_2_assign, channel_3_assign, channel_4_assign;
byte roll_axis, pitch_axis, yaw_axis;
byte receiver_check_byte, gyro_check_byte;
volatile int receiver_input_channel_1, receiver_input_channel_2, receiver_input_channel_3, receiver_input_channel_4;
int center_channel_1, center_channel_2, center_channel_3, center_channel_4;
int high_channel_1 = 0;
int high_channel_2 = 0;
int high_channel_3 = 0;
int high_channel_4 = 0;
int low_channel_1 = 0; 
int low_channel_2 = 0; 
int low_channel_3 = 0; 
int low_channel_4 = 0;
int address, cal_int;
unsigned long timer, timer_1, timer_2, timer_3, timer_4, current_time;
float gyro_pitch, gyro_roll, gyro_yaw;
float gyro_roll_cal, gyro_pitch_cal, gyro_yaw_cal;


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
  mot1.attach(pwmMotor1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot2.attach(pwmMotor2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot3.attach(pwmMotor3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot4.attach(pwmMotor4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  /* set to minimum to stop the motor and also avoid beep sound */
  mot1.writeMicroseconds(MIN_PULSE_LENGTH); 
  mot2.writeMicroseconds(MIN_PULSE_LENGTH);
  mot3.writeMicroseconds(MIN_PULSE_LENGTH);
  mot4.writeMicroseconds(MIN_PULSE_LENGTH);Wire.begin();             //Start the I2C as master
  Serial.begin(115200);      //Start the serial connetion @ 115200bps
  delay(250);               //Give the gyro time to start 
}
//Main program
void loop(){
  //Show the YMFC-3D V2 intro
  intro();
  
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  Serial.println(F("System check"));
  Serial.println(F("==================================================="));
  delay(1000);
  Serial.println(F("Checking I2C clock speed."));
  delay(1000);
  
  TWBR = 12;                      //Set the I2C clock speed to 400kHz.
  
  #if F_CPU == 16000000L          //If the clock speed is 16MHz include the next code line when compiling
    clockspeed_ok = 1;            //Set clockspeed_ok to 1
  #endif                          //End of if statement

  if(TWBR == 12 && clockspeed_ok){
    Serial.println(F("I2C clock speed is correctly set to 400kHz."));
  }
  else{
    Serial.println(F("I2C clock speed is not set to 400kHz. (ERROR 8)"));
    error = 1;
  }
  
  if(error == 0){
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Transmitter setup"));
    Serial.println(F("==================================================="));
    delay(1000);
    Serial.print(F("Checking for valid receiver signals."));
    //Wait 10 seconds until all receiver inputs are valid
    wait_for_receiver();
    Serial.println(F(""));
  }
  //Quit the program in case of an error
  if(error == 0){
    delay(2000);
    Serial.println(F("Place all sticks and subtrims in the center position within 3 seconds."));
    for(int i = 3;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
    }
    Serial.println("Wating signal from controller. ");
    while(false == checkRFSignal(true))
    { 
      Serial.print(".");
      delay(500);
    }
    Serial.println(" ");
    Serial.println("Received signal.");
    //Store the central stick positions
    center_channel_1 = double_yaw;
    center_channel_2 = double_lift;
    center_channel_3 = double_roll;
    center_channel_4 = double_pitch;
    Serial.println(F(""));
    Serial.println(F("Center positions stored."));
    Serial.print(F("double_yaw = "));
    Serial.println(double_yaw);
    Serial.print(F("double_lift = "));
    Serial.println(double_lift);
    Serial.print(F("double_roll = "));
    Serial.println(double_roll);
    Serial.print(F("double_pitch = "));
    Serial.println(double_pitch);
    Serial.println(F(""));
  }
  if(error == 0){  
    
    //////////////////////////////////////////////////////////////////////////////////
    //Check for throttle movement
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the throttle stick to full throttle and back to center"));
    check_receiver_inputs(1);
    if(channel_3_assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();

    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Place throttle in max position and hold for 3 seconds."));
    check_receiver_inputs(1);
    for(int i = 3;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
      checkRFSignal(true);
      if(high_channel_2 < receiver_input_channel_2)
        high_channel_2 = receiver_input_channel_2;
      Serial.print("high_channel_2 : ");
      Serial.println(high_channel_2);
      Serial.print("receiver_input_channel_2 : ");
      Serial.println(receiver_input_channel_2);
    }
    Serial.println(F("Place all to center to continue and hold."));
    wait_sticks_zero();
    low_channel_2 = receiver_input_channel_2;
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Place throttle in min position and hold for 3 seconds."));
    check_receiver_inputs(1);
    for(int i = 3;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");    
      checkRFSignal(true);
      if(low_channel_2 > receiver_input_channel_2)
        low_channel_2 = receiver_input_channel_2;
      Serial.print("low_channel_2 : ");
      Serial.println(low_channel_2);
      Serial.print("receiver_input_channel_2 : ");
      Serial.println(receiver_input_channel_2);
    }
    Serial.println(F("Place all to center to continue."));
    wait_sticks_zero();

    //////////////////////////////////////////////////////////////////////////////////
    //Check for yaw movement
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the yaw stick to simulate right wing up and back to center"));
    check_receiver_inputs(2);
    if(channel_1_assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();

    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Place roll in max position within 3 seconds and hold."));
    check_receiver_inputs(2);
    for(int i = 3;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");    
      checkRFSignal(true);
      if(high_channel_1 < receiver_input_channel_1)
        high_channel_1 = receiver_input_channel_1;
      Serial.print("high_channel_1 : ");
      Serial.println(high_channel_1);
      Serial.print("receiver_input_channel_1 : ");
      Serial.println(receiver_input_channel_1);
    }
    Serial.println(F("Place all to center to continue."));
    wait_sticks_zero();

    low_channel_1 = receiver_input_channel_1;
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Place yaw in min position within 3 seconds and hold."));
    check_receiver_inputs(2);
    for(int i = 3;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
      checkRFSignal(true);
      if(low_channel_1> receiver_input_channel_1)
        low_channel_1 = receiver_input_channel_1;
      Serial.print("low_channel_1 : ");
      Serial.println(low_channel_1);
      Serial.print("receiver_input_channel_1 : ");
      Serial.println(receiver_input_channel_1);
    }
    Serial.println(F("Place all to center to continue."));
    wait_sticks_zero();

    //////////////////////////////////////////////////////////////////////////////////
    //Check for roll movement
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the pitch stick to simulate nose right and back to center"));
    check_receiver_inputs(3);
    if(channel_2_assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
    
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Place roll in max position within 3 seconds and hold."));
    check_receiver_inputs(3);
    for(int i = 3;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
      checkRFSignal(true);
      
      if(high_channel_3 < receiver_input_channel_3)
        high_channel_3 = receiver_input_channel_3;
      Serial.print("high_channel_3 : ");
      Serial.println(high_channel_3);
      Serial.print("receiver_input_channel_3 : ");
      Serial.println(receiver_input_channel_3);
    }
    Serial.println(F("Place all to center to continue."));
    wait_sticks_zero();    

    low_channel_3 = receiver_input_channel_3;
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Place roll in min position within 3 seconds and hold."));
    check_receiver_inputs(3);
    for(int i = 3;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");
      checkRFSignal(true);
      if(low_channel_3 > receiver_input_channel_3)
        low_channel_3 = receiver_input_channel_3;
      Serial.print("low_channel_3 : ");
      Serial.println(low_channel_3);
      Serial.print("receiver_input_channel_3 : ");
      Serial.println(receiver_input_channel_3);
    }
    Serial.println(F("Place all to center to continue."));
    wait_sticks_zero();

    //////////////////////////////////////////////////////////////////////////////////
    //Check for pitch movement
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Move the roll stick to simulate nose up and back to center"));
    check_receiver_inputs(4);
    if(channel_4_assign & 0b10000000)Serial.println(F("Channel inverted = yes"));
    else Serial.println(F("Channel inverted = no"));
    wait_sticks_zero();
  
    Serial.println(F("Place pitch in max position within 3 seconds and hold."));
    check_receiver_inputs(4);
    for(int i = 3;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");   
      checkRFSignal(true);
      if(high_channel_4 < receiver_input_channel_4)
        high_channel_4 = receiver_input_channel_4;
      Serial.print("high_channel_4 : ");
      Serial.println(high_channel_4);
      Serial.print("receiver_input_channel_4 : ");
      Serial.println(receiver_input_channel_4);
    }
    Serial.println(F("Place all to center to continue."));
    wait_sticks_zero();

    low_channel_4 = receiver_input_channel_4;
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("Place pitch in min position within 3 secondsand hold. "));
    check_receiver_inputs(4);
    for(int i = 3;i > 0;i--){
      delay(1000);
      Serial.print(i);
      Serial.print(" ");    
      checkRFSignal(true);
      if(low_channel_4 > receiver_input_channel_4)
        low_channel_4 = receiver_input_channel_4;
      Serial.print("low_channel_4 : ");
      Serial.println(low_channel_4);
      Serial.print("receiver_input_channel_4 : ");
      Serial.println(receiver_input_channel_4); 
    }
    Serial.println(F("Place all to center to continue."));
    wait_sticks_zero();
    
    Serial.println(F(""));
    Serial.println(F(""));
    Serial.println(F("High, low and center values found during setup"));
    Serial.print(F("yaw:"));
    Serial.print(low_channel_1);
    Serial.print(F(" - "));
    Serial.print(center_channel_1);
    Serial.print(F(" - "));
    Serial.println(high_channel_1);
    Serial.print(F("Throttle:"));
    Serial.print(low_channel_2);
    Serial.print(F(" - "));
    Serial.print(center_channel_2);
    Serial.print(F(" - "));
    Serial.println(high_channel_2);
    Serial.print(F("Roll:"));
    Serial.print(low_channel_3);
    Serial.print(F(" - "));
    Serial.print(center_channel_3);
    Serial.print(F(" - "));
    Serial.println(high_channel_3);
    Serial.print(F("Pitch:"));
    Serial.print(low_channel_4);
    Serial.print(F(" - "));
    Serial.print(center_channel_4);
    Serial.print(F(" - "));
    Serial.println(high_channel_4);
    Serial.println(F("Move stick Pitch and back to center to continue"));
    check_to_continue();
  }
    
  if(error == 0){
    //What gyro is connected
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro search"));
    Serial.println(F("==================================================="));
    delay(2000);
    
    Serial.println(F("Searching for MPU-6050 on address 0x68/104"));
    delay(1000);
    if(search_gyro(0x68, 0x75) == 0x68){
      Serial.println(F("MPU-6050 found on address 0x68"));
      type = 1;
      gyro_address = 0x68;
    }
    
    if(type == 0){
      Serial.println(F("Searching for MPU-6050 on address 0x69/105"));
      delay(1000);
      if(search_gyro(0x69, 0x75) == 0x68){
        Serial.println(F("MPU-6050 found on address 0x69"));
        type = 1;
        gyro_address = 0x69;
      }
    }
    
    if(type == 0){
      Serial.println(F("Searching for L3G4200D on address 0x68/104"));
      delay(1000);
      if(search_gyro(0x68, 0x0F) == 0xD3){
        Serial.println(F("L3G4200D found on address 0x68"));
        type = 2;
        gyro_address = 0x68;
      }
    }
    
    if(type == 0){
      Serial.println(F("Searching for L3G4200D on address 0x69/105"));
      delay(1000);
      if(search_gyro(0x69, 0x0F) == 0xD3){
        Serial.println(F("L3G4200D found on address 0x69"));
        type = 2;
        gyro_address = 0x69;
      }
    }
    
    if(type == 0){
      Serial.println(F("Searching for L3GD20H on address 0x6A/106"));
      delay(1000);
      if(search_gyro(0x6A, 0x0F) == 0xD7){
        Serial.println(F("L3GD20H found on address 0x6A"));
        type = 3;
        gyro_address = 0x6A;
      }
    }
    
    if(type == 0){
     Serial.println(F("Searching for L3GD20H on address 0x6B/107"));
      delay(1000);
      if(search_gyro(0x6B, 0x0F) == 0xD7){
        Serial.println(F("L3GD20H found on address 0x6B"));
        type = 3;
        gyro_address = 0x6B;
      }
    }
    
    if(type == 0){
      Serial.println(F("No gyro device found!!! (ERROR 3)"));
      error = 1;
    }
    
    else{
      delay(3000);
      Serial.println(F(""));
      Serial.println(F("==================================================="));
      Serial.println(F("Gyro register settings"));
      Serial.println(F("==================================================="));
      start_gyro(); //Setup the gyro for further use
    }
  }
  
  //If the gyro is found we can setup the correct gyro axes.
  if(error == 0){
    delay(3000);
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro calibration"));
    Serial.println(F("==================================================="));
    Serial.println(F("Don't move the quadcopter!! Calibration starts in 3 seconds"));
    delay(3000);
    Serial.println(F("Calibrating the gyro, this will take +/- 8 seconds"));
    Serial.print(F("Please wait"));
    //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
    for (cal_int = 0; cal_int < 2000 ; cal_int ++){              //Take 2000 readings for calibration.
      if(cal_int % 100 == 0)Serial.print(F("."));                //Print dot to indicate calibration.
      gyro_signalen();                                           //Read the gyro output.
      gyro_roll_cal += gyro_roll;                                //Ad roll value to gyro_roll_cal.
      gyro_pitch_cal += gyro_pitch;                              //Ad pitch value to gyro_pitch_cal.
      gyro_yaw_cal += gyro_yaw;                                  //Ad yaw value to gyro_yaw_cal.
      delay(4);                                                  //Wait 3 milliseconds before the next loop.
    }
    //Now that we have 2000 measures, we need to devide by 2000 to get the average gyro offset.
    gyro_roll_cal /= 2000;                                       //Divide the roll total by 2000.
    gyro_pitch_cal /= 2000;                                      //Divide the pitch total by 2000.
    gyro_yaw_cal /= 2000;                                        //Divide the yaw total by 2000.
    
    //Show the calibration results
    Serial.println(F(""));
    Serial.print(F("Axis 1 offset="));
    Serial.println(gyro_roll_cal);
    Serial.print(F("Axis 2 offset="));
    Serial.println(gyro_pitch_cal);
    Serial.print(F("Axis 3 offset="));
    Serial.println(gyro_yaw_cal);
    Serial.println(F(""));
    
    Serial.println(F("==================================================="));
    Serial.println(F("Gyro axes configuration"));
    Serial.println(F("==================================================="));
    
    //Detect the left wing up movement
    Serial.println(F("Lift the left side of the quadcopter to a 45 degree angle within 10 seconds"));
    //Check axis movement
    check_gyro_axes(1);
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(roll_axis & 0b00000011);
      if(roll_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();

      //Detect the nose up movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Lift the nose of the quadcopter to a 45 degree angle within 10 seconds"));
      //Check axis movement
      check_gyro_axes(2);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(pitch_axis & 0b00000011);
      if(pitch_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
      
      //Detect the nose right movement
      Serial.println(F(""));
      Serial.println(F(""));
      Serial.println(F("Rotate the nose of the quadcopter 45 degree to the right within 10 seconds"));
      //Check axis movement
      check_gyro_axes(3);
    }
    if(error == 0){
      Serial.println(F("OK!"));
      Serial.print(F("Angle detection = "));
      Serial.println(yaw_axis & 0b00000011);
      if(yaw_axis & 0b10000000)Serial.println(F("Axis inverted = yes"));
      else Serial.println(F("Axis inverted = no"));
      Serial.println(F("Put the quadcopter back in its original position"));
      Serial.println(F("Move stick 'nose up' and back to center to continue"));
      check_to_continue();
    }
  }

  Serial.println(F(""));
  
  if(error == 0){
    Serial.println(F("==================================================="));
    Serial.println(F("Final setup check"));
    Serial.println(F("==================================================="));
    delay(1000);
    if(receiver_check_byte == 0b00001111){
      Serial.println(F("Receiver channels ok"));
    }
    else{
      Serial.println(F("Receiver channel verification failed!!! (ERROR 6)"));
      error = 1;
    }
    delay(1000);
    if(gyro_check_byte == 0b00000111){
      Serial.println(F("Gyro axes ok"));
    }
    else{
      Serial.println(F("Gyro exes verification failed!!! (ERROR 7)"));
      error = 1;
    }
  }     
  
  if(error == 0){
    //If all is good, store the information in the EEPROM
    Serial.println(F(""));
    Serial.println(F("==================================================="));
    Serial.println(F("Storing EEPROM information"));
    Serial.println(F("==================================================="));
    Serial.println(F("Writing EEPROM"));
    delay(1000);
    Serial.println(F("Done!"));
    EEPROM.write(0, center_channel_1 & 0b11111111);
    EEPROM.write(1, center_channel_1 >> 8);
    EEPROM.write(2, center_channel_2 & 0b11111111);
    EEPROM.write(3, center_channel_2 >> 8);
    EEPROM.write(4, center_channel_3 & 0b11111111);
    EEPROM.write(5, center_channel_3 >> 8);
    EEPROM.write(6, center_channel_4 & 0b11111111);
    EEPROM.write(7, center_channel_4 >> 8);
    EEPROM.write(8, high_channel_1 & 0b11111111);
    EEPROM.write(9, high_channel_1 >> 8);
    EEPROM.write(10, high_channel_2 & 0b11111111);
    EEPROM.write(11, high_channel_2 >> 8);
    EEPROM.write(12, high_channel_3 & 0b11111111);
    EEPROM.write(13, high_channel_3 >> 8);
    EEPROM.write(14, high_channel_4 & 0b11111111);
    EEPROM.write(15, high_channel_4 >> 8);
    EEPROM.write(16, low_channel_1 & 0b11111111);
    EEPROM.write(17, low_channel_1 >> 8);
    EEPROM.write(18, low_channel_2 & 0b11111111);
    EEPROM.write(19, low_channel_2 >> 8);
    EEPROM.write(20, low_channel_3 & 0b11111111);
    EEPROM.write(21, low_channel_3 >> 8);
    EEPROM.write(22, low_channel_4 & 0b11111111);
    EEPROM.write(23, low_channel_4 >> 8);
    EEPROM.write(24, channel_1_assign);
    EEPROM.write(25, channel_2_assign);
    EEPROM.write(26, channel_3_assign);
    EEPROM.write(27, channel_4_assign);
    EEPROM.write(28, roll_axis);
    EEPROM.write(29, pitch_axis);
    EEPROM.write(30, yaw_axis);
    EEPROM.write(31, type);
    EEPROM.write(32, gyro_address);
    //Write the EEPROM signature
    EEPROM.write(33, 'J'); 
    EEPROM.write(34, 'M');
    EEPROM.write(35, 'B');
        
    
    //To make sure evrything is ok, verify the EEPROM data.
    Serial.println(F("Verify EEPROM data"));
    delay(1000);
    if(center_channel_1 != ((EEPROM.read(1) << 8) | EEPROM.read(0)))error = 1;
    if(center_channel_2 != ((EEPROM.read(3) << 8) | EEPROM.read(2)))error = 1;
    if(center_channel_3 != ((EEPROM.read(5) << 8) | EEPROM.read(4)))error = 1;
    if(center_channel_4 != ((EEPROM.read(7) << 8) | EEPROM.read(6)))error = 1;
    
    if(high_channel_1 != ((EEPROM.read(9) << 8) | EEPROM.read(8)))error = 1;
    if(high_channel_2 != ((EEPROM.read(11) << 8) | EEPROM.read(10)))error = 1;
    if(high_channel_3 != ((EEPROM.read(13) << 8) | EEPROM.read(12)))error = 1;
    if(high_channel_4 != ((EEPROM.read(15) << 8) | EEPROM.read(14)))error = 1;
    
    if(low_channel_1 != ((EEPROM.read(17) << 8) | EEPROM.read(16)))error = 1;
    if(low_channel_2 != ((EEPROM.read(19) << 8) | EEPROM.read(18)))error = 1;
    if(low_channel_3 != ((EEPROM.read(21) << 8) | EEPROM.read(20)))error = 1;
    if(low_channel_4 != ((EEPROM.read(23) << 8) | EEPROM.read(22)))error = 1;
    
    if(channel_1_assign != EEPROM.read(24))error = 1;
    if(channel_2_assign != EEPROM.read(25))error = 1;
    if(channel_3_assign != EEPROM.read(26))error = 1;
    if(channel_4_assign != EEPROM.read(27))error = 1;
    
    if(roll_axis != EEPROM.read(28))error = 1;
    if(pitch_axis != EEPROM.read(29))error = 1;
    if(yaw_axis != EEPROM.read(30))error = 1;
    if(type != EEPROM.read(31))error = 1;
    if(gyro_address != EEPROM.read(32))error = 1;
    
    if('J' != EEPROM.read(33))error = 1;
    if('M' != EEPROM.read(34))error = 1;
    if('B' != EEPROM.read(35))error = 1;
  
    if(error == 1)Serial.println(F("EEPROM verification failed!!! (ERROR 5)"));
    else Serial.println(F("Verification done"));
  }
  
  
  if(error == 0){
    Serial.println(F("Setup is finished."));
    Serial.println(F("You can now calibrate the esc's and upload the YMFC-AL code."));
  }
  while(1);
}

//Search for the gyro and check the Who_am_I register
byte search_gyro(int gyro_address, int who_am_i){
  Wire.beginTransmission(gyro_address);
  Wire.write(who_am_i);
  Wire.endTransmission();
  Wire.requestFrom(gyro_address, 1);
  timer = millis() + 100;
  while(Wire.available() < 1 && timer > millis());
  lowByte = Wire.read();
  address = gyro_address;
  return lowByte;
}

void start_gyro(){
  //Setup the L3G4200D or L3GD20H
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);                             //Start communication with the gyro with the address found during search
    Wire.write(0x20);                                            //We want to write to register 1 (20 hex)
    Wire.write(0x0F);                                            //Set the register bits as 00001111 (Turn on the gyro and enable all axis)
    Wire.endTransmission();                                      //End the transmission with the gyro

    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x20);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 6 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x20 is set to:"));
    Serial.println(Wire.read(),BIN);

    Wire.beginTransmission(address);                             //Start communication with the gyro  with the address found during search
    Wire.write(0x23);                                            //We want to write to register 4 (23 hex)
    Wire.write(0x90);                                            //Set the register bits as 10010000 (Block Data Update active & 500dps full scale)
    Wire.endTransmission();                                      //End the transmission with the gyro
    
    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x23);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 6 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x23 is set to:"));
    Serial.println(Wire.read(),BIN);

  }
  //Setup the MPU-6050
  if(type == 1){
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //PWR_MGMT_1 register
    Wire.write(0x00);                                            //Set to zero to turn on the gyro
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x6B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x6B is set to:"));
    Serial.println(Wire.read(),BIN);
    
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x1B);                                            //GYRO_CONFIG register
    Wire.write(0x08);                                            //Set the register bits as 00001000 (500dps full scale)
    Wire.endTransmission();                                      //End the transmission
    
    Wire.beginTransmission(address);                             //Start communication with the gyro (adress 1101001)
    Wire.write(0x1B);                                            //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 1);                                //Request 1 bytes from the gyro
    while(Wire.available() < 1);                                 //Wait until the 1 byte is received
    Serial.print(F("Register 0x1B is set to:"));
    Serial.println(Wire.read(),BIN);

  }
}

void gyro_signalen(){
  if(type == 2 || type == 3){
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(168);                                             //Start reading @ register 28h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address, 6);                                //Request 6 bytes from the gyro
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    gyro_roll = ((highByte<<8)|lowByte);                         //Multiply highByte by 256 (shift left by 8) and ad lowByte
    if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    gyro_pitch = ((highByte<<8)|lowByte);                        //Multiply highByte by 256 (shift left by 8) and ad lowByte
    if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
    lowByte = Wire.read();                                       //First received byte is the low part of the angular data
    highByte = Wire.read();                                      //Second received byte is the high part of the angular data
    gyro_yaw = ((highByte<<8)|lowByte);                          //Multiply highByte by 256 (shift left by 8) and ad lowByte
    if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
  }
  if(type == 1){
    Wire.beginTransmission(address);                             //Start communication with the gyro
    Wire.write(0x43);                                            //Start reading @ register 43h and auto increment with every read
    Wire.endTransmission();                                      //End the transmission
    Wire.requestFrom(address,6);                                 //Request 6 bytes from the gyro
    while(Wire.available() < 6);                                 //Wait until the 6 bytes are received
    gyro_roll=Wire.read()<<8|Wire.read();                        //Read high and low part of the angular data
    if(cal_int == 2000)gyro_roll -= gyro_roll_cal;               //Only compensate after the calibration
    gyro_pitch=Wire.read()<<8|Wire.read();                       //Read high and low part of the angular data
    if(cal_int == 2000)gyro_pitch -= gyro_pitch_cal;             //Only compensate after the calibration
    gyro_yaw=Wire.read()<<8|Wire.read();                         //Read high and low part of the angular data
    if(cal_int == 2000)gyro_yaw -= gyro_yaw_cal;                 //Only compensate after the calibration
  }
}

//Check if a receiver input value is changing within 30 seconds
void check_receiver_inputs(byte movement){
  byte trigger = 0;
  int pulse_length;
  timer = millis() + 30000;
  while(timer > millis() && trigger == 0){
    delay(250);
    checkRFSignal(false);
    if(receiver_input_channel_1 > 1750 || receiver_input_channel_1 < 1250){
      trigger = 1;
      receiver_check_byte |= 0b00000001;
      pulse_length = receiver_input_channel_1;
    }
    if(receiver_input_channel_2 > 1750 || receiver_input_channel_2 < 1250){
      trigger = 2;
      receiver_check_byte |= 0b00000010;
      pulse_length = receiver_input_channel_2;
    }
    if(receiver_input_channel_3 > 1750 || receiver_input_channel_3 < 1250){
      trigger = 3;
      receiver_check_byte |= 0b00000100;
      pulse_length = receiver_input_channel_3;
    }
    if(receiver_input_channel_4 > 1750 || receiver_input_channel_4 < 1250){
      trigger = 4;
      receiver_check_byte |= 0b00001000;
      pulse_length = receiver_input_channel_4;
    } 
  }
  if(trigger == 0){
    error = 1;
    Serial.println(F("No stick movement detected in the last 30 seconds!!! (ERROR 2)"));
  }
  //Assign the stick to the function.
  else{
    if(movement == 1){
      channel_3_assign = trigger;
      if(pulse_length < 1250)channel_3_assign += 0b10000000;
    }
    if(movement == 2){
      channel_1_assign = trigger;
      if(pulse_length < 1250)channel_1_assign += 0b10000000;
    }
    if(movement == 3){
      channel_2_assign = trigger;
      if(pulse_length < 1250)channel_2_assign += 0b10000000;
    }
    if(movement == 4){
      channel_4_assign = trigger;
      if(pulse_length < 1250)channel_4_assign += 0b10000000;
    }
  }
}

void check_to_continue(){
  byte continue_byte = 0;
  while(continue_byte == 0){
    check_receiver_inputs(3);
    if(channel_2_assign == 0b00000001 && receiver_input_channel_1 > center_channel_1 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000001 && receiver_input_channel_1 < center_channel_1 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000010 && receiver_input_channel_2 > center_channel_2 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000010 && receiver_input_channel_2 < center_channel_2 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000011 && receiver_input_channel_3 > center_channel_3 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000011 && receiver_input_channel_3 < center_channel_3 - 150)continue_byte = 1;
    if(channel_2_assign == 0b00000100 && receiver_input_channel_4 > center_channel_4 + 150)continue_byte = 1;
    if(channel_2_assign == 0b10000100 && receiver_input_channel_4 < center_channel_4 - 150)continue_byte = 1;
    delay(100);
  }
  wait_sticks_zero();
}

//Check if the transmitter sticks are in the neutral position
void wait_sticks_zero(){
  byte zero = 0;
  while(zero < 15){
    checkRFSignal(true);
    if(receiver_input_channel_1 < center_channel_1 + 20 && receiver_input_channel_1 > center_channel_1 - 20)zero |= 0b00000001;
    if(receiver_input_channel_2 < center_channel_2 + 20 && receiver_input_channel_2 > center_channel_2 - 20)zero |= 0b00000010;
    if(receiver_input_channel_3 < center_channel_3 + 20 && receiver_input_channel_3 > center_channel_3 - 20)zero |= 0b00000100;
    if(receiver_input_channel_4 < center_channel_4 + 20 && receiver_input_channel_4 > center_channel_4 - 20)zero |= 0b00001000;
    Serial.print("zero: ");
    Serial.println(zero);
    delay(1000);
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
  byte bReceived = 0;
  timer = millis() + 10000;
  while(timer > millis()){
  
    if (true == checkRFSignal(true))
    {                     
      bReceived = 1;
      break;
    }
    delay(500);
    Serial.print(F("."));
  }
  if(bReceived == 0){
    error = 1;
    Serial.println(F("."));
    Serial.println(F("No valid receiver signals found!!! (ERROR 1)"));
  }
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
//Check if the angular position of a gyro axis is changing within 10 seconds
void check_gyro_axes(byte movement){
  byte trigger_axis = 0;
  float gyro_angle_roll, gyro_angle_pitch, gyro_angle_yaw;
  //Reset all axes
  gyro_angle_roll = 0;
  gyro_angle_pitch = 0;
  gyro_angle_yaw = 0;
  gyro_signalen();
  timer = millis() + 10000;    
  while(timer > millis() && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_signalen();
    if(type == 2 || type == 3){
      gyro_angle_roll += gyro_roll * 0.00007;              //0.00007 = 17.5 (md/s) / 250(Hz)
      gyro_angle_pitch += gyro_pitch * 0.00007;
      gyro_angle_yaw += gyro_yaw * 0.00007;
    }
    if(type == 1){
      gyro_angle_roll += gyro_roll * 0.0000611;          // 0.0000611 = 1 / 65.5 (LSB degr/s) / 250(Hz)
      gyro_angle_pitch += gyro_pitch * 0.0000611;
      gyro_angle_yaw += gyro_yaw * 0.0000611;
    }
    
    delayMicroseconds(3700); //Loop is running @ 250Hz. +/-300us is used for communication with the gyro
  }
  //Assign the moved axis to the orresponding function (pitch, roll, yaw)
  if((gyro_angle_roll < -30 || gyro_angle_roll > 30) && gyro_angle_pitch > -30 && gyro_angle_pitch < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000001;
    if(gyro_angle_roll < 0)trigger_axis = 0b10000001;
    else trigger_axis = 0b00000001;
  }
  if((gyro_angle_pitch < -30 || gyro_angle_pitch > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_yaw > -30 && gyro_angle_yaw < 30){
    gyro_check_byte |= 0b00000010;
    if(gyro_angle_pitch < 0)trigger_axis = 0b10000010;
    else trigger_axis = 0b00000010;
  }
  if((gyro_angle_yaw < -30 || gyro_angle_yaw > 30) && gyro_angle_roll > -30 && gyro_angle_roll < 30 && gyro_angle_pitch > -30 && gyro_angle_pitch < 30){
    gyro_check_byte |= 0b00000100;
    if(gyro_angle_yaw < 0)trigger_axis = 0b10000011;
    else trigger_axis = 0b00000011;
  }
  
  if(trigger_axis == 0){
    error = 1;
    Serial.println(F("No angular motion is detected in the last 10 seconds!!! (ERROR 4)"));
  }
  else
  if(movement == 1)roll_axis = trigger_axis;
  if(movement == 2)pitch_axis = trigger_axis;
  if(movement == 3)yaw_axis = trigger_axis;
  
}

//Intro subroutine
void intro(){
  Serial.println(F("==================================================="));
  delay(500);
  Serial.println(F(""));
  Serial.println(F("Your"));
  delay(250);
  Serial.println(F("  iSEB"));
  delay(250);
  Serial.println(F("    Drone"));
  delay(250);
  Serial.println(F("       Setup Program"));
  Serial.println(F(""));
  Serial.println(F("==================================================="));
  delay(500);
  Serial.println(F("Have fun!"));
}
