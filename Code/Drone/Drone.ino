#include "pitches.h"
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <SD.h>
#include "RF24.h"
#include <Servo.h>

#define GPS_DEBUG 0
#define BME_DEBUG 0
#define MPU6050_DEBUG 1
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

// Motor
#define MIN_PULSE_LENGTH 1000 // Minimum pulse length in µs
#define MAX_PULSE_LENGTH 5000 // Maximum pulse length in µs

uint8_t state = 0 ;

// ---------------------------------------------------------------------------

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
uint8_t address[][6] = { "AAAAAA", "555555" };
uint8_t u8Yaw  = 0; /* left X */
uint8_t u8Lift  = 0; /* left Y */
uint8_t u8Roll = 0; /* Right X */
uint8_t u8Pitch = 0; /* RIght Y */
bool bRightButton = false;
bool bLeftButton = false;

/*GPS*/
TinyGPS gps;
SoftwareSerial mySerial(4,7);
long gpstimer = 0;
bool newData = false;
unsigned long chars;
unsigned short sentences, failed;


/* SD Card */
File myFile;
byte bSDCard = 0;

/* BME */
#define BME280_ADDRESS 0x76
unsigned long int hum_raw,temp_raw,pres_raw;
signed long int t_fine;

uint16_t dig_T1;
int16_t dig_T2;
int16_t dig_T3;
uint16_t dig_P1;
int16_t dig_P2;
int16_t dig_P3;
int16_t dig_P4;
int16_t dig_P5;
int16_t dig_P6;
int16_t dig_P7;
int16_t dig_P8;
int16_t dig_P9;
int8_t  dig_H1;
int16_t dig_H2;
int8_t  dig_H3;
int16_t dig_H4;
int16_t dig_H5;
int8_t  dig_H6;
long BME280timer = 0;

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//PID gain and limit settings
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
float pid_p_gain_roll = 1.3;               //Gain setting for the roll P-controller
float pid_i_gain_roll = 0.04;              //Gain setting for the roll I-controller
float pid_d_gain_roll = 18.0;              //Gain setting for the roll D-controller
int pid_max_roll = 400;                    //Maximum output of the PID-controller (+/-)

float pid_p_gain_pitch = pid_p_gain_roll;  //Gain setting for the pitch P-controller.
float pid_i_gain_pitch = pid_i_gain_roll;  //Gain setting for the pitch I-controller.
float pid_d_gain_pitch = pid_d_gain_roll;  //Gain setting for the pitch D-controller.
int pid_max_pitch = pid_max_roll;          //Maximum output of the PID-controller (+/-)

float pid_p_gain_yaw = 4.0;                //Gain setting for the pitch P-controller. //4.0
float pid_i_gain_yaw = 0.02;               //Gain setting for the pitch I-controller. //0.02
float pid_d_gain_yaw = 0.0;                //Gain setting for the pitch D-controller.
int pid_max_yaw = 400;                     //Maximum output of the PID-controller (+/-)

boolean auto_level = true;                 //Auto level on (true) or off (false)

//notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

/* MPU6050 */
#define MPU6050_ADDR                  0x68
#define MPU6050_CONFIG_REGISTER       0x1a
#define MPU6050_GYRO_CONFIG_REGISTER  0x1b
#define MPU6050_ACCEL_CONFIG_REGISTER 0x1c
#define MPU6050_PWR_MGMT_1_REGISTER   0x6b

#define MPU6050_GYRO_OUT_REGISTER     0x43
#define MPU6050_ACCEL_OUT_REGISTER    0x3B

#define MPU6050_CAL_NUM               1000

uint16_t cal_int = 0;
long acc_x, acc_y, acc_z, acc_total_vector;
unsigned long timer_channel_1, timer_channel_2, timer_channel_3, timer_channel_4, esc_timer, esc_loop_timer;
unsigned long timer_1, timer_2, timer_3, timer_4, current_time;
unsigned long loop_timer;
double gyro_pitch, gyro_roll, gyro_yaw;
double gyro_axis_cal[6];
float pid_error_temp;
float pid_i_mem_roll, pid_roll_setpoint, gyro_roll_input, pid_output_roll, pid_last_roll_d_error;
float pid_i_mem_pitch, pid_pitch_setpoint, gyro_pitch_input, pid_output_pitch, pid_last_pitch_d_error;
float pid_i_mem_yaw, pid_yaw_setpoint, gyro_yaw_input, pid_output_yaw, pid_last_yaw_d_error;
float angle_roll_acc, angle_pitch_acc, angle_pitch, angle_roll;
boolean gyro_angles_set;
int temperature;
int acc_axis[4], gyro_axis[4];
float roll_level_adjust, pitch_level_adjust;
long MPU6050timer = 0;
long MPU6050Printtimer = 0;

void setup() {
  
  /* Serial */
  Serial.begin(115200);
  Serial.println("Hello World!");

  Wire.begin();   //Start the I2C as master.
  TWBR = 12;   //Set the I2C clock speed to 400kHz.

  mot1.attach(pwmMotor1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot2.attach(pwmMotor2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot3.attach(pwmMotor3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot4.attach(pwmMotor4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  /* set to minimum to stop the motor and also avoid beep sound */
  mot1.writeMicroseconds(MIN_PULSE_LENGTH); 
  mot2.writeMicroseconds(MIN_PULSE_LENGTH);
  mot3.writeMicroseconds(MIN_PULSE_LENGTH);
  mot4.writeMicroseconds(MIN_PULSE_LENGTH);

  delay(250);   
  BME_Init();
  delay(250);  
  MPU6050_Init(); //Set the specific gyro registers. 
  delay(250);  

  //Let's take multiple gyro data samples so we can determine the average gyro offset (calibration).
  Serial.println("Calibrating MPU6050 please dont move the device!");
  for (cal_int = 0; cal_int < MPU6050_CAL_NUM ; cal_int ++){                           //Take 2000 readings for calibration.
    MPU6050_readData();                                                        //Read the gyro output.
    gyro_axis_cal[1] += gyro_axis[1];                                       //Ad roll value to gyro_roll_cal.
    gyro_axis_cal[2] += gyro_axis[2];                                       //Ad pitch value to gyro_pitch_cal.
    gyro_axis_cal[3] += gyro_axis[3];                                       //Ad yaw value to gyro_yaw_cal.
     if(cal_int % 125 == 0)Serial.print(".");                             //Print a dot on the LCD every 125 readings
    delay(4);                                                               //Wait 1000us.
  }

  //Now that we have MPU6050_CAL_NUM measures, we need to devide by 2000 to get the average gyro offset.
  gyro_axis_cal[1] /= MPU6050_CAL_NUM;                                                 //Divide the roll total by 2000.
  gyro_axis_cal[2] /= MPU6050_CAL_NUM;                                                 //Divide the pitch total by 2000.
  gyro_axis_cal[3] /= MPU6050_CAL_NUM;                                                 //Divide the yaw total by 2000.

  mySerial.begin(9600);
 
  
  /* should read the battery voltage here */
  //battery_voltage = (analogRead(0) + 65) * 1.2317;
  //delay(500);

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
    radio.openWritingPipe(address[1]);  // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1,address[0]);  // using pipe 1
    memset(radioMessage,0x00,6);
    radio.writeAckPayload(1, &radioMessage, sizeof(radioMessage));
    radio.startListening();  // put radio in RX mode
  }

  delay(3000); /* delay 3 second */
}

void loop() {
  BME_task();
  GPS_task();
  radio_task();
  switch(state)
  {
    case 0:
    {
      if((1200 > double_lift)&& (1200 > double_yaw))
      {
        state = 1; 
      }
      state = 2; //* for testing purposer */
      break;
    }
    case 1:
    {
      if(( 1050 < double_lift ) && ( 1200 > double_lift))
      {
        state = 2;
        angle_pitch = angle_pitch_acc;                                          //Set the gyro pitch angle equal to the accelerometer pitch angle when the quadcopter is started.
        angle_roll = angle_roll_acc;                                            //Set the gyro roll angle equal to the accelerometer roll angle when the quadcopter is started.

        //Reset the PID controllers for a bumpless start.
        pid_i_mem_roll = 0;
        pid_last_roll_d_error = 0;
        pid_i_mem_pitch = 0;
        pid_last_pitch_d_error = 0;
        pid_i_mem_yaw = 0;
        pid_last_yaw_d_error = 0;
      }
      break;
    }
    case 2:
    {
      if(millis() - MPU6050timer > 4)/* loop every 4 ms */ 
      {
        /* get data from sensor */
        MPU6050_readData();
        // To average the value againt changes 
        // 2 btye 65535 range ± 500 °/s so we will got 65.5 = 1 °/s 
        //65.5 = 1 deg/sec (check the datasheet of the MPU-6050 for more information).
        gyro_roll_input = (gyro_roll_input * 0.7) + ((gyro_roll / 65.5) * 0.3);   //Gyro pid input is deg/sec.
        gyro_pitch_input = (gyro_pitch_input * 0.7) + ((gyro_pitch / 65.5) * 0.3);//Gyro pid input is deg/sec.
        gyro_yaw_input = (gyro_yaw_input * 0.7) + ((gyro_yaw / 65.5) * 0.3);      //Gyro pid input is deg/sec.

        // Calculate the traveled pitch and roll angle
        // we have degree per second ( °/s  )
        // Task call every 4ms = 250Hz 
        // 2 btye 65535 range ± 500 °/s so we will got 65.5 = 1 °/s 
        // data / 250 / 65.5  = traveled angle  1/(250*65.5) = 0.0000611
        angle_pitch += gyro_pitch * 0.0000611;                                    //Calculate the traveled pitch angle and add this to the angle_pitch variable.
        angle_roll += gyro_roll * 0.0000611;                                      //Calculate the traveled roll angle and add this to the angle_roll variable.

        // add in factor when the device is not flat which gyro yaw is no 90 ( sin 90 = 1 )
        // convert data from degree to radian  =
        // radian = degree * PI / 180 
        //0.000001066 = 0.0000611 * (3.142(PI) / 180degr) 
        //The Arduino sin function is in radians 
        angle_pitch -= angle_roll * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the roll angle to the pitch angel.
        angle_roll += angle_pitch * sin(gyro_yaw * 0.000001066);                  //If the IMU has yawed transfer the pitch angle to the roll angel.

        //Accelerometer angle calculations
        acc_total_vector = sqrt((acc_x*acc_x)+(acc_y*acc_y)+(acc_z*acc_z));       //Calculate the total accelerometer vector.
        //57.296 = 1 / (3.142 / 180) The Arduino asin function is in radians
        if(abs(acc_y) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
          angle_pitch_acc = asin((float)acc_y/acc_total_vector)* 57.296;          //Calculate the pitch angle.
        }
        if(abs(acc_x) < acc_total_vector){                                        //Prevent the asin function to produce a NaN
          angle_roll_acc = asin((float)acc_x/acc_total_vector)* -57.296;          //Calculate the roll angle.
        }
        
        //Place the MPU-6050 spirit level and note the values in the following two lines for calibration.
        angle_pitch_acc -= 0.0;                                                   //Accelerometer calibration value for pitch.
        angle_roll_acc -= 0.0;                                                    //Accelerometer calibration value for roll.
        
        /* to compenstate the gyro drift */
        angle_pitch = angle_pitch * 0.9996 + angle_pitch_acc * 0.0004;            //Correct the drift of the gyro pitch angle with the accelerometer pitch angle.
        angle_roll = angle_roll * 0.9996 + angle_roll_acc * 0.0004;               //Correct the drift of the gyro roll angle with the accelerometer roll angle.

        pitch_level_adjust = angle_pitch * 15;                                    //Calculate the pitch angle correction
        roll_level_adjust = angle_roll * 15;                                      //Calculate the roll angle correction

        //The PID set point in degrees per second is determined by the roll receiver input.
        //In the case of deviding by 3 the max roll rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
        pid_roll_setpoint = 0;
        //We need a little dead band of 16us for better results.
        if(double_roll > 1508)pid_roll_setpoint = double_roll - 1508;
        else if(double_roll < 1492)pid_roll_setpoint = double_roll - 1492;

        pid_roll_setpoint -= roll_level_adjust;                                   //Subtract the angle correction from the standardized receiver roll input value.
        pid_roll_setpoint /= 3.0;                                                 //Divide the setpoint for the PID roll controller by 3 to get angles in degrees.


        //The PID set point in degrees per second is determined by the pitch receiver input.
        //In the case of deviding by 3 the max pitch rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
        pid_pitch_setpoint = 0;
        //We need a little dead band of 16us for better results.
        if(double_pitch > 1508)pid_pitch_setpoint = double_pitch - 1508;
        else if(double_pitch < 1492)pid_pitch_setpoint = double_pitch - 1492;

        pid_pitch_setpoint -= pitch_level_adjust;                                  //Subtract the angle correction from the standardized receiver pitch input value.
        pid_pitch_setpoint /= 3.0;                                                 //Divide the setpoint for the PID pitch controller by 3 to get angles in degrees.

        //The PID set point in degrees per second is determined by the yaw receiver input.
        //In the case of deviding by 3 the max yaw rate is aprox 164 degrees per second ( (500-8)/3 = 164d/s ).
        pid_yaw_setpoint = 0;
        //We need a little dead band of 16us for better results.
        if(u8Lift > 1050){ //Do not yaw when turning off the motors.
          if(double_yaw > 1508)pid_yaw_setpoint = (double_yaw - 1508)/3.0;
          else if(double_yaw < 1492)pid_yaw_setpoint = (double_yaw - 1492)/3.0;
        }

        /* use pid to calculate motor outputt */
        calculate_pid();                                                            //PID inputs are known. So we can calculate the pid output.

        /* update motor frequency */
        mot1.writeMicroseconds(esc_1);
        mot2.writeMicroseconds(esc_2);
        mot3.writeMicroseconds(esc_4);
        mot4.writeMicroseconds(esc_4);

        MPU6050timer = millis();
      }
      break;
    }
    default:
    {
      state = 0;
      break;
    }
  }
}


//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void radio_task()
{
  uint8_t pipe;
  if (radio.available(&pipe)) {                     // is there a payload? get the pipe number that recieved it
    // load the payload for the first received transmission on pipe 0
    // radio.writeAckPayload(1,&radioMessage,6);
    uint8_t bytes = radio.getDynamicPayloadSize();  // get the size of the payload
    char received[6];
    radio.read(received, sizeof(received));  // get incoming ACK payload
    u8Yaw = received[0];
    u8Lift = received[1];
    u8Roll = received[2];
    u8Pitch = received[3];
    // Serial.print(F("Recieved on pipe "));
    // Serial.print(u8Yaw);     
    // Serial.print(" | ");
    // Serial.print(u8Lift);     
    // Serial.print(" | ");
    // Serial.print(u8Roll);     
    // Serial.print(" | ");
    // Serial.print(u8Pitch);  
    // Serial.print(" | ");
    // Serial.print(0x00 != (received[4]&&0x01)?"PRESS":"RELEASED");
    // Serial.print(" | ");
    // Serial.println(0x00 != (received[4]&&0x02)?"PRESS":"RELEASED");
    // Serial.print(F(" Sent: "));
    // Serial.println(radioMessage);    // print outgoing message
     
    double_yaw   =  ((uint32_t)u8Yaw   * 900 / 255 )+ 1050 ; 
    double_lift  =  ((uint32_t)u8Lift  * 900 / 255 )+ 1050 ; 
    double_roll  =  ((uint32_t)u8Roll  * 900 / 255 )+ 1050 ; 
    double_pitch =  ((uint32_t)u8Pitch * 900 / 255 )+ 1050 ; 

  }
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//Subroutine for calculating pid outputs
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//The PID controllers are explained in part 5 of the YMFC-3D video session:
//https://youtu.be/JBvnB0279-Q 
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void calculate_pid(){
  //pid_roll_setpoint ->set target
  //gryo_roll_input -> actual input from sensor
  //pid_i_gain_roll -> PID -> i component 
  //pid_i_mem_roll ->  Still unsure 
  // pid_p_gain_roll -> PID -> P component
  // pid_d_gain_roll -> PID -> D component 
  // p = proportional  y= mx + c (m)
  // i = intergration = area under graph 
  // d = dy / dx
  //Roll calculations
  pid_error_temp = gyro_roll_input - pid_roll_setpoint;
  pid_i_mem_roll += pid_i_gain_roll * pid_error_temp;
  if(pid_i_mem_roll > pid_max_roll)pid_i_mem_roll = pid_max_roll;
  else if(pid_i_mem_roll < pid_max_roll * -1)pid_i_mem_roll = pid_max_roll * -1;

  pid_output_roll = pid_p_gain_roll * pid_error_temp + pid_i_mem_roll + pid_d_gain_roll * (pid_error_temp - pid_last_roll_d_error);
  if(pid_output_roll > pid_max_roll)pid_output_roll = pid_max_roll;
  else if(pid_output_roll < pid_max_roll * -1)pid_output_roll = pid_max_roll * -1;

  pid_last_roll_d_error = pid_error_temp;

  //Pitch calculations
  pid_error_temp = gyro_pitch_input - pid_pitch_setpoint;
  pid_i_mem_pitch += pid_i_gain_pitch * pid_error_temp;
  if(pid_i_mem_pitch > pid_max_pitch)pid_i_mem_pitch = pid_max_pitch;
  else if(pid_i_mem_pitch < pid_max_pitch * -1)pid_i_mem_pitch = pid_max_pitch * -1;

  pid_output_pitch = pid_p_gain_pitch * pid_error_temp + pid_i_mem_pitch + pid_d_gain_pitch * (pid_error_temp - pid_last_pitch_d_error);
  if(pid_output_pitch > pid_max_pitch)pid_output_pitch = pid_max_pitch;
  else if(pid_output_pitch < pid_max_pitch * -1)pid_output_pitch = pid_max_pitch * -1;

  pid_last_pitch_d_error = pid_error_temp;

  //Yaw calculations
  pid_error_temp = gyro_yaw_input - pid_yaw_setpoint;
  pid_i_mem_yaw += pid_i_gain_yaw * pid_error_temp;
  if(pid_i_mem_yaw > pid_max_yaw)pid_i_mem_yaw = pid_max_yaw;
  else if(pid_i_mem_yaw < pid_max_yaw * -1)pid_i_mem_yaw = pid_max_yaw * -1;

  pid_output_yaw = pid_p_gain_yaw * pid_error_temp + pid_i_mem_yaw + pid_d_gain_yaw * (pid_error_temp - pid_last_yaw_d_error);
  if(pid_output_yaw > pid_max_yaw)pid_output_yaw = pid_max_yaw;
  else if(pid_output_yaw < pid_max_yaw * -1)pid_output_yaw = pid_max_yaw * -1;

  pid_last_yaw_d_error = pid_error_temp;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MPU6050 start
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
void MPU6050_readData(){

  //Read the MPU-6050
  Wire.beginTransmission(MPU6050_ADDR);                                   //Start communication with the gyro.
  Wire.write(MPU6050_ACCEL_OUT_REGISTER);                                                       //Start reading @ register 43h and auto increment with every read.
  Wire.endTransmission();                                                 //End the transmission.
  Wire.requestFrom(MPU6050_ADDR,14);                                      //Request 14 bytes from the gyro.
  
  while(Wire.available() < 14);                                           //Wait until the 14 bytes are received.
  acc_axis[1] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_x variable.
  acc_axis[2] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_y variable.
  acc_axis[3] = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the acc_z variable.
  temperature = Wire.read()<<8|Wire.read();                               //Add the low and high byte to the temperature variable.
  gyro_axis[1] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  gyro_axis[2] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.
  gyro_axis[3] = Wire.read()<<8|Wire.read();                              //Read high and low part of the angular data.

  if(cal_int == MPU6050_CAL_NUM){
    gyro_axis[1] -= gyro_axis_cal[1];                                       //Only compensate after the calibration.
    gyro_axis[2] -= gyro_axis_cal[2];                                       //Only compensate after the calibration.
    gyro_axis[3] -= gyro_axis_cal[3];                                       //Only compensate after the calibration.
  }
  gyro_roll = gyro_axis[1];                      //Set gyro_roll to the correct axis that was stored in the EEPROM.
  gyro_pitch = gyro_axis[2];                     //Set gyro_pitch to the correct axis that was stored in the EEPROM.
  gyro_yaw = gyro_axis[3];                       //Set gyro_yaw to the correct axis that was stored in the EEPROM.
  
  acc_x = acc_axis[1];                           //Set acc_x to the correct axis that was stored in the EEPROM.
  acc_y = acc_axis[2];                           //Set acc_y to the correct axis that was stored in the EEPROM.
  acc_z = acc_axis[3];                           //Set acc_z to the correct axis that was stored in the EEPROM.  
#if MPU6050_DEBUG == 1
  if((millis() - MPU6050Printtimer > 1000)&&(2==state))
  {
    Serial.print("gyro_roll : ");
    Serial.print(gyro_roll);
    Serial.print(" | gyro_pitch : ");
    Serial.print(gyro_pitch);
    Serial.print(" | gyro_yaw: ");
    Serial.println(gyro_yaw);
    Serial.print("acc_x : ");
    Serial.print(acc_x);
    Serial.print(" | acc_y : ");
    Serial.print(acc_y);
    Serial.print(" | acc_z: ");
    Serial.println(acc_z);

    MPU6050Printtimer = millis();
  }
#endif
}

void MPU6050_Init(){
  //Setup the MPU-6050
  Wire.beginTransmission(MPU6050_ADDR);                                      //Start communication with the address found during search.
  Wire.write(MPU6050_PWR_MGMT_1_REGISTER);                                   //We want to write to the PWR_MGMT_1 register (6B hex) to reset it 
  Wire.write(0x00);                                                          //Set the register bits as 00000000 to activate the gyro
  Wire.endTransmission();                                                    //End the transmission with the gyro.

  Wire.beginTransmission(MPU6050_ADDR);                                      //Start communication with the address found during search.
  Wire.write(MPU6050_GYRO_CONFIG_REGISTER);                                  //We want to write to the GYRO_CONFIG register (1B hex)
  Wire.write(0x08);                                                          //Set the register bits as 00001000 (500dps full scale)
  Wire.endTransmission();                                                    //End the transmission with the gyro

  Wire.beginTransmission(MPU6050_ADDR);                                      //Start communication with the address found during search.
  Wire.write(MPU6050_ACCEL_CONFIG_REGISTER);                                 //We want to write to the ACCEL_CONFIG register (1A hex)
  Wire.write(0x10);                                                          //Set the register bits as 00010000 (+/- 8g full scale range)
  Wire.endTransmission();                                                    //End the transmission with the gyro

  //Let's perform a random register check to see if the values are written correct
  Wire.beginTransmission(MPU6050_ADDR);                                      //Start communication with the address found during search
  Wire.write(MPU6050_GYRO_CONFIG_REGISTER);                                                          //Start reading @ register 0x1B
  Wire.endTransmission();                                                    //End the transmission
  Wire.requestFrom(MPU6050_ADDR, 1);                                         //Request 1 bytes from the gyro
  while(Wire.available() < 1);                                               //Wait until the 6 bytes are received
  if(Wire.read() != 0x08){                                                   //Check if the value is 0x08
    Serial.println("MPU6050 update fail.");                                  //Stay in this loop for ever
  }

  Wire.beginTransmission(MPU6050_ADDR);                                      //Start communication with the address found during search
  Wire.write(MPU6050_CONFIG_REGISTER);                                                          //We want to write to the CONFIG register (1A hex)
  Wire.write(0x03);                                                          //Set the register bits as 00000011 (Set Digital Low Pass Filter to ~43Hz)
  Wire.endTransmission();                                                    //End the transmission with the gyro    
}
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//MPU6050 end
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////


////////////////////////////////////////////////////////////////////////////////////
// BME start
///////////////////////////////////////////////////////////////////////////////////
void BME_Init()
{
  uint8_t osrs_t = 1;             //Temperature oversampling x 1
  uint8_t osrs_p = 1;             //Pressure oversampling x 1
  uint8_t osrs_h = 1;             //Humidity oversampling x 1
  uint8_t mode = 3;               //Normal mode
  uint8_t t_sb = 5;               //Tstandby 1000ms
  uint8_t filter = 0;             //Filter off 
  uint8_t spi3w_en = 0;           //3-wire SPI Disable
  
  uint8_t ctrl_meas_reg = (osrs_t << 5) | (osrs_p << 2) | mode;
  uint8_t config_reg    = (t_sb << 5) | (filter << 2) | spi3w_en;
  uint8_t ctrl_hum_reg  = osrs_h;

  Wire.begin();
    
  writeReg(0xF2,ctrl_hum_reg);
  writeReg(0xF4,ctrl_meas_reg);
  writeReg(0xF5,config_reg);
  readTrim(); 
}
void BME_task()
{
  if(millis() - BME280timer > 1000)
  { 
    double temp_act = 0.0, press_act = 0.0,hum_act=0.0;
    signed long int temp_cal;
    unsigned long int press_cal,hum_cal;

    readData();

    temp_cal = calibration_T(temp_raw);
    press_cal = calibration_P(pres_raw);
    hum_cal = calibration_H(hum_raw);
    temp_act = (double)temp_cal / 100.0;
    press_act = (double)press_cal / 100.0;
    hum_act = (double)hum_cal / 1024.0;
    #if BME_DEBUG == 1
    Serial.print("TEMP : ");
    Serial.print(temp_act);
    Serial.print(" DegC  PRESS : ");
    Serial.print(press_act);
    Serial.print(" hPa  HUM : ");
    Serial.print(hum_act);
    Serial.println(" %");    
    #endif
    BME280timer = millis();
  }
}

void readTrim()
{
    uint8_t data[32],i=0;
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0x88);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,24);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xA1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,1);
    data[i] = Wire.read();
    i++;
    
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xE1);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,7);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;    
    }
    dig_T1 = (data[1] << 8) | data[0];
    dig_T2 = (data[3] << 8) | data[2];
    dig_T3 = (data[5] << 8) | data[4];
    dig_P1 = (data[7] << 8) | data[6];
    dig_P2 = (data[9] << 8) | data[8];
    dig_P3 = (data[11]<< 8) | data[10];
    dig_P4 = (data[13]<< 8) | data[12];
    dig_P5 = (data[15]<< 8) | data[14];
    dig_P6 = (data[17]<< 8) | data[16];
    dig_P7 = (data[19]<< 8) | data[18];
    dig_P8 = (data[21]<< 8) | data[20];
    dig_P9 = (data[23]<< 8) | data[22];
    dig_H1 = data[24];
    dig_H2 = (data[26]<< 8) | data[25];
    dig_H3 = data[27];
    dig_H4 = (data[28]<< 4) | (0x0F & data[29]);
    dig_H5 = (data[30] << 4) | ((data[29] >> 4) & 0x0F);
    dig_H6 = data[31];   
}

void writeReg(uint8_t reg_address, uint8_t data)
{
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(reg_address);
    Wire.write(data);
    Wire.endTransmission();    
}

void readData()
{
    int i = 0;
    uint32_t data[8];
    Wire.beginTransmission(BME280_ADDRESS);
    Wire.write(0xF7);
    Wire.endTransmission();
    Wire.requestFrom(BME280_ADDRESS,8);
    while(Wire.available()){
        data[i] = Wire.read();
        i++;
    }
    pres_raw = (data[0] << 12) | (data[1] << 4) | (data[2] >> 4);
    temp_raw = (data[3] << 12) | (data[4] << 4) | (data[5] >> 4);
    hum_raw  = (data[6] << 8) | data[7];
}

signed long int calibration_T(signed long int adc_T)
{
    
    signed long int var1, var2, T;
    var1 = ((((adc_T >> 3) - ((signed long int)dig_T1<<1))) * ((signed long int)dig_T2)) >> 11;
    var2 = (((((adc_T >> 4) - ((signed long int)dig_T1)) * ((adc_T>>4) - ((signed long int)dig_T1))) >> 12) * ((signed long int)dig_T3)) >> 14;
    
    t_fine = var1 + var2;
    T = (t_fine * 5 + 128) >> 8;
    return T; 
}

unsigned long int calibration_P(signed long int adc_P)
{
    signed long int var1, var2;
    unsigned long int P;
    var1 = (((signed long int)t_fine)>>1) - (signed long int)64000;
    var2 = (((var1>>2) * (var1>>2)) >> 11) * ((signed long int)dig_P6);
    var2 = var2 + ((var1*((signed long int)dig_P5))<<1);
    var2 = (var2>>2)+(((signed long int)dig_P4)<<16);
    var1 = (((dig_P3 * (((var1>>2)*(var1>>2)) >> 13)) >>3) + ((((signed long int)dig_P2) * var1)>>1))>>18;
    var1 = ((((32768+var1))*((signed long int)dig_P1))>>15);
    if (var1 == 0)
    {
        return 0;
    }    
    P = (((unsigned long int)(((signed long int)1048576)-adc_P)-(var2>>12)))*3125;
    if(P<0x80000000)
    {
       P = (P << 1) / ((unsigned long int) var1);   
    }
    else
    {
        P = (P / (unsigned long int)var1) * 2;    
    }
    var1 = (((signed long int)dig_P9) * ((signed long int)(((P>>3) * (P>>3))>>13)))>>12;
    var2 = (((signed long int)(P>>2)) * ((signed long int)dig_P8))>>13;
    P = (unsigned long int)((signed long int)P + ((var1 + var2 + dig_P7) >> 4));
    return P;
}

unsigned long int calibration_H(signed long int adc_H)
{
    signed long int v_x1;
    
    v_x1 = (t_fine - ((signed long int)76800));
    v_x1 = (((((adc_H << 14) -(((signed long int)dig_H4) << 20) - (((signed long int)dig_H5) * v_x1)) + 
              ((signed long int)16384)) >> 15) * (((((((v_x1 * ((signed long int)dig_H6)) >> 10) * 
              (((v_x1 * ((signed long int)dig_H3)) >> 11) + ((signed long int) 32768))) >> 10) + (( signed long int)2097152)) * 
              ((signed long int) dig_H2) + 8192) >> 14));
   v_x1 = (v_x1 - (((((v_x1 >> 15) * (v_x1 >> 15)) >> 7) * ((signed long int)dig_H1)) >> 4));
   v_x1 = (v_x1 < 0 ? 0 : v_x1);
   v_x1 = (v_x1 > 419430400 ? 419430400 : v_x1);
   return (unsigned long int)(v_x1 >> 12);   
}
////////////////////////////////////////////////////////////////////////////////////
// BME end
///////////////////////////////////////////////////////////////////////////////////

////////////////////////////////////////////////////////////////////////////////////
// GPS start
///////////////////////////////////////////////////////////////////////////////////
void GPS_task()
{
  /* GPS start */ 
  while (mySerial.available())
  {
      char c = mySerial.read();
      // Serial.write(c); // uncomment this line if you want to see the GPS data flowing
      if (gps.encode(c)) // Did a new valid sentence come in?
        newData = true;
  }

  if(millis() - gpstimer > 1000) // print data every second
  {
    if (newData)
    {
      newData = false;
      float flat, flon;
      unsigned long age;
#if GPS_DEBUG == 1
      gps.f_get_position(&flat, &flon, &age);
      Serial.print("LAT=");
      Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      Serial.print(" LON=");
      Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      Serial.print(" SAT=");
      Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      Serial.print(" PREC=");
      Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
#endif
    }
    gpstimer = millis();
  }
/* GPS end */
}
////////////////////////////////////////////////////////////////////////////////////
// GPS start
///////////////////////////////////////////////////////////////////////////////////