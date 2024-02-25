#include "pitches.h"
#include <MPU6050_light.h>
#include <Wire.h>
#include <SoftwareSerial.h>
#include <TinyGPS.h>
#include <SPI.h>
#include <SD.h>
#include "RF24.h"
#include <Servo.h>

#define GPS_DEBUG 0
#define BME_DEBUG 0


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
// ---------------------------------------------------------------------------
Servo mot1, mot2, mot3, mot4;


/* RF */
RF24 radio(rfCEPin,rfCSNPin);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
byte bRadio = false;
char radioMessage[6];  // only using 6 characters for TX & ACK payloads
int radioTimeout = 500;
uint8_t address[][6] = { "AAAAAA", "555555" };
uint8_t u8LeftX  = 0;
uint8_t u8LeftY  = 0;
uint8_t u8RightX = 0;
uint8_t u8RightY = 0;
bool bRightButton = false;
bool bLeftButton = false;

enum {
  RF_BME280_TEMPERATURE = 0x00,
  RF_BME280_PRESSURE = 0x00,
  RF_BME280_HUMIDITY = 0x00,
  RF_ACC_X = 0x00,
  RF_ACC_Y = 0x00,
  RF_ACC_Z = 0x00,
  RF_GYRO_X = 0x00,
  RF_GYRO_Y = 0x00,
  RF_GYRO_Z = 0x00,
  RF_GPS_LOCATION = 0x00,
}rfTaskState;

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


//notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};

byte bMPU6050  = 0;
MPU6050 mpu(Wire);
long mpu6050timer = 0;

void setup() {
  
  Wire.begin();
  /* Serial */
  Serial.begin(115200);
  Serial.println("Hello World!");
  mySerial.begin(9600);
  delay(500);    
  // if (!SD.begin(sdCSPin)) 
  // {
  //   Serial.println("initialization failed!");
  //   bSDCard =0;
  // }
  // else
  // {
  //   Serial.println("initialization done.");
  //   bSDCard =1;
  // }
 
  // if(1 == bSDCard)
  // {
  //   // open the file. note that only one file can be open at a time,
  //   // so you have to close this one before opening another.
  //   myFile = SD.open("test.txt", FILE_WRITE);

  //   // if the file opened okay, write to it:
  //   if (myFile) 
  //   {
  //     Serial.print("Writing to test.txt...");
  //     myFile.println("testing 1, 2, 3.");
  //     // close the file:
  //     myFile.close();
  //     Serial.println("done.");
  //   } else 
  //   {
  //       // if the file didn't open, print an error:
  //       Serial.println("error opening test.txt");
  //   }

  //   // re-open the file for reading:
  //   myFile = SD.open("test.txt");
  //   if (myFile) 
  //   {
  //     Serial.println("test.txt:");

  //     // read from the file until there's nothing else in it:
  //     while (myFile.available())
  //     {
  //       Serial.write(myFile.read());
  //     }
  //     // close the file:
  //     myFile.close();
  //   } else 
  //   {
  //     // if the file didn't open, print an error:
  //     Serial.println("error opening test.txt");
  //   }
  // }
  bMPU6050 = mpu.begin();
  if(0x00 == bMPU6050)
  {
    Serial.println(F("Calculating offsets, do not move MPU6050"));
  }

  
  
  delay(500);

  // Buzzer
  // for (int thisNote = 0; thisNote < 8; thisNote++) {
  //   // to calculate the note duration, take one second divided by the note type.
  //   //e.g. quarter note = 1000 / 4, eighth note = 1000/8, etc.
  //   int noteDuration = 1000 / noteDurations[thisNote];
  //   tone(buzzerPin, melody[thisNote], noteDuration);
  //   // to distinguish the notes, set a minimum time between them.
  //   // the note's duration + 30% seems to work well:
  //   int pauseBetweenNotes = noteDuration * 1.30;
  //   delay(pauseBetweenNotes);
  //   // stop the tone playing:
  //   noTone(buzzerPin);
  // }



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
  
  

  if(0x00 == bMPU6050)
  {
    mpu.calcOffsets(true,true); // gyro and accelero
    Serial.println("offset Done!\n");
  }

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

  mot1.attach(pwmMotor1, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot2.attach(pwmMotor2, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot3.attach(pwmMotor3, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot4.attach(pwmMotor4, MIN_PULSE_LENGTH, MAX_PULSE_LENGTH);
  mot1.writeMicroseconds(MIN_PULSE_LENGTH);
  mot2.writeMicroseconds(MIN_PULSE_LENGTH);
  mot3.writeMicroseconds(MIN_PULSE_LENGTH);
  mot4.writeMicroseconds(MIN_PULSE_LENGTH);
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

  uint8_t pipe;
  if (radio.available(&pipe)) {                     // is there a payload? get the pipe number that recieved it
    // load the payload for the first received transmission on pipe 0
    radio.writeAckPayload(1,&radioMessage,6);
    uint8_t bytes = radio.getDynamicPayloadSize();  // get the size of the payload
    char received[6];
    radio.read(received, sizeof(received));  // get incoming ACK payload
    u8LeftX = received[0];
    u8LeftY = received[1];
    u8RightX = received[2];
    u8RightY = received[3];
    Serial.print(F("Recieved on pipe "));
    Serial.print(u8LeftX);     
    Serial.print(" | ");
    Serial.print(u8LeftY);     
    Serial.print(" | ");
    Serial.print(u8RightX);     
    Serial.print(" | ");
    Serial.print(u8RightY);  
    Serial.print(" | ");
    Serial.print(0x00 != (received[4]&&0x01)?"PRESS":"RELEASED");
    Serial.print(" | ");
    Serial.println(0x00 != (received[4]&&0x02)?"PRESS":"RELEASED");
    Serial.print(F(" Sent: "));
    Serial.println(radioMessage);    // print outgoing message
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