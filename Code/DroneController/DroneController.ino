#include "pitches.h"
#include "Adafruit_NeoPixel.h"
#include <Wire.h>
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7735.h> // Hardware-specific library for ST7735
#include <SPI.h>
#include "RF24.h"

/* Pinout Definition */
#define rfCSNPin 10
#define rfCEPin 9
#define buzzerPin 8
#define tftCsPin  7
#define tftSD_CSNPin 6
#define rgbledPin 4
#define leftButtonPin 3
#define rightButtonPin 2

#define tftDcPin    A5
#define tftRstPin   A4 // Or set to -1 and connect to Arduino RESET pin
#define rightXPin   A3
#define rightYPin   A2
#define leftXPin    A1
#define leftYPin   A0

/* RF */
RF24 radio(rfCEPin,rfCSNPin);  // using pin 7 for the CE pin, and pin 8 for the CSN pin
byte bRadio = false;
char radioMessage[6];  // only using 6 characters for TX & ACK payloads
int radioTimeout = 500;
uint8_t address[][6] = { "AAAAAA", "555555" };
uint8_t bRadioStatus = 0;
uint8_t bPrevRadioStatus = 0;

bool bBuzzer = false;

bool bRightButton = false;
bool bLeftButton = false;

int timeout = 1000;
int displayTimeout = 100;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(1, rgbledPin, NEO_GRB + NEO_KHZ800);


// The colors we actually want to use
uint16_t        Display_Text_Color         = 0x0000;
uint16_t        Display_Backround_Color    = 0xFFFF;
Adafruit_ST7735 tft = Adafruit_ST7735(tftCsPin, tftDcPin, tftRstPin);
String displayString = "HELLO WORLD!";


float flLeftX = 0;
float flLeftY = 0;
float flRightX = 0;
float flRightY = 0;
float flPrevLeftX = 0;
float flPrevLeftY = 0;
float flPrevRightX = 0;
float flPrevRightY = 0;
float flOffsetLeftX  = 0 ;
float flOffsetLeftY  = 0 ;
float flOffsetRightX  = 0 ;
float flOffsetRightY  = 0 ;


/* buzzer */
// int tempo = 108;
// int melody[] = {
//   // Dart Vader theme (Imperial March) - Star wars 
//   // Score available at https://musescore.com/user/202909/scores/1141521
//   // The tenor saxophone part was used
  
//   NOTE_AS4,8, NOTE_AS4,8, NOTE_AS4,8,//1
//   NOTE_F5,2, NOTE_C6,2,
//   NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F6,2, NOTE_C6,4,  
//   NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F6,2, NOTE_C6,4,  
//   NOTE_AS5,8, NOTE_A5,8, NOTE_AS5,8, NOTE_G5,2, NOTE_C5,8, NOTE_C5,8, NOTE_C5,8,
//   NOTE_F5,2, NOTE_C6,2,
//   NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F6,2, NOTE_C6,4,  
//   NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F6,2, NOTE_C6,4, //8  
//   NOTE_AS5,8, NOTE_A5,8, NOTE_AS5,8, NOTE_G5,2, NOTE_C5,-8, NOTE_C5,16, 
//   NOTE_D5,-4, NOTE_D5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
//   NOTE_F5,8, NOTE_G5,8, NOTE_A5,8, NOTE_G5,4, NOTE_D5,8, NOTE_E5,4,NOTE_C5,-8, NOTE_C5,16,
//   NOTE_D5,-4, NOTE_D5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
//   NOTE_C6,-8, NOTE_G5,16, NOTE_G5,2, REST,8, NOTE_C5,8,//13
//   NOTE_D5,-4, NOTE_D5,8, NOTE_AS5,8, NOTE_A5,8, NOTE_G5,8, NOTE_F5,8,
//   NOTE_F5,8, NOTE_G5,8, NOTE_A5,8, NOTE_G5,4, NOTE_D5,8, NOTE_E5,4,NOTE_C6,-8, NOTE_C6,16,
//   NOTE_F6,4, NOTE_DS6,8, NOTE_CS6,4, NOTE_C6,8, NOTE_AS5,4, NOTE_GS5,8, NOTE_G5,4, NOTE_F5,8,
//   NOTE_C6,1
// };

// int notes = sizeof(melody) / sizeof(melody[0]) / 2;
// // note durations: 4 = quarter note, 8 = eighth note, etc.:
// int noteDurations[] = {
//   4, 8, 8, 4, 4, 4, 4, 4
// };
// int wholenote = (60000 * 4) / tempo;
// int divider = 0, noteDuration = 0;

// Buzzer 2 notes in the melody:
int melody[] = {
  NOTE_C4, NOTE_G3, NOTE_G3, NOTE_A3, NOTE_G3, 0, NOTE_B3, NOTE_C4
};

// note durations: 4 = quarter note, 8 = eighth note, etc.:
int noteDurations[] = {
  4, 8, 8, 4, 4, 4, 4, 4
};


void setup() {
  /* Serial */
  Serial.begin(115200);
  Serial.println("Hello World!");


  /* Strip */
  strip.begin();
  strip.setBrightness(4);  // 0 = minimum, 255- maximum
  strip.setPixelColor(0,255,0,255);
  strip.show();

  tft.initR(INITR_BLACKTAB);      // Init ST7735S chip, black tab
  // initialise the display
  tft.setFont();
  tft.fillScreen(Display_Backround_Color);
  tft.setTextColor(Display_Text_Color);
  tft.setTextSize(1);
  tft.setCursor(1,1);
  tft.print("SM LAB");
  tft.setCursor(1,10);
  tft.print("Left Button");
  tft.setCursor(1,40);
  tft.print("Right Button");

  if (!radio.begin()) {
    bRadio = false;
    bRadioStatus = 0xFF;
    Serial.println(F("radio hardware is not responding!!"));
  }
  else
  {
    bRadio = true;
    bRadioStatus = 0x01;
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
    radio.openWritingPipe(address[0]);  // always uses pipe 0

    // set the RX address of the TX node into a RX pipe
    radio.openReadingPipe(1,address[1]);  // using pipe 1
    memset(radioMessage,0x00,6);
    radio.stopListening();                 // put radio in TX mode
  }
  // the display is now on
  // isDisplayVisible = true;

  // Buzzer 2 
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
  // Buzzer 2 END 
  // Buzzer 1 
  // for (int thisNote = 0; thisNote < notes * 2; thisNote = thisNote + 2) {
  //   // calculates the duration of each note
  //   divider = melody[thisNote + 1];
  //   if (divider > 0) {
  //     // regular note, just proceed
  //     noteDuration = (wholenote) / divider;
  //   } else if (divider < 0) {
  //     // dotted notes are represented with negative durations!!
  //     noteDuration = (wholenote) / abs(divider);
  //     noteDuration *= 1.5; // increases the duration in half for dotted notes
  //   }

  //   // we only play the note for 90% of the duration, leaving 10% as a pause
  //   tone(buzzerPin, melody[thisNote], noteDuration*0.9);

  //   // Wait for the specief duration before playing the next note.
  //   delay(noteDuration);

  //   // stop the waveform generation before the next note.
  //   noTone(buzzerPin);
  // }
  // Buzzer 1 ENd

   /* button */
  Serial.println("Calculating button adc offset please wait.");
  /* expect to be 512 */
  flLeftX   = analogRead(leftXPin);
  flLeftY   = analogRead(leftYPin);
  flRightX  = analogRead(rightXPin);
  flRightY  = analogRead(rightYPin);
  Serial.print((flLeftX));     
  Serial.print(" | ");
  Serial.print((flLeftY));     
  Serial.print(" | ");
  Serial.print((flRightX));     
  Serial.print(" | ");
  Serial.println((flRightY)); 
  flOffsetLeftX   = flLeftX  - 512 ;
  flOffsetLeftY   = flLeftY  - 512 ;
  flOffsetRightX  = flRightX - 512 ;
  flOffsetRightY  = flRightY - 512 ;
  Serial.print(flOffsetLeftX);     
  Serial.print(" | ");
  Serial.print(flOffsetLeftY);     
  Serial.print(" | ");
  Serial.print(flOffsetRightX);     
  Serial.print(" | ");
  Serial.println(flOffsetRightY);  
}

void updateaAdcButton()
{
  flLeftX   = analogRead(leftXPin) - flOffsetLeftX;
  flLeftY   = analogRead(leftYPin) - flOffsetLeftY;
  flRightX = analogRead(rightXPin) - flOffsetRightX;
  flRightY = analogRead(rightYPin) - flOffsetRightY;
  flLeftX = 1027- flLeftX;
  flRightX = 1027 - flRightX;
  
  if(flLeftX<62)
    flLeftX = 62;
  else if(flLeftX>962)
    flLeftX = 962 ;
  if(flLeftY<62)
    flLeftY = 62;
  else if(flLeftY>962)
    flLeftY = 962 ;
  if(flRightX<62)
    flRightX = 62;
  else if(flRightX>962)
    flRightX = 962 ;
  if(flRightY<62)
    flRightY = 62;
  else if(flRightY>962)
    flRightY = 962 ;
    
  Serial.print((flLeftX));     
  Serial.print(" | ");
  Serial.print((flLeftY));     
  Serial.print(" | ");
  Serial.print((flRightX));     
  Serial.print(" | ");
  Serial.println((flRightY)); 
}
void loop() {
  // put your main code here, to run repeatedly:
   delay(1);
  if(0 != timeout)
  {
    timeout--;
    if(0 == timeout)
    {
      timeout = 500;
      updateaAdcButton();
      if((flPrevLeftX != flLeftX) || (flPrevLeftY != flLeftY))
      {
        tft.setTextColor(Display_Backround_Color);
        tft.setCursor(1,30); 
        tft.print(String(flPrevLeftX)+" | "+String(flPrevLeftY));
        tft.setTextColor(Display_Text_Color);
        tft.setCursor(1,30); 
        tft.print(String(flLeftX)+" | "+String(flLeftY));
        flPrevLeftY = flLeftY;
        flPrevLeftX = flLeftX;
      }
      if((flPrevRightX != flRightX) || (flPrevRightY != flRightY))
      {
        tft.setTextColor(Display_Backround_Color);
        tft.setCursor(1,60); 
        tft.print(String(flPrevRightX)+" | "+String(flPrevRightY));
        tft.setTextColor(Display_Text_Color);
        tft.setCursor(1,60); 
        tft.print(String(flRightX)+" | "+String(flRightY));
        flPrevRightY = flRightY;
        flPrevRightX = flRightX;
      }
    }
  } 

  if(0 != displayTimeout)
  {
    displayTimeout--;
    if(0 == displayTimeout)
    {
      /* only gt 21 char */
      displayTimeout = 500;
      strip.setPixelColor(0,flRightX*255/1028,flRightY*255/1028,flLeftX*255/1028);
      strip.show();
    }
  }
  if (bLeftButton != digitalRead(leftButtonPin)) {
    bLeftButton = digitalRead(leftButtonPin);
    tft.fillRect(1,20,150, 10, ST77XX_WHITE);
    tft.setCursor(1,20);
    if (0 == bLeftButton) {
      Serial.println("Left button is pressed");
      tone(buzzerPin, NOTE_C4, 150);
      tft.print("Pressed");
    } else {
      Serial.println("Left button is released");
      tft.print("Released");
    }
  }

  if (bRightButton != digitalRead(rightButtonPin)) {
    bRightButton = digitalRead(rightButtonPin);
    tft.fillRect(1,50,150, 10, ST77XX_WHITE);
    tft.setCursor(1,50);
    if (0 == bRightButton) {
      Serial.println("Right button is pressed");
      tone(buzzerPin, NOTE_G7, 150);
      bBuzzer = true;
      tft.print("Pressed");
    } else {
      Serial.println("Right button is released");
      tft.print("Released");
    }
  }

  if(bRadio)
  {
    if(0 != radioTimeout)
    {
      radioTimeout--;
      if(0 == radioTimeout)
      {
        radioTimeout = 200;
        updateaAdcButton();
        radioMessage[0]= (uint8_t) ((flLeftX-62)*255/900);
        radioMessage[1]= (uint8_t) ((flLeftY-62)*255/900);
        radioMessage[2]= (uint8_t) ((flRightX-62)*255/900);
        radioMessage[3]= (uint8_t) ((flRightY-62)*255/900);
        radioMessage[4] = 0x00;
        radioMessage[4]|= (!bRightButton)?0x01:0x00;
        radioMessage[4]|= (!bLeftButton)?0x02:0x00;
        radioMessage[5] = 0xFF;
        bool report = radio.write(radioMessage,6);  // transmit & save the report
  
        
        if(report)
        {
          // Serial.print(F("Tx: "));  // payload was delivered
          // Serial.print(radioMessage);  // print the outgoing message
          uint8_t pipe;
          if (radio.available(&pipe)) {  // is there an ACK payload? grab the pipe number that received it
            // char received[6];
            // radio.read(received, sizeof(received));  // get incoming ACK payload
            // Serial.print(F(" Rx "));
            // Serial.print(pipe);  // print pipe number that received the ACK
            // Serial.print(F(": "));
            // Serial.println(received);    // print incoming message
          }
          else 
          {
            // Serial.println(F(" Recieved: an empty ACK packet"));  // empty ACK packet received
          }
        } 
        else 
        {
          // Serial.println(F("Transmission failed or timed out"));  // payload was not delivered
        }
      }
    }
  }
}

