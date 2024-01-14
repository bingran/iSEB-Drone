#include <TinyGPS.h>
#include <SoftwareSerial.h>

/* Pinout Definition */
#define gpsTx 7
#define gpsRx 4

/*GPS*/
TinyGPS gps;
SoftwareSerial mySerial(4,7);
long gpstimer = 0;
bool newData = false;
unsigned long chars;
unsigned short sentences, failed;

void setup() {
  
  /* Serial */
  Serial.begin(115200);
  Serial.println("Hello World!");
  mySerial.begin(9600);
}

void loop() {
    while (mySerial.available())
  {
      char c = mySerial.read();
      Serial.write(c); // uncomment this line if you want to see the GPS data flowing
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
      gps.f_get_position(&flat, &flon, &age);
      Serial.print("LAT=");
      Serial.print(flat == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flat, 6);
      Serial.print(" LON=");
      Serial.print(flon == TinyGPS::GPS_INVALID_F_ANGLE ? 0.0 : flon, 6);
      Serial.print(" SAT=");
      Serial.print(gps.satellites() == TinyGPS::GPS_INVALID_SATELLITES ? 0 : gps.satellites());
      Serial.print(" PREC=");
      Serial.println(gps.hdop() == TinyGPS::GPS_INVALID_HDOP ? 0 : gps.hdop());
    }
    gpstimer = millis();
  }
}
