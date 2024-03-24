#include <SoftwareSerial.h>

/* Pinout Definition */
#define bluetoothTx 14
#define bluetoothRx 15

/*bluetooth*/
SoftwareSerial mySerial(bluetoothRx,bluetoothTx); // RX, TX


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  mySerial.begin(9600);
  Serial.write("Hello World\n");
}

void loop() {
  // put your main code here, to run repeatedly:
  delay(1);
  bluetooth();
}

void bluetooth()
{
  if (mySerial.available()) {
    Serial.print("bluetooth :");
    Serial.write(mySerial.read());
    Serial.println();
  }
}