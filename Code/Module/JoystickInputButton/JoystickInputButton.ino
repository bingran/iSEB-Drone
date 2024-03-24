/* Pinout Definition */
#define leftButtonPin 5
#define rightButtonPin 4

/* state of buttons */
bool bRightButton = false;
bool bLeftButton = false;


void setup() {
  /* Serial */
  Serial.begin(115200);
  Serial.println("Hello World!");
}

void loop() {
  /* Detect change state of left button */
  if (bLeftButton != digitalRead(leftButtonPin)) {
    bLeftButton = digitalRead(leftButtonPin);
    if (0 == bLeftButton) {
      Serial.println("Left button is pressed");
    } else {
      Serial.println("Left button is released");
    }
  }
  /* Detect change state of right button */
  if (bRightButton != digitalRead(rightButtonPin)) {
    bRightButton = digitalRead(rightButtonPin);
    if (0 == bRightButton) {
      Serial.println("Right button is pressed");
    } else {
      Serial.println("Right button is released");
    }
  }
}
