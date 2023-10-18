/*
This is a test sketch for the Adafruit assembled Motor Shield for Arduino v2
It won't work with v1.x motor shields! Only for the v2's with built in PWM
control

For use with the Adafruit Motor Shield v2
---->	http://www.adafruit.com/products/1438
*/

#include <Adafruit_MotorShield.h>
#include <Encoder.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Encoder knobLeft(1, 2);
Encoder knobRight(3, 4);
// Or, create it with a different I2C address (say for stacking)
// Adafruit_MotorShield AFMS = Adafruit_MotorShield(0x61);

// Select which 'port' M1, M2, M3 or M4. In this case, M1
Adafruit_DCMotor *leftMotor = AFMS.getMotor(2);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(3);
// You can also make another motor on port M2
//Adafruit_DCMotor *myOtherMotor = AFMS.getMotor(2);

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps
  Serial.println("Adafruit Motorshield v2 - DC Motor test!");

  if (!AFMS.begin()) {         // create with the default frequency 1.6KHz
  // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");
}

void loop() {
  //uint8_t i;
  long newLeft, newRight;

  knobLeft.write(0);
  knobRight.write(0);

  leftMotor->setSpeed(65);
  rightMotor->setSpeed(70);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(2000);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  newLeft = knobLeft.read();
  Serial.print("Left = ");
  Serial.print(newLeft);
  Serial.println();
  newRight = knobRight.read();
  Serial.print("Right = ");
  Serial.print(newRight);
  Serial.println();
  delay(100000);

  // myMotor->run(FORWARD);
  // for (i=0; i<255; i++) {
  //   myMotor->setSpeed(i);
  //   delay(10);
  // }
  // for (i=255; i!=0; i--) {
  //   myMotor->setSpeed(i);
  //   delay(10);
  // }
  // newLeft = knobLeft.read();
  // Serial.print("Left = ");
  // Serial.print(newLeft);
  // Serial.println();

  // myMotor->run(BACKWARD);
  // for (i=0; i<255; i++) {
  //   myMotor->setSpeed(i);
  //   delay(10);
  // }
  // for (i=255; i!=0; i--) {
  //   myMotor->setSpeed(i);
  //   delay(10);
  // }
  // newLeft = knobLeft.read();
  // Serial.print("Left = ");
  // Serial.print(newLeft);
  // Serial.println();

  // myMotor->run(RELEASE);
  // delay(1000);
}