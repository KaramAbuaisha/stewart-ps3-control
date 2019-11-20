/*
 Example sketch for the PS3 USB library - developed by Kristian Lauszus
 For more information visit my blog: http://blog.tkjelectronics.dk/ or
 send me an e-mail:  kristianl@tkjelectronics.com
 */

#include <PS3USB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <revKin.cpp>
#include <math.h>
#include <Geometry.h>

#define PI 3.14159265

USB Usb;
/* You can create the instance of the class in two ways */
PS3USB PS3(&Usb); // This will just create the instance
//PS3USB PS3(&Usb,0x00,0x15,0x83,0x3D,0x0A,0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

bool printAngle = True;
uint8_t state = 0;

const int SERVOMIN = 175; // 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 465; // 'maximum' pulse length count (out of 4096)
const int SERVOMID = floor(320); // 'mid' pulse length count (out of 4096)
const int PS3_CENTER = 127;
const int TOL = 5;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

String valInput; // Serial input var.
int i=0; // loop index var.
int val[6] = {SERVOMID, SERVOMID, SERVOMID, SERVOMID, SERVOMID, SERVOMID}; // PWM var

// inverse kin constants 

Point p[6];
Point b[6];

p[0].X() = -111; p[0].Y() = 29; p[0].Z() = 0;
p[1].X() = -111; p[1].Y() = -29; p[1].Z() = 0;
p[2].X() = 27.5; p[2].Y() = -108.63; p[2].Z() = 0;
p[3].X() = 77.5; p[3].Y() = -81.2; p[3].Z() = 0;
p[4].X() = 77.4; p[4].Y() = 81.6; p[4].Z() = 0;
p[5].X() = 27; p[5].Y() = 109.6; p[5].Z() = 0;

b[0].X() = 104.2; b[0].Y() = -17.5; b[0].Z() = 0;
b[1].X() = 104.2; b[0].Y() = 17.5; b[1].Z() = 0;
b[2].X() = -36.95; b[2].Y() = 99; b[2].Z() = 0;
b[3].X() = -67.3; b[3].Y() = 81.5; b[3].Z() = 0;
b[4].X() = -67.3; b[3].Y() = -81.5; b[4].Z() = 0;
b[5].X() = -37; b[5].Y() = -99; b[5].Z() = 0;

point T;
T.X() = 0; T.Y() = 0; T.Z() = 9;

float a = 16.03;
float s = 145.23;


void setup() {
  Serial.begin(9600);
#if !defined(__MIPSEL__)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 USB Library Started"));
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // set neutral 
  for (i=0; i<6; i++) {
    pwm.setPWM(i+1, 0, SERVOMID); // added +1 to match PWM port numbering (pins 1..6 used)
  }

}

// desired angle is roll (x), pitch (y), yaw (unchanged, 0)
void move_motors(uint8_t x, uint8_t y){
  y = 255 - y;
  float desired_angles[3];  
  if (x >= 128){
    desired_angles[0] = 30*(x-128)*(PI/180); // positive
  }else{
    desired_angles[0] = 30*(x-127)*(PI/180); // negative
  } 
  if (y >= 128){
    desired_angles[1] = 30*(y-128)*(PI/180); // positive
  }else{
    desired_angles[1] = 30*(y-127)*(PI/180); // negative
  }
  desired_angles[2] = 0;


}

void loop() {
  Usb.Task();

  if (PS3.PS3Connected) {
    if (PS3.getAnalogHat(LeftHatX) > 132 || PS3.getAnalogHat(LeftHatX) < 122 || PS3.getAnalogHat(LeftHatY) > 132 || PS3.getAnalogHat(LeftHatY) < 122) {
      Serial.print(F("\r\nLeftHatX: "));
      Serial.print(PS3.getAnalogHat(LeftHatX));
      Serial.print(F("\tLeftHatY: "));
      Serial.print(PS3.getAnalogHat(LeftHatY));
      move_motors(LeftHatX, LeftHatY);
    }
    else{
      // set neutral 
      for (i=0; i<6; i++) {
        pwm.setPWM(i+1, 0, SERVOMID); // added +1 to match PWM port numbering (pins 1..6 used)
      }
    }
    // Analog button values can be read from almost all buttons
    if (PS3.getAnalogButton(L2) || PS3.getAnalogButton(R2)) {
      Serial.print(F("\r\nL2: "));
      Serial.print(PS3.getAnalogButton(L2));
      if (!PS3.PS3NavigationConnected) {
        Serial.print(F("\tR2: "));
        Serial.print(PS3.getAnalogButton(R2));
      }
    }
    // if (PS3.getButtonClick(PS))
    //   Serial.print(F("\r\nPS"));

    // if (PS3.getButtonClick(TRIANGLE))
    //   Serial.print(F("\r\nTraingle"));
    // if (PS3.getButtonClick(CIRCLE))
    //   Serial.print(F("\r\nCircle"));
    // if (PS3.getButtonClick(CROSS))
    //   Serial.print(F("\r\nCross"));
    // if (PS3.getButtonClick(SQUARE))
    //   Serial.print(F("\r\nSquare"));

    // if (PS3.getButtonClick(UP)) {
    //   Serial.print(F("\r\nUp"));
    //   PS3.setLedOff();
    //   PS3.setLedOn(LED4);
    // }
    // if (PS3.getButtonClick(RIGHT)) {
    //   Serial.print(F("\r\nRight"));
    //   PS3.setLedOff();
    //   PS3.setLedOn(LED1);
    // }
    // if (PS3.getButtonClick(DOWN)) {
    //   Serial.print(F("\r\nDown"));
    //   PS3.setLedOff();
    //   PS3.setLedOn(LED2);
    // }
    // if (PS3.getButtonClick(LEFT)) {
    //   Serial.print(F("\r\nLeft"));
    //   PS3.setLedOff();
    //   PS3.setLedOn(LED3);
    // }

    // if (PS3.getButtonClick(L1))
    //   Serial.print(F("\r\nL1"));
    // if (PS3.getButtonClick(L3))
    //   Serial.print(F("\r\nL3"));
    // if (PS3.getButtonClick(R1))
    //   Serial.print(F("\r\nR1"));
    // if (PS3.getButtonClick(R3))
    //   Serial.print(F("\r\nR3"));

    // if (PS3.getButtonClick(SELECT)) {
    //   Serial.print(F("\r\nSelect - "));
    //   PS3.printStatusString();
    // }
    // if (PS3.getButtonClick(START)) {
    //   Serial.print(F("\r\nStart"));
    //   printAngle = !printAngle;
    // }
    if (printAngle) {
      Serial.print(F("\r\nPitch: "));
      Serial.print(PS3.getAngle(Pitch));
      Serial.print(F("\tRoll: "));
      Serial.print(PS3.getAngle(Roll));
    }
  }
  // else if (PS3.PS3MoveConnected) { // One can only set the color of the bulb, set the rumble, set and get the bluetooth address and calibrate the magnetometer via USB
  //   if (state == 0) {
  //     PS3.moveSetRumble(0);
  //     PS3.moveSetBulb(Off);
  //   } else if (state == 1) {
  //     PS3.moveSetRumble(75);
  //     PS3.moveSetBulb(Red);
  //   } else if (state == 2) {
  //     PS3.moveSetRumble(125);
  //     PS3.moveSetBulb(Green);
  //   } else if (state == 3) {
  //     PS3.moveSetRumble(150);
  //     PS3.moveSetBulb(Blue);
  //   } else if (state == 4) {
  //     PS3.moveSetRumble(175);
  //     PS3.moveSetBulb(Yellow);
  //   } else if (state == 5) {
  //     PS3.moveSetRumble(200);
  //     PS3.moveSetBulb(Lightblue);
  //   } else if (state == 6) {
  //     PS3.moveSetRumble(225);
  //     PS3.moveSetBulb(Purple);
  //   } else if (state == 7) {
  //     PS3.moveSetRumble(250);
  //     PS3.moveSetBulb(White);
  //   }

  //   state++;
  //   if (state > 7)
  //     state = 0;
  //   delay(1000);
  // }
}