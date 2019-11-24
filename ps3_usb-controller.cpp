#include <PS3USB.h>

// Satisfy the IDE, which needs to see the include statment in the ino too.
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <Geometry.h>

USB Usb;
/* You can create the instance of the class in two ways */
PS3USB PS3(&Usb); // This will just create the instance
//PS3USB PS3(&Usb,0x00,0x15,0x83,0x3D,0x0A,0x57); // This will also store the bluetooth address - this can be obtained from the dongle when running the sketch

const float SERVOMIN = 175; // 'minimum' pulse length count (out of 4096)
const float SERVOMAX = 475; // 'maximum' pulse length count (out of 4096)
const float SERVOMID[6] = {294, 350, 308, 352, 363, 355}; // 'mid' pulse length count (out of 4096)
const float PULSE_PER_RAD = 138.8;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Rotation get_rot_matrix(float desired_angles[]){
  Rotation rot_matrix;
  rot_matrix.RotateX(desired_angles[0]);
  rot_matrix.RotateY(desired_angles[1]);
  rot_matrix.RotateZ(desired_angles[2]);
  return rot_matrix;
}


// T and desired_angles are input, p, b, beta, s and a are constants, out is alpha
void inverse_kin(Point pos, float desired_angles[], Point p[], Point b[], float beta[], float s, float a, float out[]) {

  float tmp[6];

  Rotation rot_matrix = get_rot_matrix(desired_angles);

  Point l;

  for (int i = 0; i < 6; i++) {

    l = pos + rot_matrix * p[i] - b[i];

    float L = pow(l.X(), 2) + pow(l.Y(), 2) + pow(l.Z(), 2) + pow(a, 2) - pow(s, 2);
    float M = 2 * a * l.Z();
    float N = 2 * a * (cos(beta[i]) * l.X() + sin(beta[i]) * l.Y());

    tmp[i] = asin(L / sqrt(pow(M, 2) + pow(N, 2))) - atan2(N, M);

    if(isnan(tmp[i])){
      Serial.print("INVERSE KIN NOT POSSIBLE.\n");
      // Serial.print(" ");
      // Serial.print("Motor: ");
      // Serial.print(i);
      // Serial.print(", l.X(): ");
      // Serial.print(l.X());
      // Serial.print(", l.Y(): ");
      // Serial.print(l.Y());
      // Serial.print(", l.Z(): ");
      // Serial.print(l.Z());
      // Serial.print(", L: ");
      // Serial.print(L);
      // Serial.print(", M: ");
      // Serial.print(M);
      // Serial.print(", N: ");
      // Serial.print(N);
      // Serial.print("\n");
      return;
    }
  }

  for (int i = 0; i < 6; i++) {
    out[i] = tmp[i];
  }  
  return;
}

// Helper function defines constants
void inverse_kin_helper(float desired_angles[], float out[]){
  float beta[6] = {0, 180, 240, 60, 120, 300}; // servo arm angles
  for(int i=0; i<6; i++) {
    beta[i] /= DEG_TO_RAD
  }
  float a = 16.03;
  float s = 149.9;
  
  Point p[6]; // servo position
  Point b[6]; // bearing positions
  
  p[0].X() = 29.1; p[0].Y() = -110.9; p[0].Z() = 0;
  p[1].X() = -28.9; p[1].Y() = -110.9; p[1].Z() = 0;
  p[2].X() = -110.5; p[2].Y() = 30; p[2].Z() = 0;
  p[3].X() = -81.42; p[3].Y() = 80.38; p[3].Z() = 0;
  p[4].X() = 81.42; p[4].Y() = 80.38; p[4].Z() = 0;
  p[5].X() = 110.5; p[5].Y() = 30; p[5].Z() = 0;
  
  b[0].X() = 81.5; b[0].Y() = -67.26; b[0].Z() = 0;
  b[1].X() = -81.5; b[1].Y() = -67.26; b[1].Z() = 0;
  b[2].X() = -99; b[2].Y() = -36.95; b[2].Z() = 0;
  b[3].X() = -17.5; b[3].Y() = 104.21; b[3].Z() = 0;
  b[4].X() = 17.5; b[4].Y() = 104.21; b[4].Z() = 0;
  b[5].X() = 99; b[5].Y() = -36.95; b[5].Z() = 0;

  
  Point pos;
  pos.X() = 0; pos.Y() = 0; pos.Z() = 126.17;
  inverse_kin(pos, desired_angles, p, b, beta, s, a, out);
  return;
}

// desired angle is roll (x), pitch 󰀀, yaw (unchanged, 0)
void move_motors(float current_state[], float x, float y, float out[]){
  y = 255 - y;
  float DELTA = 0.03; // TUNE THIS
  if (x >= 128){
    current_state[0] += DELTA*((x-128)/127)*DEG_TO_RAD; // positive
  }else{
    current_state[0] += DELTA*((x-127)/127)*DEG_TO_RAD; // negative
  } 
  if (y >= 128){
    current_state[1] += DELTA*((y-128)/127)*DEG_TO_RAD; // positive
  }else{
    current_state[1] += DELTA*((y-127)/127)*DEG_TO_RAD; // negative
  }
  current_state[2] = 0;
  // Serial.print("Desired State: ");
  // Serial.print(current_state[0]);
  // Serial.print(", ");
  // Serial.print(current_state[1]);
  // Serial.print(" FROM ");
  // Serial.print(x);
  // Serial.print(", ");
  // Serial.print(y);
  // Serial.print("\n");
  inverse_kin_helper(current_state, out);
}

// Initialize servo state and set neutral
int servo_PWM[6];
float servo_angles[6];
float current_state[3];

void setup() {
  Serial.begin(9600);
#if !defined(MIPSEL)
  while (!Serial); // Wait for serial port to connect - used on Leonardo, Teensy and other boards with built-in USB CDC serial connection
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); //halt
  }
  Serial.print(F("\r\nPS3 USB Library Started"));
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
 
  for (int i=0; i<6; i++) {
    servo_angles[i] = 0;
    servo_PWM[i] = SERVOMID[i];
    pwm.setPWM(i, 0, servo_PWM[i]); // added +1 to match PWM port numbering (pins 1..6 used)
  }
  for (int i=0; i<3; i++) {
    current_state[i]=0;
  }

}


// Print only when something changes
bool flag = true;
bool neutral = true;

void loop() {
  Usb.Task();

  if (PS3.PS3Connected) {
    if (PS3.getAnalogHat(LeftHatX) > 132 || PS3.getAnalogHat(LeftHatX) < 122 || PS3.getAnalogHat(LeftHatY) > 132 || PS3.getAnalogHat(LeftHatY) < 122) {
      // Serial.print(F("\r\nLeftHatX: "));
      // Serial.print(PS3.getAnalogHat(LeftHatX));
      // Serial.print(F("\tLeftHatY: "));
      // Serial.print(PS3.getAnalogHat(LeftHatY));
      // Serial.print("\n");
      move_motors(current_state, PS3.getAnalogHat(LeftHatX), PS3.getAnalogHat(LeftHatY), servo_angles);
      flag = true;
      neutral = false;
      for (int i=0; i<6; i++) {
        servo_PWM[i] = max(min(((int)(servo_angles[i] * PULSE_PER_RAD) + SERVOMID[i]), SERVOMAX), SERVOMIN);
        pwm.setPWM(i, 0, servo_PWM[i]);
      }
    }
    else if (PS3.getButtonClick(PS) && !neutral){
      // set neutral
      neutral = true;
      flag = true;
      for (int i=0; i<6; i++) {
        servo_angles[i] = 0;
        servo_PWM[i] = SERVOMID[i];
        pwm.setPWM(i, 0, servo_PWM[i]); // added +1 to match PWM port numbering (pins 1..6 used)
      }
      for (int i=0; i<3; i++) {
        current_state[i]=0;
      }
    }
  }
  
  if (flag) {
    flag=false;
    for (int i=0; i<6; i++) {
      Serial.print(servo_PWM[i]);
      Serial.print(" ");
    }
    Serial.print("\n");
    for (int i=0; i<6; i++) {
      float angle_print = servo_angles[i]/DEG_TO_RAD;
      Serial.print(angle_print);
      Serial.print(" ");
    }
    Serial.print("\n");
  }
}