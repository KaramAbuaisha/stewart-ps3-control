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

const float SERVOMIN = 160; // 'minimum' pulse length count (out of 4096)
const float SERVOMAX = 460; // 'maximum' pulse length count (out of 4096)
const float SERVOMID[6] = {323, 337, 342, 317, 342, 307}; // 'mid' pulse length count (out of 4096)
//const float SERVOMID[6] = {327, 339, 331, 314, 339, 311}; // 'mid' pulse length count (out of 4096)
const float PULSE_PER_RAD = 138.8;

int servo_PWM[6];
float servo_angles[6];
float current_state[3];
float desired_state[3];

// Print only when something changes
bool neutral = true;

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Rotation get_rot_matrix(){
  Rotation rot_matrix;
  rot_matrix.RotateX(current_state[0]);
  rot_matrix.RotateY(current_state[1]);
  rot_matrix.RotateZ(current_state[2]);
  return rot_matrix;
}

// Inverse kin parameters
float beta[6] = {0, 180, 240, 60, 120, 300}; // servo arm angles
for(int i=0; i<6; i++) {
  beta[i] *= DEG_TO_RAD;
}
float a = 12.50;
float s = 138;

Point p[6]; // servo position
Point b[6]; // bearing positions

p[0].X() = 29; p[0].Y() = -110.9; p[0].Z() = 0;
p[1].X() = -29; p[1].Y() = -110.9; p[1].Z() = 0;
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
pos.X() = 0; pos.Y() = 0; pos.Z() = 114;

// T and desired_angles are input, p, b, beta, s and a are constants, servo_angles is alpha, current_state is [pitch, roll, yaw=0]
bool inverse_kin() {

  float tmp[6];

  Rotation rot_matrix = get_rot_matrix();

  Point l;

  for (int i = 0; i < 6; i++) {

    l = pos + rot_matrix * p[i] - b[i];

    float L = pow(l.X(), 2) + pow(l.Y(), 2) + pow(l.Z(), 2) + pow(a, 2) - pow(s, 2);
    float M = 2 * a * l.Z();
    float N = 2 * a * (cos(beta[i]) * l.X() + sin(beta[i]) * l.Y());

    tmp[i] = asin(L / sqrt(pow(M, 2) + pow(N, 2))) - atan2(N, M);

    if(isnan(tmp[i])){
      Serial.print("INVERSE KIN NOT POSSIBLE.\n");
      return false;
    }
  }

  for (int i = 0; i < 6; i++) {
    servo_angles[i] = tmp[i];
  }  
  return true;
}


// desired angle is roll (x), pitch 󰀀, yaw (unchanged, 0)
void move_motors(float x, float y){
  y = 255 - y;
  float ANGLE_LIMIT = 3.5*DEG_TO_RAD; // TUNE 
  float pitch = current_state[0];
  float roll = current_state[1];
  if (x > 162){
    desired_state[0] = min(ANGLE_LIMIT, ANGLE_LIMIT*(sqrt(x-128.0)/sqrt(127.0))); // positive
  }else if (x < 92){
    desired_state[0] = max(-ANGLE_LIMIT, -ANGLE_LIMIT*(sqrt(127.0-x)/sqrt(127.0))); // negative
  }else{
    desired_state[0] = 0;
  }

  if (y > 162){{
    desired_state[1] = min(ANGLE_LIMIT, ANGLE_LIMIT*(sqrt(y-128.0)/sqrt(127.0))); // positive
  }else if (y < 92){
    desired_state[1] = max(-ANGLE_LIMIT, -ANGLE_LIMIT*(sqrt(127.0-y)/sqrt(127.0))); // negative
  }else{
    desired_state[1] = 0;
  }
  float DELTA = 0.8*DEG_TO_RAD;
  float delta_pitch, delta_roll, mag;

  delta_pitch = desired_state[0] - current_state[0];
  delta_roll = desired_state[1] - current_state[1];
  mag = sqrt(pow(delta_pitch, 2) + pow(delta_roll, 2));
  if (desired_state[0] == 0 && desired_state[1] == 0 && DELTA > mag){
    if (!neutral){
      neutral = true;
    }
    for (int i=0; i<6; i++) {
      servo_angles[i] = 0;
    }
    current_state[0] = 0;
    current_state[1] = 0;
  }
  else{
    neutral = false;
    if (mag > DELTA){
      current_state[0] += delta_pitch * (DELTA/mag);
      current_state[1] += delta_roll * (DELTA/mag);
    }else{
      current_state[0] += delta_pitch;
      current_state[1] += delta_roll;
    }
    if (!inverse_kin()) {
      current_state[0] = pitch;
      current_state[1] = roll;
    }
  }
}


// Initialize servo state and set neutral
void setup() {
  Serial.begin(9600);
#if !defined(MIPSEL)
  while (!Serial); 
#endif
  if (Usb.Init() == -1) {
    Serial.print(F("OSC did not start\r\n"));
    while (1); //halt
  }
  Serial.print(F("PS3 USB Library Started\r\n"));
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates
 
  for (int i=0; i<6; i++) {
    servo_angles[i] = 0;
    servo_PWM[i] = SERVOMID[i];
    pwm.setPWM(i, 0, servo_PWM[i]); 
  }
  for (int i=0; i<3; i++) {
    current_state[i]=0;
  }

}

void loop() {
  Usb.Task();
  delay(20);
  if (PS3.PS3Connected) {
  
    move_motors(PS3.getAnalogHat(LeftHatX), PS3.getAnalogHat(LeftHatY));

    for (int i=0; i<6; i++) {
      servo_PWM[i] = max(min(((int)(servo_angles[i] * PULSE_PER_RAD) + SERVOMID[i]), SERVOMAX), SERVOMIN);
      pwm.setPWM(i, 0, servo_PWM[i]);
    }
  }
}