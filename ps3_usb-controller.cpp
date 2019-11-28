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

const float SERVOMIN = 150; // 'minimum' pulse length count (out of 4096)
const float SERVOMAX = 450; // 'maximum' pulse length count (out of 4096)
const float SERVOMID[6] = {327, 339, 331, 314, 339, 311}; // 'mid' pulse length count (out of 4096)
const float PULSE_PER_RAD = 138.8;

int servo_PWM[6];
float servo_angles[6];
float current_state[3];
float desired_state[3];

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

Rotation get_rot_matrix(float desired_angles[]){
  Rotation rot_matrix;
  rot_matrix.RotateX(desired_angles[0]);
  rot_matrix.RotateY(desired_angles[1]);
  rot_matrix.RotateZ(desired_angles[2]);
  return rot_matrix;
}


// T and desired_angles are input, p, b, beta, s and a are constants, servo_angles is alpha
bool inverse_kin(Point pos, Point p[], Point b[], float beta[], float s, float a) {

  float tmp[6];

  Rotation rot_matrix = get_rot_matrix(current_state);

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
      return false;
    }
  }

  for (int i = 0; i < 6; i++) {
    servo_angles[i] = tmp[i];
  }  
  return true;
}

// Helper function defines constants
bool inverse_kin_helper(){
  float beta[6] = {0, 180, 240, 60, 120, 300}; // servo arm angles
  for(int i=0; i<6; i++) {
    // beta[i] /= DEG_TO_RAD;
    beta[i] *= RAD_TO_DEG;
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
  return inverse_kin(pos, p, b, beta, s, a);
}

// desired angle is roll (x), pitch 󰀀, yaw (unchanged, 0)
void move_motors(float x, float y){
  y = 255 - y;
  float DELTA = 3*DEG_TO_RAD; // TUNE 
  float DELTA_LIMIT = 2*(DEG_TO_RAD); // TUNE
  float pitch = current_state[0];
  float roll = current_state[1];
  if (x >= 128){
    current_state[0] += min(DELTA_LIMIT, DELTA*(sqrt(x-128.0)/sqrt(127.0))); // positive
  }else{
    current_state[0] += max(-DELTA_LIMIT, -DELTA*(sqrt(127.0-x)/sqrt(127.0))); // negative
  } 
  if (y >= 128){
    current_state[1] += min(DELTA_LIMIT, DELTA*(sqrt(y-128.0)/sqrt(127.0))); // positive
  }else{
     current_state[1] += max(-DELTA_LIMIT, -DELTA*(sqrt(127.0-y)/sqrt(127.0))); // negative
  }
  float ANGLE_LIMIT = 6*DEG_TO_RAD; // TUNE
  current_state[0] = max(min(current_state[0], ANGLE_LIMIT), -ANGLE_LIMIT);
  current_state[1] = max(min(current_state[1], ANGLE_LIMIT), -ANGLE_LIMIT);
  current_state[2] = 0;
  Serial.print("Desired State: ");
  Serial.print(current_state[0]*RAD_TO_DEG);
  Serial.print(", ");
  Serial.print(current_state[1]*RAD_TO_DEG);
  // Serial.print(" FROM ");
  // Serial.print(x);
  // Serial.print(", ");
  // Serial.print(y);
  Serial.print("\n");
  if (!inverse_kin_helper()){
    current_state[0] = pitch;
    current_state[1] = roll;
  }
}


void move_motors2(float x, float y){
  y = 255 - y;
  float DELTA = 0.001F*DEG_TO_RAD; // TUNE 
  float DELTA_LIMIT = 10*DEG_TO_RAD; // TUNE

  int movement = 0;
  if (x >= 128){
    movement = min(DELTA_LIMIT, DELTA*(sqrt(x-128.0)/sqrt(127.0))); // positive
    move_up(movement);
  }else{
    movement = min(DELTA_LIMIT, DELTA*(sqrt(127.0-x)/sqrt(127.0))); // negative
    move_down(movement);
  } 
  if (y >= 128){
    movement = min(DELTA_LIMIT, DELTA*(sqrt(y-128.0)/sqrt(127.0))); // positive
    move_left(movement);
  }else{
    movement = min(DELTA_LIMIT, DELTA*(sqrt(127.0-y)/sqrt(127.0))); // negative
    move_right(movement);
  }
}


void move_motors3(float x, float y){

  // forward 0.8 4.2
  // back 2 -3.5
  // right 3.2 2.4
  // left -3 0.8

  y = 255 - y;
  float DELTA = 3*DEG_TO_RAD; // TUNE 

  float pitch = current_state[0];
  float roll = current_state[1];
  float desired_pich = 0;
  float desired_roll = 0;

  if (x >= 128){
    x = sqrt(sqrt(x-128.0))/sqrt(sqrt(127.0));// right
  }else{
    x = -sqrt(sqrt(127.0-x))/sqrt(sqrt(127.0));// left
  } 
  if (y >= 128){
    y = sqrt(sqrt(y-128.0))/sqrt(sqrt(127.0));// forward
  }else{
    y = -sqrt(sqrt(127.0-y))/sqrt(sqrt(127.0));// backward
  }
  
  if(abs(x) > abs(y)){
    if (x > 0){
      current_state[0] = 3.7 * DEG_TO_RAD; // * x;
      current_state[1] = 0.7 * DEG_TO_RAD; // * x;
    }
    else{
      x *= -1;
      current_state[0] = -3 * DEG_TO_RAD; // * x;
      current_state[1] = -0.5 * DEG_TO_RAD; // * x;
    }
  } else {
    if (y > 0){
      current_state[0] = -0.2 * DEG_TO_RAD; // * y;
      current_state[1] = 3 * DEG_TO_RAD; // * y;
    }
    else{
      y *= -1;
      current_state[0] = 3.5 * DEG_TO_RAD; // * y;
      current_state[1] = -3 * DEG_TO_RAD; // * y;
    }
  }

  Serial.print("Desired State: ");
  Serial.print(current_state[0]*RAD_TO_DEG);
  Serial.print(", ");
  Serial.print(current_state[1]*RAD_TO_DEG);
  Serial.print(" FROM ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print("\n");
  if (!inverse_kin_helper()) {
    current_state[0] = pitch;
    current_state[1] = roll;
  }
}

void move_motors4(float x, float y){

  // forward 0.8 4.2
  // back 2 -3.5
  // right 3.2 2.4
  // left -3 0.8

  y = 255 - y;

  float pitch = current_state[0];
  float roll = current_state[1];

  if (x > 162){
    x = 1;
  }else if (x < 92){
    x = -1;
  }else{
    x = 0;
  }
  if (y > 162){
    y = 1;
  }else if (y < 92){
    y = -1;
  }else{
    y = 0;
  }

  if(x==1 && y==1){ // right and forward
    desired_state[0] = 2;
    desired_state[1] = 3.3;
  }else if(x==1 && y==-1){ // right and back
    desired_state[0] = 2.6;
    desired_state[1] = -2.6;
  }else if(x==1 && y==0){ // right
    desired_state[0] = 3.2;
    desired_state[1] = 2.4;
  }else if(x==-1 && y==1){ // left and forward
    desired_state[0] = -2.1;
    desired_state[1] = 2.5;
  }else if(x==-1 && y==-1){ // left and back
    desired_state[0] = -1;
    desired_state[1] = -2.7;
  }else if(x==-1 && y==0){ // left
    desired_state[0] = -3;
    desired_state[1] = 0.8;
  }else if(x==0 && y==1){ // forward
    desired_state[0] = 0.8;
    desired_state[1] = 4.2;
  }else if(x==0 && y==-1){ // back
    desired_state[0] = 2;
    desired_state[1] = -3.5;
  }else{
    Serial.print("Error");
    Serial.print("\n");
  }
  desired_state[0] *= DEG_TO_RAD;
  desired_state[1] *= DEG_TO_RAD;

  Serial.print("Desired State: ");
  Serial.print(desired_state[0]*RAD_TO_DEG);
  Serial.print(", ");
  Serial.print(desired_state[1]*RAD_TO_DEG);
  Serial.print(" FROM ");
  Serial.print(x);
  Serial.print(", ");
  Serial.print(y);
  Serial.print("\n");

  float DELTA = 0.5 * DEG_TO_RAD;
  float delta_pitch, delta_roll, mag;

  delta_pitch = desired_state[0] - current_state[0];
  delta_roll = desired_state[1] - current_state[1];
  mag = sqrt(pow(delta_pitch, 2) + pow(delta_roll, 2));
  if (mag > DELTA){
    current_state[0] += delta_pitch * (DELTA/mag);
    current_state[1] += delta_roll * (DELTA/mag);
  }else{
    current_state[0] += delta_pitch * DEG_TO_RAD;
    current_state[1] += delta_roll * DEG_TO_RAD;
  }

  if (!inverse_kin_helper()) {
    current_state[0] = pitch;
    current_state[1] = roll;
  }
}

// Initialize servo state and set neutral
void setup() {
  Serial.begin(9600);
#if !defined(MIPSEL)
  while (!Serial); 
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
    pwm.setPWM(i, 0, servo_PWM[i]); 
  }
  for (int i=0; i<3; i++) {
    current_state[i]=0;
  }

}


// Print only when something changes
bool flag = true;
bool neutral = true;
int counter = 0;
float t1,t2;

void loop() {
  delay(100);
  t1 = millis();
  Usb.Task();
//  counter += 1;
//  if (counter != 13){
//    for (int i=0; i<6; i++) {
//        pwm.setPWM(i, 0, servo_PWM[i]);
//        delay(1);
//      }
//  }
  if (PS3.PS3Connected) {
    counter = 0;
    // if (PS3.getAnalogHat(LeftHatX) > 162 || PS3.getAnalogHat(LeftHatX) < 92 || PS3.getAnalogHat(LeftHatY) > 162 || PS3.getAnalogHat(LeftHatY) < 92) {
      // Serial.print(F("\r\nLeftHatX: "));
      // Serial.print(PS3.getAnalogHat(LeftHatX));
      // Serial.print(F("\tLeftHatY: "));
      // Serial.print(PS3.getAnalogHat(LeftHatY));
      // Serial.print("\n");
    
      move_motors4(PS3.getAnalogHat(LeftHatX), PS3.getAnalogHat(LeftHatY));
      //move_motors2(PS3.getAnalogHat(LeftHatX), PS3.getAnalogHat(LeftHatY));

      flag = true;
      neutral = false;
      for (int i=0; i<6; i++) {
        servo_PWM[i] = max(min(((int)(servo_angles[i] * PULSE_PER_RAD) + SERVOMID[i]), SERVOMAX), SERVOMIN);
        pwm.setPWM(i, 0, servo_PWM[i]);
      }
    // }
    //else if (PS3.getButtonClick(PS) && !neutral){
    // else if(!neutral){
    //   // set neutral
    //   neutral = true;
    //   flag = true;
    //   for (int i=0; i<6; i++) {
    //     servo_angles[i] = 0;
    //     servo_PWM[i] = SERVOMID[i];
    //     pwm.setPWM(i, 0, servo_PWM[i]); // added +1 to match PWM port numbering (pins 1..6 used)
    //   }
    //   for (int i=0; i<3; i++) {
    //     current_state[i]=0;
    //   }
    // }
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
    //t2 = millis();
    //Serial.print(t2-t1);
    //Serial.print("\n");
}


//switch the motor direction depending on the motor number 
void move_servo(int servonum, int servochg){
  if (servonum%2 ==0) {
   servochg= -servochg; 
  }
  servo_angles[servonum] = max(min(servo_angles[servonum]+servochg, SERVOMAX), SERVOMIN); //ensure we are in MAX MIN range 

};

void move_up(int movement){
  move_servo(4, -movement);
  move_servo(5, -movement);
  move_servo(1, movement);
  move_servo(2, movement);  
};

void move_down(int movement){
  move_servo(4, movement);
  move_servo(5, movement);
  move_servo(1, -movement);
  move_servo(2, -movement);  
};


void move_left(int movement){
  move_servo(0, -movement);
  move_servo(1, -movement);
  move_servo(5, -movement);
  move_servo(2, movement);  
  move_servo(3, movement);  
  move_servo(4, movement);  
};

void move_right(int movement){
  move_servo(0, movement);
  move_servo(1, movement);
  move_servo(5, movement);
  move_servo(2, -movement);  
  move_servo(3, -movement);  
  move_servo(4, -movement); 
};

// void move_neutral(){
//   for (i=0; i<6; i++) {
//     val[i] = 320; 
//   }
// };

// void loop() {
//   if (Serial.available() > 0) {

//   valInput = Serial.readString();
//   Serial.print("I received: ");
//   Serial.print(valInput);
  
//   for(auto c: valInput){
//     Serial.print('\n');
//     Serial.print(c);
//     switch(c){
//       case 'w':
//         move_up();
//         break;
//       case 's':
//         move_down();
//         break;
//       case 'a':
//         move_left();
//         break;
//       case 'd':
//         move_right();
//         break;
//       case 'm':
//         move_neutral();
//         break;

      
        
//       default:
//         break;
//       }
//   }



    
//   for (i=0; i<6; i++) {
//     pwm.setPWM(i, 0, val[i]); // added +1 to match PWM port numbering (pins 1..6 used)
//   }
//   }
// }