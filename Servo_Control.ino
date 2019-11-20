/* =================================================================================================== 
 *  This code has been provided as an example to help you get started on your project. The objective 
 *  is to provide user input to the Arduino board and have the servo motors actuate. Several lines of 
 *  code are accredited to the Adafruit PWM Servo Driver Library example code. To use the Adafruit 
 *  PWM Servo Shield, be sure to add the Adafruit library to your Arduino IDE. 
 *  (Adafruit example: File menu > Examples > Adafruit PWM Servo Drivers Library > servo)
 *  
 *  Add Adafruit Library: In the Arduino IDE application select: Sketch menu > Include Libraries > 
 *  Manage Libraries. In the Library Manager window, search and install the "Adafruit PWM Servo 
 *  Driver Library".
 *  
 *  NOTE: Depending on your servo motor, the pulse width min/max may be different. Adjust to match 
 *  your servo motor.
 =================================================================================================== */

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <revKin.cpp>

const int SERVOMIN = 175; // 'minimum' pulse length count (out of 4096)
const int SERVOMAX = 475; // 'maximum' pulse length count (out of 4096)
const int SERVOMID = floor(320); // 'mid' pulse length count (out of 4096)
const int SERVOCHG = 50; // 'change' pulse length count

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

String valInput; // Serial input var.
int i=0; // loop index var.
int val[6] = {SERVOMID, SERVOMID, SERVOMID, SERVOMID, SERVOMID, SERVOMID}; // PWM var

void setup() {
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.setTimeout(10); // change default (1000ms) to have faster response time
  Serial.println("Running example: Servo motor actuation using messaging");
  pwm.begin();
  pwm.setPWMFreq(60);  // Analog servos run at ~60 Hz updates

  // set neutral 
  for (i=0; i<6; i++) {
    pwm.setPWM(i+1, 0, 320); // added +1 to match PWM port numbering (pins 1..6 used)
  }
  
}

//switch the motor direction depending on the motor number 
void move_servo(int servonum, int servochg){
  if (servonum%2 ==0) {
   servochg= -servochg; 
  }
  val[servonum] = max(min(val[servonum]+servochg, SERVOMAX), SERVOMIN); //ensure we are in MAX MIN range 

};

const int movement = 10;

void move_up(){
  move_servo(4, -movement);
  move_servo(5, -movement);
  move_servo(1, movement);
  move_servo(2, movement);  
};

void move_down(){
  move_servo(4, movement);
  move_servo(5, movement);
  move_servo(1, -movement);
  move_servo(2, -movement);  
};


void move_left(){
  move_servo(0, -movement);
  move_servo(1, -movement);
  move_servo(5, -movement);
  move_servo(2, movement);  
  move_servo(3, movement);  
  move_servo(4, movement);  
};

void move_right(){
  move_servo(0, movement);
  move_servo(1, movement);
  move_servo(5, movement);
  move_servo(2, -movement);  
  move_servo(3, -movement);  
  move_servo(4, -movement); 
};

void move_neutral(){
  for (i=0; i<6; i++) {
    val[i] = 320; 
  }
};

void loop() {
  if (Serial.available() > 0) {

  valInput = Serial.readString();
  Serial.print("I received: ");
  Serial.print(valInput);
  
  for(auto c: valInput){
    Serial.print('\n');
    Serial.print(c);
    switch(c){
      case 'w':
        move_up();
        break;
      case 's':
        move_down();
        break;
      case 'a':
        move_left();
        break;
      case 'd':
        move_right();
        break;
      case 'm':
        move_neutral();
        break;
        
      default:
        break;
      }
  }
    
  for (i=0; i<6; i++) {
    pwm.setPWM(i+1, 0, val[i]); // added +1 to match PWM port numbering (pins 1..6 used)
  }
  }
}

