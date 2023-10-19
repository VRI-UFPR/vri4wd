w/*
Date: 23.05.2023
Project: VRI 4WD Robot ROS
Design: Carlos Eduardo Magrin/Eduardo Todt

Especifications:
- 4WD Platform
- Power Module DF Robot
- Board Sparkfun Atmega328p + I/O Expansion Shield V7 DF Robot
- DC Motor Driver DF Robot 2x15A
- (4) 75:1 Metal Gearmotor 6V with 48 CPR Encoder

NEW Speed adjust with code:
How to Control a Robot’s Velocity Remotely Using ROS
https://automaticaddison.com/how-to-control-a-robots-velocity-remotely-using-ros/
*/


#include "ros.h"
#include <ros/time.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>

//Motor DC 4WD
#define M1_PWM    6 //Right
#define M2_PWM    10
#define M1_EN     12     
#define M2_EN     11
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 2
#define ENC_IN_RIGHT_A 3
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 19
#define ENC_IN_RIGHT_B 18

//Global variables
int locomotion = 0, i = 30, speedMotor = 0, speedMotorL = 0, speedMotorR = 0, speedMotorRamp = 0; 

// True = Forward; False = Reverse
boolean Direction_left = true;
boolean Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;

// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count; //standard
ros::Publisher rightPub("right_encoder", &right_wheel_tick_count); //standard

std_msgs::Int16 left_wheel_tick_count; //standard
ros::Publisher leftPub("left_encoder", &left_wheel_tick_count); //standard


// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K_P = 278;
// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 75; //standard 52
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;
double lastCmdVelRight = 0;
double lastCmdVelLeft = 0;
// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;
double pwmSpeed = 0;
// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 80;
// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;
// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;

// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 600; //ex 620
// Wheel radius in meters
const double WHEEL_RADIUS = 0.12; //ex 0.033
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.255; //ex 0.17
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 3100; // Originally 2880

// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 100; // about 0.1 m/s
const int PWM_MAX = 120; // about 0.172 m/s
// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 

ros::NodeHandle  nh;


// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if (val == LOW) {
    Direction_right = false; // Reverse
  }
  else {
    Direction_right = true; // Forward
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if (val == LOW) {
    Direction_left = true; // Reverse
  }
  else {
    Direction_left = false; // Forward
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}

 
/////////////////////// Motor Controller Functions ////////////////////////////
 
// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is rpublished on the /left_ticks topic. 
void calc_vel_left_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velLeftWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;
 
  // Update the timestamp
  prevTime = (millis()/1000);
 
}
 
// Calculate the right wheel linear velocity in m/s every time a 
// tick count message is published on the /right_ticks topic. 
void calc_vel_right_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;
 
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per second
  velRightWheel = numOfTicks/TICKS_PER_METER/((millis()/1000)-prevTime);
 
  prevRightCount = right_wheel_tick_count.data;
   
  prevTime = (millis()/1000);
 
}


// Take the velocity command as input and calculate the PWM values.
void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
  // Record timestamp of last velocity command received
  lastCmdVelReceived = (millis()/1000);
  
  lastCmdVelRight = cmdVel.linear.x;
  lastCmdVelLeft = cmdVel.angular.z; 
  
   // Calculate the PWM value given the desired velocity 
   

     
      
    if (cmdVel.linear.x != 0.0 || cmdVel.angular.z != 0.0) { 
    if (cmdVel.linear.x > 0.0) { //front
      pwmSpeed = ((K_P * ((cmdVel.linear.x)) + b));
      speedMotorL = pwmSpeed;
      speedMotorR = pwmSpeed;
      locomotion = 1;
      
    }
    if (cmdVel.linear.x < 0.0) { //rear
      pwmSpeed = ((K_P * ((-(cmdVel.linear.x))) + b));
      speedMotorL = pwmSpeed;
      speedMotorR = pwmSpeed;
      locomotion = 4;
      
    }
 

      // Turn left
    if (cmdVel.angular.z > 0.0) { //left
      pwmSpeed = ((K_P * ((cmdVel.angular.z)/10) + b));
      speedMotorL = pwmSpeed;
      speedMotorR = pwmSpeed;
      locomotion = 2;
      
    }
    if (cmdVel.angular.z < 0.0) { //right
      pwmSpeed = ((K_P * ((-(cmdVel.angular.z))/10) + b));
      speedMotorL = pwmSpeed;
      speedMotorR = pwmSpeed;
      locomotion = 3;
      
    }
   
    }  
  
  else {      
      speedMotorL = 0;
      speedMotorR = 0;
      // Stop 
      locomotion = 0;
      
    } 

 

  char log_msg[60];
  sprintf(log_msg, "PWM L: %3d PWM R: %3d | %2d", (int)(speedMotorL), (int)(speedMotorR), locomotion);

  nh.loginfo(log_msg);    

}

  // Set up ROS subscriber to the velocity command
ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );

void setup(){
  
   
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
  
  // Every time the pin goes high, this is a tick
  attachInterrupt(0 , left_wheel_tick, RISING); //INT0
  attachInterrupt(1 , right_wheel_tick, RISING); //INT1
  
  
  // Motor control pins are outputs
  pinMode(M1_PWM, OUTPUT);  
  pinMode(M2_PWM, OUTPUT); 
  pinMode(M1_EN, OUTPUT);
  pinMode(M2_EN, OUTPUT); 
  
  // Turn off motors - Initial state
  digitalWrite(M1_EN, LOW);
  digitalWrite(M2_EN, LOW);
  
  // Set the motor speed
  analogWrite(M1_PWM, 0); 
  analogWrite(M2_PWM, 0);
  

  nh.initNode();
  nh.getHardware()->setBaud(57600);
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.subscribe(subCmdVel);
   
}

void loop(){
  
  nh.spinOnce();
  
 
  // Record the time
  currentMillis = millis();
  
  
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;

    // Publish tick counts to topics
    leftPub.publish( &left_wheel_tick_count); //standard
    rightPub.publish( &right_wheel_tick_count ); //standard
 
    // Calculate the velocity of the right and left wheels
    //calc_vel_right_wheel();
    //calc_vel_left_wheel();
     
  }
  
  //Controle utilizando /cmd_vel rodando o pacote cbs_level0_locomotion
  
  // Stop the car if there are no cmd_vel messages
  //if((millis()/1000) - lastCmdVelReceived > 1) {
  //  pwmLeftReq = 0;
  //  pwmRightReq = 0;
  //}
  
  //set_pwm_values();  
  
  
   // Set the direction of the motors
   
int speed_constant = 90;

switch (locomotion){
case(1):    //anda frente
            //nh.loginfo("Front");
            digitalWrite(M1_EN, LOW);  
            digitalWrite(M2_EN, HIGH);

            analogWrite(M1_PWM, speedMotorR+10); 
            analogWrite(M2_PWM, speedMotorL);
            break;

case (2):   //anda esquerda
            //nh.loginfo("Turn left");
            digitalWrite(M1_EN, LOW);  
            digitalWrite(M2_EN, LOW); 

           
            analogWrite(M1_PWM, speedMotorR+20); 
            analogWrite(M2_PWM, speedMotorL+10);
            break;
    
case(3):    //anda direita
            //nh.loginfo("Turn right");
            digitalWrite(M1_EN, HIGH);  
            digitalWrite(M2_EN, HIGH);
            //speedMotorL = 100;  
            //speedMotorR = 100; 
    
            analogWrite(M1_PWM, speedMotorR+20); 
            analogWrite(M2_PWM, speedMotorL+10);;
            break;
 
case(4):     //retrocede
            //nh.loginfo("Rear");            
            digitalWrite(M1_EN, HIGH);  
            digitalWrite(M2_EN, LOW);
    
            analogWrite(M1_PWM, 90); 
            analogWrite(M2_PWM, 80);
            break;    

default:
    analogWrite(M1_PWM, 0); 
    analogWrite(M2_PWM, 0);
    break;
}
 
}
