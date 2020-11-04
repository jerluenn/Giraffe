
/*
 * rosserial Servo Control Example
 *
 * This sketch demonstrates the control of hobby R/C servos
 * using ROS and the arduiono
 * 
 * For the full tutorial write up, visit
 * www.ros.org/wiki/rosserial_arduino_demos
 *
 * For more information on the Arduino Servo Library
 * Checkout :
 * http://www.arduino.cc/en/Reference/Servo
 */

// Use the following line if you have a Leonardo or MKR1000


#define USE_USBCON
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif

#include "BasicStepperDriver.h"
#include <ros.h>
#include <std_msgs/Int16.h>

// Motor steps per revolution. Most steppers are 200 steps or 1.8 degrees/step
#define MOTOR_STEPS 1605 //1605 steps gives one revolution/360 degrees/2pi for the "rotate" function
#define RPM 45

// Since microstepping is set externally, make sure this matches the selected mode
// If it doesn't, the motor will move at a different RPM than chosen
// 1=full step, 2=half step etc.
#define MICROSTEPS 1  

// All the wires needed for full functionality
#define DIR 8
#define STEP 9
#define ENABLE 7

#define DIR2 10
#define STEP2 11

//Uncomment line to use enable/disable functionality
//#define SLEEP 13

// 2-wire basic config, microstepping is hardwired on the driver
BasicStepperDriver stepper1(MOTOR_STEPS, DIR, STEP);
BasicStepperDriver stepper2(MOTOR_STEPS, DIR2, STEP2);

//Uncomment line to use enable/disable functionality
//BasicStepperDriver stepper(MOTOR_STEPS, DIR, STEP, SLEEP);

ros::NodeHandle  nh;

void stepper_cb1( const std_msgs::Int16& cmd_msg){
//  stepper1.move(cmd_msg.data*MOTOR_STEPS*MICROSTEPS);
  stepper1.rotate(cmd_msg.data);
  digitalWrite(13, HIGH-digitalRead(13));  //toggle led  
}

void stepper_cb2( const std_msgs::Int16& cmd_msg){
  stepper2.rotate(cmd_msg.data);
//  stepper2.move(cmd_msg.data*MOTOR_STEPS*MICROSTEPS);
}


ros::Subscriber<std_msgs::Int16> sub1("stepper1", stepper_cb1);
ros::Subscriber<std_msgs::Int16> sub2("stepper2", stepper_cb2);

void setup(){
  pinMode(13, OUTPUT);
//  pinMode(ENABLE, OUTPUT);
//  digitalWrite(ENABLE, LOW);
  nh.initNode();
  nh.subscribe(sub1);
  nh.subscribe(sub2);
  
  stepper1.begin(RPM, MICROSTEPS);
  stepper2.begin(RPM, MICROSTEPS);
}

void loop(){
  nh.spinOnce();
  delay(1);
}
