
#include <ros.h>

#include <Arduino.h>
#include <Servo.h>

#define NEAR_SERVO_PIN 3
#define FAR_SERVO_PIN 2

ros::NodeHandle nh;

Servo nearServo;
Servo farServo;


int i = 0;
int servoPos = 0;


void setup()
{
    // setup node
    nh.initNode();
    nh.getHardware()->setBaud(57600);
    
    // setup servos
    nearServo.attach(NEAR_SERVO_PIN);
    farServo.attach(FAR_SERVO_PIN);

}

void loop()
{
    // move the servos around
    servoPos = i % 180 + 20;
    i++;

    nearServo.write(servoPos);
    farServo.write(servoPos);

    nh.spinOnce();
    delay(1000);
}


