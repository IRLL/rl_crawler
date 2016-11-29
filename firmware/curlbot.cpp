/*

	This file (curlbot.cpp) is the firmware that will run on our curlbot and interact with the agent.

*/


#include <ros.h>
#include <rl_crawler/command.h>
#include <rl_crawler/distance.h>


#include <Arduino.h>
#include <Servo.h>

#include "NewPing.h"


#define NEAR_SERVO_PIN 3
#define FAR_SERVO_PIN 2

#define ULTRASONIC_TRIGGER_PIN 12
#define ULTRASONIC_ECHO_PIN 13


ros::NodeHandle nh;
rl_crawler::command command;
rl_crawler::distance distance;

Servo nearServo;
Servo farServo;

// setup range finder
NewPing rangeFinder(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);


// Move the servos into the places given by the command message
void callback(const rl_crawler::command& command_msg)
{
	// Decode the command message
	int nearServoPos = command_msg.nearServoPos;
	int farServoPos = command_msg.farServoPos;

	// move the servos into the desired places
	nearServo.write(nearServoPos);
	farServo.write(farServoPos);
}

ros::Publisher distance_pub("distance", &distance);
ros::Subscriber<rl_crawler::command> command_sub("command", &callback);

void setup()
{
	// setup node
    nh.initNode();
    nh.advertise(distance_pub);
    nh.subscribe(command_sub);

	// setup servos
	nearServo.attach(NEAR_SERVO_PIN);
	farServo.attach(FAR_SERVO_PIN);


	// setup arduino serial
	Serial.begin(10000); // setup communications with computer at a 10 kb/sec bitrate
	delay(1000);

}

void loop()
{
    nh.spinOnce();
    delay(200);

	// get new distance measurement and publish to distance channel
	int millisecondsReturnTime = rangeFinder.ping_median(5); // ping 5 times and return the median
	double cmDistance = rangeFinder.convert_cm(millisecondsReturnTime);
	distance.wallDistance = cmDistance;
	distance_pub.publish(&distance);

}
