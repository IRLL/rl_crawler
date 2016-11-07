#include <ros.h>
#include <rl_crawler/command.h>
#include <rl_crawler/distance.h>


#include <Arduino.h>
#include <Servo.h>
// TODO include Arduino servo.h somehow


#define NEAR_SERVO_PIN 1
#define FAR_SERVO_PIN 2

#define ULTRASONIC_PIN 7


ros::NodeHandle nh;
rl_crawler::command command;
rl_crawler::distance distance;

Servo nearServo;
Servo farServo;

// Move the servos into the places given by the command message
void callback(const rl_crawler::command& command_msg)
{
	// Decode the command message
	int nearServoPos = command_msg.nearServoPos;
	int farServoPos = command_msg.farServoPos;

	// move the servos into the desired places
	arm1.write(nearServoPos);
	arm2.write(farServoPos);
}

ros::Publisher distance_pub("distance", &state);
ros::Subscriber<rl_crawler::action> command_sub("command", &callback);

void setup()
{
	// setup node
    nh.initNode();
    nh.advertise(distance_pub);
    nh.subscribe(command_sub);

	// setup servos
	arm1.attach(NEAR_SERVO_PIN);
	arm2.attach(FAR_SERVO_PIN);

	// setup arduino serial
	Serial.begin(10000); // setup communications with computer at a 10 kb/sec bitrate
	delay(1000);

}

void loop()
{
    nh.spinOnce();
    delay(100);

	// get new distance measurement and publish to distance channel
}
