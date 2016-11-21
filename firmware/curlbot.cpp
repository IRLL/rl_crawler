


#include <ros.h>
#include <rl_crawler/command.h>
#include <rl_crawler/distance.h>


#include <Arduino.h>
#include <Servo.h>
#include <libraries/NewPing.h>


#define NEAR_SERVO_PIN 1
#define FAR_SERVO_PIN 2

#define ULTRASONIC_TRIGGER_PIN 7
#define ULTRASONIC_ECHO_PIN 6


ros::NodeHandle nh;
rl_crawler::command command;
rl_crawler::distance distance;

Servo nearServo;
Servo farServo;

NewPing rangeFinder;


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

ros::Publisher distance_pub("distance", &distance);
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

	// setup range finder
	rangeFinder = NewPing(ULTRASONIC_TRIGGER_PIN, ULTRASONIC_ECHO_PIN);

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
	distance distanceResult = distance(cmDistance);
	distance_pub.publish(distanceResult);

}
