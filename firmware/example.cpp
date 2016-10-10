#include <ros.h>
#include <rl_crawler/action.h>
#include <rl_crawler/state.h>
#include <Arduino.h>


ros::NodeHandle nh;
rl_crawler::action action;
rl_crawler::state state;

void callback(const rl_crawler::action& action_msg)
{
    //do something!
}

ros::Publisher state_pub("state", &state);
ros::Subscriber<rl_crawler::action> action_sub("action", &callback);

void setup()
{
    nh.initNode();
    nh.advertise(state_pub);
    nh.subscribe(action_sub);
}

void loop()
{

    nh.spinOnce();
    delay(100);
}
