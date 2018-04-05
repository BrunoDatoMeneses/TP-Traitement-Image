/*
  command_node.cpp
  Bruno Dato and Tristan Klempka

  ROS Node to command the turtlebot in distance and angle.
  This node was at first made for searching a ball but it's also used to seek a marker

 */
#include "ros/ros.h"
#include "TurtleBotCommand.hpp"
#include <cmath>

#define FREQ 10 //10 Hz

int main(int argc, char **argv)
{
    ROS_INFO("Launching command_node ...");
    ros::init(argc, argv, "command_node");
    ros::NodeHandle node;

    // Object which provides the functions to command in speed a turtlebot
    // Here we will only use turning on itself and moving forward
    TurtleBotCommand turtlebotCommand(node);


    // Rate of the node
    ros::Rate r(FREQ);

    while(ros::ok())
    {
        // Launches callbacks which received messages
        ros::spinOnce();

        // Synchroniztation with the rate
        r.sleep();
    }

    return 0;
}
