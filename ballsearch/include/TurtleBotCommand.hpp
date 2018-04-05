/*
  TurtleBotCommand.hpp
  Bruno Dato & Tristant Klempka

  Header file 
 
 */
#ifndef _TURTLEBOTCOMMAND_
#define _TURTLEBOTCOMMAND_

#include <ros/ros.h>
#include <string>
#include <vector>
#include "geometry_msgs/Twist.h"
#include "ballsearch/command.h"
#include "std_msgs/Bool.h"


const float ROBOT_MAX_LINEAR_VELOCITY = 0.25f;
const float ROBOT_MAX_ANGULAR_VELOCITY = 3.1416f;

class TurtleBotCommand 
{    
private:

    
    //Publishers
    ros::Publisher publisherMobileBaseCommandsVelocity;
    
    //Subscibers
    ros::Subscriber subCommandReceived;
    
    //Messages
    geometry_msgs::Twist mobileBaseCommandsVelocity;
    ballsearch::command commandAsked;
    
    //CallBacks
    void callBackCommandReceived(const ballsearch::command& msg);
    
    
    // Speed command
    void setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ);
        
public:

    TurtleBotCommand(ros::NodeHandle& node);
    ~TurtleBotCommand();
    
    //Motions
    void stop();
    void moveAndTurn(const float linearVelocity, const float angularVelocity);
    
};

#endif
