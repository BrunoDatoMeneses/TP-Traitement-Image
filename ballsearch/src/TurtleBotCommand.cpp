/*
  TurtleBot_command.cpp
  Bruno Dato and Tristan Klempka

  Class to command the turtlebot in speed and distance.

 */
#include "TurtleBotCommand.hpp"

// Constructor
TurtleBotCommand::TurtleBotCommand(ros::NodeHandle& node):
    //subscribers
    subCommandReceived(node.subscribe("/nav/open_loop_command", 1, &TurtleBotCommand::callBackCommandReceived,this)),

    //publishers
    publisherMobileBaseCommandsVelocity(node.advertise<geometry_msgs::Twist>("/mobile_base/commands/velocity", 1))
{
    // Initialization of the command custom message
    commandAsked.linearVelocity = 0;
    commandAsked.angularVelocity = 0;
    commandAsked.distance = 0;
    commandAsked.angle = 0;

    // Initialy the robot doesn't move
    stop();
}

// Destructor
TurtleBotCommand::~TurtleBotCommand()
{}


//Callbacks
// Updates the custom command message when a command is asked
// Sets the boolean values to start a mouvement on the FSM of the command_node
void TurtleBotCommand::callBackCommandReceived(const ballsearch::command& msg)
{
    ROS_INFO("Command received...");
    commandAsked = msg;

    if(commandAsked.distance>0)
    {
        TurtleBotCommand::moveAndTurn(3*commandAsked.linearVelocity*commandAsked.distance, commandAsked.angularVelocity*commandAsked.angle*3);
    }
    else if(commandAsked.distance<0)
    {
        TurtleBotCommand::moveAndTurn(-3*commandAsked.linearVelocity*commandAsked.distance, commandAsked.angularVelocity*commandAsked.angle*3);
    }
    else
    {
        TurtleBotCommand::moveAndTurn(0, commandAsked.angularVelocity*commandAsked.angle*3);
    }



}

// Sends linear and angular speeds to the /mobile_base/commands/velocity topic
void TurtleBotCommand::setMobileBaseCommandsVelocity(const float linearX, const float linearY, const float linearZ, const float angularX, const float angularY, const float angularZ)
{
    if(linearX > ROBOT_MAX_LINEAR_VELOCITY) mobileBaseCommandsVelocity.linear.x=ROBOT_MAX_LINEAR_VELOCITY;
    else mobileBaseCommandsVelocity.linear.x=linearX;
    mobileBaseCommandsVelocity.linear.y=linearY;
    mobileBaseCommandsVelocity.linear.z=linearZ;
    mobileBaseCommandsVelocity.angular.x=angularX;
    mobileBaseCommandsVelocity.angular.y=angularY;
    if(angularZ > ROBOT_MAX_ANGULAR_VELOCITY) mobileBaseCommandsVelocity.angular.z=ROBOT_MAX_ANGULAR_VELOCITY;
    else mobileBaseCommandsVelocity.angular.z=angularZ;
    // publish the message on the associated topic
    publisherMobileBaseCommandsVelocity.publish(mobileBaseCommandsVelocity);
}


//Motions
// Stops the robot
void TurtleBotCommand::stop()
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(0, 0, 0, 0, 0, 0);
    //busy.data = false;
    //pubCommandState.publish(busy);
}


// Makes the robot move forward and turn at the same time at linear and angular defined seeds
void TurtleBotCommand::moveAndTurn(const float linearVelocity, const float angularVelocity)
{
    TurtleBotCommand::setMobileBaseCommandsVelocity(linearVelocity, 0, 0, 0, 0, angularVelocity);
}


