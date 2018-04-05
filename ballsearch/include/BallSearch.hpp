/*
  BallSearch.hpp
  Bruno Dato, Marine Bouchet & Thibaut Aghnatios 

  Header file 
 
 */
#ifndef _BALLSEARCH_
#define _BALLSEARCH_

#include <ros/ros.h>
#include "traitement.hpp"
#include "analyse.hpp"
#include "std_msgs/Bool.h"
#include "std_msgs/Float32.h"
#include "ballsearch/command.h"
#include "sensor_msgs/LaserScan.h"
#include "kobuki_msgs/BumperEvent.h"


class BallSearch 
{
    
private:
    
    // Subscrbers
    ros::Subscriber subscriberScan, subscriberBumper;

    // Publishers
    ros::Publisher publisherBallReference;
    
    // Callback
    void callbackBumper(const kobuki_msgs::BumperEvent& msg);
    void callbackScan(const sensor_msgs::LaserScan& msg);

    bool massiveObstacleTest;
    bool touchedObstacle;
    bool getMin(std::vector<float> tab);
    
public:

    BallSearch(ros::NodeHandle& node);
    ~BallSearch();
    
    // Members
    void sendBallReference(const float linearVelocity, const float angularVelocity, const float distance, const float angle);
    Objet * Recherche_balle(unsigned char* raw, int  width, int height, int couleur);
    void attente(int nsec, int sec);
    bool getMassiveObstacle();
    bool getTouchedObstacle();


};

#endif
