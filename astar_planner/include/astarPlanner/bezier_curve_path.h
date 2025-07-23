#pragma once

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <iostream>

using namespace std;

class SubscribeAndPublishPath
{
    public:
        SubscribeAndPublishPath();
        ~SubscribeAndPublishPath()=default;
        void createCurve(const nav_msgs::Path &originPoint,nav_msgs::Path &output);
        void deal_path(const nav_msgs::Path& input,nav_msgs::Path& output,int distance);
     
    private:
        float bezier3func(float uu,float* controlPoint);
        float bezier2func(float uu,float* controlPoint);

    private:
        ros::NodeHandle n_; 
        ros::Publisher pub_;
        ros::Publisher pub1_;
        ros::Subscriber sub_;
        ros::Subscriber sub1_;
};