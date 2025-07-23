#include <math.h>
#include <ros/console.h>
#include <ros/ros.h>
#include <fstream>
#include <iostream>
#include "astarPlanner/astar_searcher.h"

using namespace Eigen;

int main(int argc, char** argv) {
  ROS_INFO("enter main function");
  ros::init(argc, argv, "a_star_node");
  ros::NodeHandle nh;
  ros::Rate loop_rate(100);
  AstarPathFinder* astar_path_finder = new AstarPathFinder();
  while (ros::ok())
  {
  	
    //CubicSplineInterpolator* astar_smoothpath_finder = new CubicSplineInterpolator();
    astar_path_finder->initGridMap();
    astar_path_finder->makePlan();
	  
  	ros::spin();
	
    loop_rate.sleep();
  	
  }
  delete astar_path_finder;
  return 0;
}
