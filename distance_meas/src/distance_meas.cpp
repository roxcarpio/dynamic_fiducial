#include "distance_meas/distancemeasnode.hpp"
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <ros/ros.h>
#include "std_msgs/Float32.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"
#include "std_msgs/Empty.h"

int main(int argc, char* argv[]) {
  ros::init(argc, argv, "distance_meas");  // Name of the node
  DistanceMeasNode Node;

  while(Node.nh.ok())
  {
    ros::spin();
  }
  return 0;

  //int32_t looprate = 1000; //hz
  //ros::Rate loop_rate(looprate);
  //while (Node.nh.ok()) {
    //ros::spinOnce();
    //loop_rate.sleep();
  //}
}
