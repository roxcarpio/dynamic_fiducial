#ifndef DISTANCEMEASNODE_HPP
#define DISTANCEMEASNODE_HPP

/// Some parts of this package were modified from:
/// https://github.com/omwdunkley/ollieRosTools.git
///
#include <ros/timer.h>
#include <ros/ros.h>
#include <ros/duration.h>

#include <dynamic_reconfigure/server.h>
#include <distance_meas/dynamic_param_configConfig.h>

#include <std_msgs/Float64.h>

#include <image_transport/image_transport.h>
#include <camera_info_manager/camera_info_manager.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <map>
#include <sensor_msgs/BatteryState.h>

class DistanceMeasNode {
 public:
    DistanceMeasNode();

    ros::NodeHandle nh;
    void dynamicReconfigureCb(
        distance_meas::dynamic_param_configConfig& config, uint32_t level);

    void change_distance();
    void change_rate( );
    void send_timer_callback(const ros::TimerEvent& event);

private:
    // dynamic reconfigure server
    dynamic_reconfigure::Server<distance_meas::dynamic_param_configConfig> server_;

    ros::Publisher dist_pub;

    // Parameters (dynamic reconfigure ROS)
    double total_distance;
    double delta_distance;
    bool random_option;
    bool saved_random_option;

    double send_seconds;
    double saved_send_seconds;

    double read_distance;

    ros::Duration send_rate;
    ros::Timer send_distance;

    unsigned int ticks;

    unsigned int init_iterations;
    unsigned int init_rate;
    int init_mode;

    unsigned int decision_mode;

    int combine;

    double period_variation;
    bool random_period;



};

#endif  // DISTANCEMEASNODE_HPP
