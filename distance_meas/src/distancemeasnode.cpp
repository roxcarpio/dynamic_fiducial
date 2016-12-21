#include "distance_meas/distancemeasnode.hpp"

#include <stdlib.h>

DistanceMeasNode::DistanceMeasNode()
    : read_distance(0.0),
      total_distance(0.0),
      delta_distance (0.0),
      random_option(false),
      send_rate( 0 ),
      init_mode(0),
      ticks ( 0 )
{
    ros::NodeHandle params("~");
    // Topic Parameters
    dist_pub = nh.advertise<sensor_msgs::BatteryState>("/distance_meas/distance",100);

    // Dynamic parameter reconfigure

    dynamic_reconfigure::Server<distance_meas::dynamic_param_configConfig>::CallbackType f;
    f = boost::bind(&DistanceMeasNode::dynamicReconfigureCb, this, _1, _2);
    server_.setCallback(f);
}

void DistanceMeasNode::dynamicReconfigureCb(
    distance_meas::dynamic_param_configConfig& config, uint32_t level)
{
    ROS_DEBUG("Reconfigure Request");

    init_mode = config.Working_mode;

    read_distance = config.Distance;

    delta_distance = config.Distance_Offset;

    saved_send_seconds = config.NormalMode_Rate;
    saved_random_option = config.Random_Offset;

    init_iterations = config.Init_iterations;

    init_rate = config.Init_rate;

    decision_mode = config.Decision_mode;

    combine = config.Combine_algorithms;

    period_variation = config.Period_Variation;

    random_period = config.Random_Period;


    if (decision_mode != 0 && init_mode == 1)
        init_mode = 2;

    if ( init_mode == 1 && decision_mode == 0)
    {
        ROS_INFO( "Start Initialization Mode" );
        random_option = false;

        // Update new duration
        send_rate.fromSec(init_rate);

        // Stop any running timer
        send_distance.stop();
        send_distance = nh.createTimer(send_rate, &DistanceMeasNode::send_timer_callback, this);

    }
    else
    {
        ROS_INFO( "Start Normal Mode" );
        random_option = config.Random_Offset;

        send_seconds = config.NormalMode_Rate;
        // Update new duration
        send_rate.fromSec(send_seconds);

        // Stop any running timer
        send_distance.stop();
        send_distance = nh.createTimer(send_rate, &DistanceMeasNode::send_timer_callback, this);
    }
}

void DistanceMeasNode::change_distance( )
{
    if ( random_option == false )
    {
        // Modify between 2 limits
        if ( total_distance == (read_distance + delta_distance) ){
            total_distance = read_distance - delta_distance;
        }
        else{
            total_distance = read_distance + delta_distance;
        }
    }
    else
    {
        // Modify randomly inside 2 limits
        total_distance = drand48() *  2 * delta_distance + ( read_distance - delta_distance) ;
    }
}

void DistanceMeasNode::change_rate( )
{
    if ( random_period == true )
    {
        double rate = drand48() * ( period_variation - 0.2) + 0.2;
        send_rate.fromSec(rate);
        send_distance.stop();
        send_distance = nh.createTimer(send_rate, &DistanceMeasNode::send_timer_callback, this);
        ROS_INFO_STREAM( "rate " << rate );
    }

}

void DistanceMeasNode::send_timer_callback( const ros::TimerEvent& event )
{
    change_distance();
    change_rate( );

    ROS_INFO_STREAM( "Send distance " << total_distance );
    sensor_msgs::BatteryState distMsg;
    distMsg.capacity = total_distance;
    distMsg.present = init_mode;
    distMsg.voltage = init_iterations;
    distMsg.charge = decision_mode;
    distMsg.percentage = combine;
    dist_pub.publish(distMsg);

    // If in initialization
    if ( init_mode == 1){
        // Add new tick
        ++ticks;
        // Only iterate Init_iterations times
        if (ticks >= init_iterations){
            ROS_INFO( "Initialization finished. Return to normal mode" );
            // Initialization finished
            init_mode = 2;
            // Reset counter
            ticks = 0;
            // Restore value
            send_rate.fromSec(saved_send_seconds);
            random_option = saved_random_option;

            // Stop any running timer
            send_distance.stop();
            send_distance = nh.createTimer(send_rate, &DistanceMeasNode::send_timer_callback, this);
        }
    }
}

