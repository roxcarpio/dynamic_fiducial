//  THIS IS RESTRUCTURE  //

#ifndef DYN_FID_HPP
#define DYN_FID_HPP

#include "std_msgs/String.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/InteractiveMarker.h"

#include <geometry_msgs/PointStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Float64.h>

#include <ros/timer.h>
#include <ros/ros.h>
#include <ros/duration.h>

#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <sstream>
//#include <math.h>
#include <cmath>

class decision_process
{
public:
  decision_process();
  ros::NodeHandle nh;

  void manager();

  void arsys_pose_callback (const geometry_msgs::PoseStamped &PoseMsg);
  void cob_pose_callback (const geometry_msgs::PoseStamped &PoseMsg);
  void distance_measure_callback (const sensor_msgs::BatteryState &sensor);

  void check_sync_callback (const geometry_msgs::PointStamped &statusMsg);

  // Timer-callbacks used to clear estimated poses after detectors
  // timeout a certain duration
  void arsys_timer_callback(const ros::TimerEvent& event);
  void cob_timer_callback(const ros::TimerEvent& event);

  // Timer-callback to enter Hysteresis mode
  void protection_timer(const ros::TimerEvent& event);

  // Timer-callback to stop Hysteresis mode
  void stop_hysteresis(const ros::TimerEvent &event);

  // Handle ON / OFF Logic
  void OnOff_calculation();
  // Handle Hysteresis Logic
  void Hysteresis_calculation();

  // Send next Marker type/size
  bool sendToDisplay(double size, std::string type, int marker_id);

  // Check if transitions are occurring faster than the limit
  void check_shifting_rate();

  // Calculate best estimation pose
  geometry_msgs::PoseStamped getBestCurrentPose();

  void reset_variables();
  void check_results();

private:
  // Publishers
  ros::Publisher marker_type_size_pub;
  ros::Publisher pose_selected_pub;

  ros::Publisher reset_counters_pub;

  // Publish info of the conters
  ros::Publisher counters_pub;

  // Subscribers
  ros::Subscriber arsys_estimation_sub;
  ros::Subscriber cob_estimation_sub;
  ros::Subscriber realdist_sub;

  ros::Subscriber checksync_sub;

    // Timer to clean pose estimations
  ros::Duration detect_timeout;
  ros::Timer arsys_timer, cob_timer;

  // Posed estimated from the algorithms
  geometry_msgs::PoseStamped arsys_estimation, cob_estimation;

  // Keeps the current algorithm used for the
  // decision process in Hysteresis mode
  std::string current_type;
  double current_size;
  int current_id;

  // Measured from the sensor (cm)
  double real_distance;

  // Received from the distance measured node
  double simu_distance;

  double current_threshold;
  double total_diff;
  unsigned int count_adaptiv;

  // Offset distance used for Hysteresis mode (cm)
  double epsilon;
  // Increase amount of epsilon if in Hysteresis mode,
  // shits occur
  double epsilon_increment;

  // Counter to check when to switch to
  // Hysteresis mode
  int shift_counter;

  // Time given for a number of shifts to happen to
  // switch to Hysteresis mode (shifts / second )
  int shifting_threshold;
  ros::Duration shifting_interval;
  // Check shifting rate
  ros::Timer check_counter;
  bool start_check;

  // Amount of time in Hysteresis mode
  ros::Duration hysteresis_duration;
  ros::Timer hysteresis_timer;
  bool end_hysteresis;
  double increment_duration;

  // Keep track of detections status
  unsigned int m_count00;
  unsigned int m_count88;
  std::string m_current_yaml;

  // Collect performance result
  unsigned int m_correct;
  unsigned int m_error;
  unsigned int m_ticks;

  // Restore performance results after unexpected behaviour
  unsigned int tmp_ticks;
  unsigned int tmp_correct;
  unsigned int tmp_error;

  // Restore state after error
  unsigned int m_correctcopy;
  unsigned int m_tickscopy;

  int in_init;
  bool after_init;
  unsigned int init_iterations;
  int init_count;
  long double total_delay;
  long double average_delay;
  ros::Time send_time, received_time;

  unsigned int decision_algorithm;
  bool combination;

  double ticks_hysteresis;

  double dist_total;
  unsigned int epsilon_counter;

  ros::Time first_send, last_send;
  bool start_record;

  // Decision mode used
  enum decision_mode {
      OnOff,
      Hysteresis
  }decision_mode;

  // Decision logic implementation
  enum decision_logic {
      On_Off,
      Hyst_static,
      Hyst_incr_duration,
      Hyst_dyn_duration,
      Hyst_incr_range,
      Hyst_adaptiv
  }decision_logic;
};

#endif  // DYN_FID_HPP
