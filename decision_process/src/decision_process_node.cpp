#include "decision_process/decision_process_node.hpp"


////////////////////// CONFIGURATION ///////////////////////////////////////////////////////

static int num_of_thresholds = 6;
double threshold_list[] = { 7 , 30 , 155 , 245 , 285 , 346 };    // [min_dist, th1 , th2 , .. , max_dist]
static const char * markertype_list[] = { "ARUCO", "PITAG", "PITAG" , "ARUCO" , "ARUCO" };
int markerId_list[] =                   { 88 , 05 , 1, 0, 88  };   // Pitag does not read Id
int markersize_list[] =                 { 4 , 14 , 14 , 14, 14};

// hysteresis range
#define EPSILON         2
// increment of range
#define EPSILON_DELTA   5
// hysteresis duration
#define HYST_DURATION   0.3
// sensibility to enter hysteresis
#define PERCENT_MARGIN  1.6
/////////////////////////////////////////////////////////////////////////////////////////////

decision_process::decision_process() :
    detect_timeout(0.1),
    shifting_interval(1), // default to 1 shift / 1 sec
    shift_counter(0),
    start_check(false),
    increment_duration(HYST_DURATION),
    hysteresis_duration(HYST_DURATION), // remaining time in hysteresis (seconds)
    end_hysteresis(true)
{
    // Initialise Hysteresis offset
    epsilon = EPSILON;
    epsilon_increment = EPSILON_DELTA;

    // Default to OnOff
    decision_mode = OnOff;
    decision_algorithm = Hyst_adaptiv;
    combination = true;

    // Keep status for Hysteresis
    current_type = "";
    current_size = 0.0;
    current_id = 0;

    // Keep track of detections status
    m_count00 = 0;
    m_count88 = 0;
    m_current_yaml = "";

    // Init performance counters
    // Restore performance results after unexpected behaviour
    // Init copies used for managing
    // Calculate delay of the system
    reset_variables();
    in_init = 0;

    // Calculate Hysteresis dynamic duration
    ticks_hysteresis = 0;

    start_record = false;

    /// Publishers ///
    // Inform about the marker type and marker to be displayed
    marker_type_size_pub = nh.advertise<visualization_msgs::InteractiveMarker>("type_size_marker",1);

    // Pose selected
    pose_selected_pub = nh.advertise<geometry_msgs::PoseStamped>("pose_selected",1);

    // Inform about sync error between ar_sys and decision process
    reset_counters_pub = nh.advertise<std_msgs::Float64>("reset_counters",1);

    // Sends the error correct and ticks conters
    counters_pub = nh.advertise<geometry_msgs::PointStamped>("counters_data", 100);

    /// Subscribers ///
    // Read estimation from Arsys
    arsys_estimation_sub = nh.subscribe("/ar_multi_boards/pose", 1, &decision_process::arsys_pose_callback , this);

    // Read estimation from Cob Fiducial
    cob_estimation_sub = nh.subscribe("/fiducials/pose", 1, &decision_process::cob_pose_callback , this);

    // Read status from Ar_sys
    checksync_sub = nh.subscribe("/ar_multi_boards/detect_status", 1, &decision_process::check_sync_callback , this);

    // Read from distance sensor
     realdist_sub = nh.subscribe("/distance_meas/distance", 1, &decision_process::distance_measure_callback , this);
}

// Listener for distances measured
void decision_process::distance_measure_callback (const sensor_msgs::BatteryState &distance_measured)
{
    // Store real distance for comparing
    simu_distance = distance_measured.capacity;
    //ROS_INFO_STREAM("Real distance is: " << real_distance);

    // Store the decision process mode selected
    decision_algorithm = distance_measured.charge;

    // Store the number of iterations used at initialization
    init_iterations = distance_measured.voltage;

    // In initialization
    in_init = distance_measured.present;

    // Combine or not ONOFF and Hysteresis
    combination = distance_measured.percentage;

    getBestCurrentPose();
}

/// Listener for Arsys estimations
void decision_process::arsys_pose_callback (const geometry_msgs::PoseStamped &PoseMsg)
{
    // Store estimation
    arsys_estimation = PoseMsg;

    // Handle timer
    // If arsys estimation received, expect more to come...
    arsys_timer = nh.createTimer(detect_timeout, &decision_process::arsys_timer_callback , this);

    getBestCurrentPose();
}

//// Listener for Cob fiducial estimations
void decision_process::cob_pose_callback (const geometry_msgs::PoseStamped &PoseMsg)
{
    // Store estimation
    cob_estimation = PoseMsg;

    // Handle timer
    // If cob estimation received, expect more to come...
    cob_timer = nh.createTimer(detect_timeout, &decision_process::cob_timer_callback , this);

    getBestCurrentPose();
}

void decision_process::OnOff_calculation()
{
    // Default to the most demanding scenario
    std::string marker_type = markertype_list[ num_of_thresholds - 2 ];
    double marker_size = markersize_list[ num_of_thresholds - 2 ]; // size in cm2
    int marker_id  = markerId_list[num_of_thresholds - 2];

    if (real_distance < threshold_list[0] || real_distance > threshold_list[num_of_thresholds -1])
    {
        ROS_INFO_STREAM( "Target out of range:" );
        ROS_INFO_STREAM( "Minimum distance = " << threshold_list[0] );
        ROS_INFO_STREAM( "Maximum distance = " << threshold_list[num_of_thresholds -1]);
        ROS_INFO_STREAM( "Distance of the target = " << real_distance );
    }
    else
    {
        // Search for the indicated option
        for (int i = 0; i < num_of_thresholds - 2; i++)
        {
            if ( threshold_list[i] <= real_distance && real_distance < threshold_list[i+1] )
            {
                marker_type = markertype_list[i];
                marker_size = markersize_list[i];
                marker_id = markerId_list[i];
            }
        }
    }

    // Combination ONOFF / Hysteresis
    if (decision_algorithm != On_Off && (combination == true) )
    {
        if ( current_type != marker_type || current_size != marker_size || current_id != marker_id ){

            shift_counter++;
            check_shifting_rate();
        }
    }
    sendToDisplay(marker_size, marker_type, marker_id);
}

void decision_process::Hysteresis_calculation()
{
    // Default to the most demanding scenario
    std::string marker_type = markertype_list[num_of_thresholds - 2];
    double marker_size = markersize_list[num_of_thresholds - 2]; // size in cm2
    int marker_id = markerId_list[num_of_thresholds - 2];

    if (real_distance < threshold_list[0] || real_distance > threshold_list[num_of_thresholds -1])
    {
        ROS_INFO( "Target out of range:" );
        ROS_INFO_STREAM( "Minimum distance = " << threshold_list[0] );
        ROS_INFO_STREAM( "Maximum distance = " << threshold_list[num_of_thresholds -1]);
        ROS_INFO_STREAM( "Distance of the target = " << real_distance );
    }
    else
    {
        if (current_type == "")
        {
            // First time normal ONOFF search to fill current_x members
            for (int i = 0; i < num_of_thresholds - 2; i++){
                if ( threshold_list[i] <= real_distance && real_distance < threshold_list[i+1] ){
                    marker_type = markertype_list[i];
                    marker_size = markersize_list[i];
                    marker_id = markerId_list[i];
                }
            }
        }
        else{
            for (int i = 0; i <= num_of_thresholds - 1; i++){
                // Fix min distance bug manually...
                if ( (threshold_list[i] <= real_distance) && real_distance <=(threshold_list[i] + epsilon)
                     && (i == 0)){
                    marker_type = markertype_list[i];
                    marker_size = markersize_list[i];
                    marker_id = markerId_list[i];
                    break;
                }
                // If in hysteresis range - keep current marker
                else if ( (threshold_list[i] - epsilon) <= real_distance &&
                     real_distance <= (threshold_list[i] + epsilon) ){
                    marker_type = current_type;
                    marker_size = current_size;
                    marker_id = current_id;
                    current_threshold = threshold_list[i];

                    // Dynamically decide remaining duration for:
                    // Hysteresis dynamic_duration, incremental_range and adaptativ
                    if ( decision_algorithm == Hyst_dyn_duration ||
                         decision_algorithm == Hyst_incr_range ||
                         decision_algorithm == Hyst_adaptiv)
                    {
                        // If in hysteresis range and
                        // If new distance smaller than Th...
                        if ( real_distance < threshold_list[i])
                        {
                            // ... and new marker would be selected...
                            if ( markerId_list[i-1] != current_id )
                                // Add
                                ticks_hysteresis++;
                        }
                        // Same on the other side
                        else{
                            if ( markerId_list[i] != current_id )
                                ticks_hysteresis++;
                        }
                        check_shifting_rate();
                    }
                    current_threshold = threshold_list[i];
                    break;
                }
                // else onoff logic
                else if ( (threshold_list[i] + epsilon) < real_distance &&
                          real_distance < (threshold_list[i+1] - epsilon) ){
                    marker_type = markertype_list[i];
                    marker_size = markersize_list[i];
                    marker_id = markerId_list[i];
                    break;
                }
            }
        }
    }
    // Hysteresis adaptativ
    if ( decision_algorithm == Hyst_adaptiv )
    {
        double diff = fabs( current_threshold - real_distance );
        total_diff += diff;
        count_adaptiv++;
        epsilon = total_diff / count_adaptiv;
    }

    // Dynamically decide if hysteresis range needs to increase for:
    // Hysteresis incremental_range
    if ( decision_algorithm == Hyst_incr_range )
    {
        // If in Hysteresis mode, shifts occur...
        if ( current_type != marker_type || current_size != marker_size || current_id != marker_id ){

            // Enlarge epsilon range
            epsilon += epsilon_increment;

            // Dont exit hysteresis mode
            end_hysteresis = false;
            hysteresis_timer.stop();
            // Restart
            end_hysteresis = true;
            hysteresis_timer.start();
        }
    }
    sendToDisplay(marker_size, marker_type, marker_id);
}

bool decision_process::sendToDisplay(double size, std::string type, int marker_id)
{
    // Only send if new display required
    if( (marker_id == current_id) && (type == current_type) && (size == current_size) )
    {
        return false;
    }

    // Update current status
    current_type = type;
    current_size = size;
    current_id = marker_id;

    if ( m_tickscopy != m_correctcopy ){
        //ROS_INFO( "ERROR - MISSED one marker");
        m_error++;

        std_msgs::Float64 resetCountersMsg;
        // Reset ar_sys counters
        resetCountersMsg.data = -1;
        reset_counters_pub.publish( resetCountersMsg );

        // Reset own counters
        m_count00 = 0;
        m_count88 = 0;

        // Reset copies
        m_correctcopy = 0;
        m_tickscopy = 0;
    }

    check_results();

    if ( current_type == "ARUCO" && current_size == 14 && current_id == 88)
    {
        m_count88++;
        m_current_yaml = "YAML_88_14";
        m_ticks++;
        m_tickscopy++;
    }
    else if ( current_type == "ARUCO" && current_size == 14 && current_id == 0)
    {
        m_count00++;
        m_current_yaml = "YAML_00_14";
        m_ticks++;
        m_tickscopy++;
    }

    ROS_INFO_STREAM( "Next marker type is : [ " << type << " ] with size [ " << size
                     << " ] and Id [ " << marker_id << " ]");
    // Publish next marker type/size
    visualization_msgs::InteractiveMarker markerToDisplayMsg;
    markerToDisplayMsg.scale = size; // Marker size
    markerToDisplayMsg.header.frame_id = type; // Marker type
    markerToDisplayMsg.pose.position.x = marker_id; // Id information
    marker_type_size_pub.publish(markerToDisplayMsg);

    return true;
}


geometry_msgs::PoseStamped decision_process::getBestCurrentPose()
{
    geometry_msgs::PoseStamped result_pose;
    double dist_estimation_cob, dist_estimation_arsys;

    // Collect current distance estimations
    dist_estimation_cob = cob_estimation.pose.position.z * 100; // cm
    dist_estimation_arsys = arsys_estimation.pose.position.z * 100; // cm

    // Fiducial detection
    if ( dist_estimation_cob > 0 )
    {
        real_distance = dist_estimation_cob;
        result_pose = cob_estimation;

    }
    // Aruco detection
    else if( dist_estimation_arsys > 0)
    {
        real_distance = dist_estimation_arsys;
        result_pose = arsys_estimation;
    }
    // Distance measure
    else
    {
        real_distance = simu_distance;
    }

    // Publish final pose
    if ( dist_estimation_cob > 0 || dist_estimation_arsys > 0 )
    {
        pose_selected_pub.publish(result_pose);
    }

    manager();

    return result_pose;
}

void decision_process::arsys_timer_callback( const ros::TimerEvent& event )
{
    // reset pose
    arsys_estimation.pose.position.x = 0;
    arsys_estimation.pose.position.y = 0;
    arsys_estimation.pose.position.z = 0;

    arsys_estimation.pose.orientation.x = 0;
    arsys_estimation.pose.orientation.y = 0;
    arsys_estimation.pose.orientation.z = 0;

    //ROS_INFO( "Aruco callback time-out!" );
    //ROS_INFO( "Reset Aruco pose estimation..." );
    arsys_timer.stop();
}

void decision_process::cob_timer_callback( const ros::TimerEvent& event)
{
    // reset pose
    cob_estimation.pose.position.x = 0;
    cob_estimation.pose.position.y = 0;
    cob_estimation.pose.position.z = 0;

    cob_estimation.pose.orientation.x = 0;
    cob_estimation.pose.orientation.y = 0;
    cob_estimation.pose.orientation.z = 0;

    //ROS_INFO( "Pitag callback time-out!" );
    //ROS_INFO( "Reset Pitag pose estimation..." );
    cob_timer.stop();
}

void decision_process::protection_timer(const ros::TimerEvent &event)
{
    // If shifts/sec > 1 shift/average_delay
    if ( shift_counter > 1  || ticks_hysteresis > 1 ){

        decision_mode = Hysteresis;
        // If called from onoff
        if (shift_counter > 1)
        {
            ROS_INFO( "START HYSTERESIS MODE" );
            if ( decision_algorithm == Hyst_incr_duration)
            {
                hysteresis_duration.fromSec(increment_duration);
                increment_duration +=1;
            }
        }

        // If called from hysteresis
        if (ticks_hysteresis > 1)
        {
            ROS_INFO( "REMAIN IN HYSTERESIS MODE" );
            // Dont exit hysteresis mode
            end_hysteresis = false;
            hysteresis_timer.stop();
            // Reset counter
            ticks_hysteresis = 0;
            // Enable go back to ONOFF
            end_hysteresis = true;
        }

        // Start new timer, only stay in Hysteresis - "hysteresis_duration"
        hysteresis_timer = nh.createTimer(hysteresis_duration, &decision_process::stop_hysteresis, this);
    }

    // After timeout, reset
    shift_counter = 0;
    check_counter.stop();
    start_check = false;
}

void decision_process::stop_hysteresis(const ros::TimerEvent &event)
{
    if (end_hysteresis == true){
        ROS_INFO( "EXIT HYSTERESIS MODE" );
        if (combination == true)
        {
            // Back to OnOff after timeout
            decision_mode = OnOff;
        }
        dist_total = 0;
        epsilon_counter = 0;
        // Back to initial epsilon length
        epsilon = EPSILON;

        // For adaptive hysteresis
        total_diff = 0;
        count_adaptiv = 0;
    }
    hysteresis_timer.stop();
}


void decision_process::check_sync_callback (const geometry_msgs::PointStamped &statusMsg)
{
    // Error from Ar_sys
    unsigned int reset_flag = statusMsg.point.z;
    if ( reset_flag == -1 )
    {
        //ROS_INFO( "Ar_sys ERROR - MISSED one Marker");
        m_error++;
        // Reset counters
        m_count00 = 0;
        m_count88 = 0;
        return;
    }

    // Current marker detector used by Ar_sys node
    std::string detect_yaml = statusMsg.header.frame_id;
    // Number of times the Ar_sys node has detected each marker
    unsigned int detection_counter_00 = statusMsg.point.x;
    unsigned int detection_counter_88 = statusMsg.point.y;

    //
    std_msgs::Float64 resetCountersMsg;

    if ( detect_yaml == m_current_yaml )
    {
        if ( detection_counter_00 == m_count00 && detection_counter_88 == m_count88)
        {
            ROS_INFO( "CORRECT detection");
            m_correct++;
            m_correctcopy++;

            if ( in_init == 1)
            {
                // Correct iteration
                init_count++;
                // End side of the count
                received_time = ros::Time::now();
                total_delay = total_delay + ( received_time.toNSec() - send_time.toNSec() );
            }
        }
        else
        {
            //ROS_INFO( "ERROR - COUNTERS do NOT match");
            //ROS_INFO_STREAM( "Ar_sys 00 COUNTER = " << detection_counter_00);
            //ROS_INFO_STREAM( "DECISION 00 COUNTER = " << m_count00);

            //ROS_INFO_STREAM( "Ar_sys 88 COUNTER = " << detection_counter_88);
            //ROS_INFO_STREAM( "DECISION 88 COUNTER = " << m_count88);

            // Reset ar_sys counters
            resetCountersMsg.data = -1;
            reset_counters_pub.publish( resetCountersMsg );

            // Reset own counters
            m_count00 = 0;
            m_count88 = 0;
        }
    }
    else
    {
        ROS_INFO( "ERROR - YAMLs do NOT match");
        ROS_INFO_STREAM( "Marker detected = " << detect_yaml);
        ROS_INFO_STREAM( "Current Marker selected = " << m_current_yaml);

        // Reset ar_sys counters
        resetCountersMsg.data = -1;
        reset_counters_pub.publish( resetCountersMsg );

        // Reset own counters
        m_count00 = 0;
        m_count88 = 0;
    }
}

void decision_process::check_shifting_rate()
{
    // Only one controler at a time
    if ( start_check == false ){
        // Start protection timer
        check_counter = nh.createTimer(shifting_interval, &decision_process::protection_timer, this);
        start_check = true;
    }
}

void decision_process::reset_variables()
{
    // Initialization parameters
    init_iterations = 0;
    init_count = 0;
    total_delay = 0;
    average_delay = 0;
    send_time.fromSec(0);
    received_time.fromSec(0);
    after_init = false;

    // Init performance counters
    m_error = 0;
    m_correct = 0;
    m_ticks = 0;

    // Restore performance results after unexpected behaviour
    tmp_ticks = 0;
    tmp_correct = 0;
    tmp_error = 0;

    // Init copies used for managing performance
    m_correctcopy = 0;
    m_tickscopy = 0;

    // Adaptive hysteresis
    total_diff = 0;
    count_adaptiv = 0;

    start_record = false;
}

void decision_process::check_results()
{
    // Results
    if ( m_ticks == m_correct + m_error)
    {
        //ROS_INFO_STREAM( "ticks [ " << m_ticks << " ] = correct [ " << m_correct
        //                << " ] + errors [ " << m_error << " ]");

        // If in Normal Mode
        if ( in_init == 0 && start_record == false)
        {
            // Start count
            first_send = ros::Time::now();
            start_record = true;
        }
        if ( m_ticks == 100 && in_init == 0)
        {
            double complete_duration = 0;
            last_send = ros::Time::now();
            start_record = false;
            complete_duration = (last_send.toNSec() - first_send.toNSec() );
            ROS_INFO_STREAM( "COMPLETE DURATION is " << complete_duration * pow (10.0, -9.0));
        }

        // Store every 100 samples
        if ( m_ticks == 100 ){
        geometry_msgs::PointStamped counters_msg;
        counters_msg.header.frame_id = "Data Counters: x=Ticks, y=Correct, z=Error";
        counters_msg.point.x = m_ticks;
        counters_msg.point.y = m_correct;
        counters_msg.point.z = m_error;
        // Send results
        counters_pub.publish(counters_msg);
        }

        // Save correct round
        tmp_ticks = m_ticks;
        tmp_correct = m_correct;
        tmp_error = m_error;
    }
    else
    {
         ROS_INFO( "State counters NOT ALIGNED");
         ROS_INFO_STREAM( "ticks [ " << m_ticks << " ] = correct [ " << m_correct
                          << " ] + errors [ " << m_error << " ]");

        // In case of wrong test, invalidate attempt and
        // use previous state
        m_ticks = tmp_ticks;
        m_correct = tmp_correct;
        m_error = tmp_error;
    }
}

void decision_process::manager()
{
    if (in_init == 2)
    {
        // Initialization was called
        if (  (after_init == true) && init_count == init_iterations )
        {
            ROS_INFO("Initialization SUCCEEDED");
            // Calculate average of the samples
            average_delay = total_delay / init_iterations;
            // Update limit of the system
            shifting_interval.fromNSec( average_delay * PERCENT_MARGIN );
            //shifting_interval.fromSec( 0.275 * PERCENT_MARGIN );
            ROS_INFO_STREAM( "Average delay of the system = " << average_delay * pow (10.0, -9.0) << " seconds");
            ROS_INFO_STREAM( "Threshold set every = " << average_delay * PERCENT_MARGIN * pow (10.0, -9.0) << " ticks/seconds");
            reset_variables();
        }
        // Just print 1 time after init request
        else if ( (after_init == true) && (init_count != init_iterations) )
        {
            ROS_INFO("Initialization FAILED");
            ROS_INFO_STREAM( "Only " << init_count << " out of " << init_iterations << " iterations succeeded");
            reset_variables();
        }
        return;
    }

    // In initialization mode
    if ( in_init == 1 )
    {
        // Start count
        send_time = ros::Time::now();
        after_init = true;
    }

    // Use OnOff and switch to Hysteresis if necessary
    if ( (decision_algorithm == On_Off) )
    {
        decision_mode = OnOff;
    }

    // Only stay in hysteresis
    if ( (combination == false) && (decision_algorithm != On_Off) )
    {
        decision_mode = Hysteresis;
    }

    // Reinitialise duration
    if ( decision_algorithm != Hyst_incr_duration )
    {
        increment_duration = HYST_DURATION;
        hysteresis_duration.fromSec(HYST_DURATION);
    }

    switch ( decision_mode )
    {
    case OnOff:
        OnOff_calculation();
        break;
    case Hysteresis:
        Hysteresis_calculation();
        break;
    default:
        ROS_INFO( "Unknown decision mode" );
        break;
    }
}

