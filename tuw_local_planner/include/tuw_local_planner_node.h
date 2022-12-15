#ifndef LOCAL_PLANNER_NODE_H
#define LOCAL_PLANNER_NODE_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose2D.h>
#include <sensor_msgs/LaserScan.h>
#include <dynamic_reconfigure/server.h>
#include <tuw_local_planner/tuw_local_planner.h>
#include <tuw_local_planner/LocalPlannerConfig.h>
/**
 * class to cover the ros communication
 **/
class LocalPlannerNode : public tuw::LocalPlanner {
public:
    LocalPlannerNode ( ros::NodeHandle & n ); /// Constructor
    void publishMotion ();      /// publishes the motion commands 
private:
    ros::NodeHandle n_;         /// node handler to the root node
    ros::NodeHandle n_param_;   /// node handler to the current node
    ros::Subscriber sub_laser_; /// Subscriber to the laser measurements
    ros::Publisher pub_cmd_;    /// publisher for the motion commands
    void callbackLaser ( const sensor_msgs::LaserScan& );   /// callback function to execute on incoming sensor data
    void callbackConfigLocalPlanner ( tuw_local_planner::LocalPlannerConfig &config, uint32_t level ); /// callback function on incoming parameter changes
    dynamic_reconfigure::Server<tuw_local_planner::LocalPlannerConfig> reconfigureServer_; /// parameter server stuff
    dynamic_reconfigure::Server<tuw_local_planner::LocalPlannerConfig>::CallbackType reconfigureFnc_;  /// parameter server stuff
};

#endif // PLANNER_LOCAL_NODE_H
