#ifndef _LIB_AGENT_H_
#define _LIB_AGENT_H_

#include <string>
#include <time.h>
#include <unistd.h>
#include <vector>
#include <queue>
#include <mutex>
#include <iostream>
#include <fstream>
#include <list>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetPlan.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

using std::string;
using std::cout;
using std::endl;
using std::vector;
using std::to_string;
using std::queue;
using std::list;

class Agent {
public:
  Agent();
  ~Agent();

  void Manager();

private:
  /** Const **/
  const int ROS_RATE_HZ = 20;
	
  /** Node Handle **/
	ros::NodeHandle n;
	ros::NodeHandle pn;

	/** Publishers **/
	ros::Publisher agent_pose_pub;
	ros::Publisher agent_id_pub;

  /** Services **/
  ros::ServiceServer goal_service;

  /** Client **/
  ros::ServiceClient path_client;

  /** Parameters **/
  string agent_id_;
  double start_x_;
  double start_y_;
  double start_z_;

  /** Variables **/

	/** Function **/
  void RunAgentSystem();
  void PublishAgentPose();

	/** Service **/
  bool GoalService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res) {
    nav_msgs::GetPlan get_plan;
    get_plan.request.goal = req.goal;
    path_client.call(get_plan);
    return true;
  }
  

};

#endif