#ifndef _LIB_PLANNING_H_
#define _LIB_PLANNING_H_

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

struct AstarPoint
{
	int x, y; //点坐标，这里为了方便按照C++的数组来计算，x代表横排，y代表竖列
	double F, G, H; //F=G+H
	AstarPoint *parent; //parent的坐标
	AstarPoint(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL) {}  //变量初始化
};

class Planning {
public:
  Planning();
  ~Planning();

  void Manager();

private:
  /** Const **/
	const int INFINITE = 10000;
  const double Astar_cost_ = 10; 
	const int Astar_depth_limit_ = 5000;
	
  /** Node Handle **/
	ros::NodeHandle n;

	/** Publishers **/
	ros::Publisher costmap_pub;
	ros::Publisher agent_1_path_pub;
	ros::Publisher agent_2_path_pub;
	ros::Publisher agent_group_pub;

	/** Subscribers **/
	ros::Subscriber agent_pose_sub;
	ros::Subscriber agent_id_sub;

  /** Services **/
  ros::ServiceServer path_service;

  /** Parameters **/
	double costmap_resolution_;
	double map_window_height_;
	double map_window_width_;

  /** Variables **/
  nav_msgs::OccupancyGrid costmap_;
  vector<vector<signed char> > Astar_costmap_array_;  //搜索栅格
	sensor_msgs::PointCloud Astar_path_;
	vector<sensor_msgs::PointCloud> Astar_all_path_;
	vector<list<AstarPoint> > Astar_all_array_;
	list<AstarPoint> Astar_array_;

	list<AstarPoint *> Astar_open_list_;  //开启列表
	list<AstarPoint *> Astar_close_list_; //关闭列表

	geometry_msgs::Point32 agent_1_pose_;
	geometry_msgs::Point32 agent_2_pose_;


	/** Function **/
	void Initialization();
	void TfBroadcaster();
	void InitLocalCostmap(nav_msgs::OccupancyGrid& Costmap);
	geometry_msgs::Point32 ConvertCartesianToAstarArray(geometry_msgs::Point32 Point);
	geometry_msgs::Point32 ConvertAstarArrayToCartesian(geometry_msgs::Point32 Point);
	double CalculateAstarG(AstarPoint *temp_start, AstarPoint *point);
	double CalculateAstarH(AstarPoint *point, AstarPoint *start, AstarPoint *end);
	double CalculateAstarF(AstarPoint *point);
	AstarPoint* GetAstarLeastFpoint();
	AstarPoint* FindAstarPath(AstarPoint &startPoint, AstarPoint &endPoint);
	sensor_msgs::PointCloud GetPath(geometry_msgs::Point32 start_array,geometry_msgs::Point32 end_array);
	void SetAstarCostmapPath(vector<list<AstarPoint> > path_costmap, signed char path_occupancy);
	AstarPoint* IsInAstarList(const list<AstarPoint *> &list, const AstarPoint *point);
	bool IsAstarCanreach(const AstarPoint *point, const AstarPoint *target);
	vector<AstarPoint *> GetAstarSurroundPoints(const AstarPoint *point);
	bool IsAstarVaildPoint(const AstarPoint *point);
	nav_msgs::Path ConvertPointcloudToPath(sensor_msgs::PointCloud Input);
	void AgentGroupPublish();

	/** Service **/
  bool PathService(nav_msgs::GetPlan::Request &req, nav_msgs::GetPlan::Response &res) {
		AgentGroupPublish();
    TfBroadcaster();
		geometry_msgs::Point32 goal_point;
		sensor_msgs::PointCloud path_pointcloud;
		goal_point.x = req.goal.pose.position.x;
		goal_point.y = req.goal.pose.position.y;

		if(req.goal.header.frame_id == "agent_1") {
			path_pointcloud = GetPath(agent_1_pose_, goal_point);
			agent_1_path_pub.publish(path_pointcloud);
		}
		else if(req.goal.header.frame_id == "agent_2") {
			path_pointcloud = GetPath(agent_2_pose_, goal_point);
			agent_2_path_pub.publish(path_pointcloud);
		}

		res.plan = ConvertPointcloudToPath(path_pointcloud);
    costmap_pub.publish(costmap_);
    return true;
  }

	/** Callback **/
	void AgentPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& Input) {
		if(Input->header.frame_id == "agent_1") {
			agent_1_pose_.x = Input->pose.position.x;
			agent_1_pose_.y = Input->pose.position.y;
			agent_1_pose_.z = Input->pose.position.z;
		}
		else if(Input->header.frame_id == "agent_2") {
			agent_2_pose_.x = Input->pose.position.x;
			agent_2_pose_.y = Input->pose.position.y;
			agent_2_pose_.z = Input->pose.position.z;
		}
	}
  

};

#endif