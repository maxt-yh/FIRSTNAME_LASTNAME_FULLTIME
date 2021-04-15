#include "libPlanning.h"

Planning::Planning() {
  path_service = n.advertiseService("/get_plan", &Planning::PathService, this);
  costmap_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_grid", 1);
	agent_group_pub = n.advertise<sensor_msgs::PointCloud>("/agent_group", 1);
	agent_1_path_pub = n.advertise<sensor_msgs::PointCloud>("/agent_1_path", 1);
	agent_2_path_pub = n.advertise<sensor_msgs::PointCloud>("/agent_2_path", 1);
	agent_pose_sub = n.subscribe("/agent_feedback", 1, &Planning::AgentPoseCallback, this);

  costmap_resolution_ = 1;
  map_window_height_ = 11;
  map_window_width_ = 11;
  
  Initialization();
}

Planning::~Planning() {
  cout << "Planner node was closed ! " << endl;
}

void Planning::Initialization() {
  TfBroadcaster();
  InitLocalCostmap(costmap_);
}


void Planning::TfBroadcaster() {
  static tf::TransformBroadcaster br;
  string child_name = "agent";
  tf::Transform transform;
  transform.setOrigin(tf::Vector3(0, 0, 0.0));
  tf::Quaternion q;
  q.setRPY(0, 0, 0);
  transform.setRotation(q);
  br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "map", child_name));
}

void Planning::InitLocalCostmap(nav_msgs::OccupancyGrid& Costmap) {
  vector<signed char> map_data;
  geometry_msgs::Pose map_origin;

  Costmap.header.frame_id =  "/map";
  Costmap.header.stamp = ros::Time::now();
  Costmap.info.resolution = costmap_resolution_;

  Costmap.info.width  = map_window_width_ / Costmap.info.resolution;
  Costmap.info.height = map_window_height_ / Costmap.info.resolution;

  int costmap_size = Costmap.info.width * Costmap.info.height;
  map_data.resize(costmap_size);
  
  map_origin.position.x = 0;
  map_origin.position.y = -map_window_height_;
  map_origin.orientation = tf::createQuaternionMsgFromRollPitchYaw(0,0,0);

  Costmap.info.origin = map_origin;
  Costmap.data = map_data;

	Astar_costmap_array_.resize(Costmap.info.width,vector<signed char>(Costmap.info.height,0));
	Astar_costmap_array_.assign(Costmap.info.width,vector<signed char>(Costmap.info.height,0));
}

geometry_msgs::Point32 Planning::ConvertCartesianToAstarArray(geometry_msgs::Point32 Point) {
	geometry_msgs::Point32 temp_point;
	temp_point.x = (Point.x - costmap_.info.origin.position.x) / costmap_.info.resolution; //row_num
	temp_point.y = (Point.y - costmap_.info.origin.position.y) / costmap_.info.resolution; //col_num

	return temp_point;
}

geometry_msgs::Point32 Planning::ConvertAstarArrayToCartesian(geometry_msgs::Point32 Point) {
	geometry_msgs::Point32 temp_point;
	temp_point.x = Point.x * costmap_.info.resolution + costmap_.info.origin.position.x; //row_num
	temp_point.y = - Point.y * costmap_.info.resolution + costmap_.info.origin.position.y; //col_num

	return temp_point;
}

double Planning::CalculateAstarG(AstarPoint *temp_start, AstarPoint *point) {
	double extraG = Astar_cost_;
	double parentG = point->parent == NULL ? 0 : temp_start->G; 
	return parentG + extraG;
}
 
double Planning::CalculateAstarH(AstarPoint *point, AstarPoint *start, AstarPoint *end) {
	if(!Astar_all_array_.empty()) {
		for(int i = 0; i < Astar_all_array_.size(); i++) {
			for(auto raw_point : Astar_all_array_[i]) {
				if(point->x == raw_point.x && point->y == raw_point.y) 
				return INFINITE;
			}
		}
	}
	return (abs(end->x - point->x) + abs(end->y - point->y)) * Astar_cost_;
}
 
double Planning::CalculateAstarF(AstarPoint *point) {
	return point->G + point->H;
}
 
AstarPoint* Planning::GetAstarLeastFpoint() {
	if(!Astar_open_list_.empty()) {
		auto resPoint = Astar_open_list_.front();
		for(auto &point : Astar_open_list_)
		if(point->F < resPoint->F)
			resPoint = point;
		return resPoint;
	}
	return NULL;
}
 
AstarPoint* Planning::FindAstarPath(AstarPoint &startPoint, AstarPoint &endPoint) {
	if(!IsAstarVaildPoint(&endPoint)){
		cout << "The endPoint was invaild !" << endl;
		return NULL;
	}

	if(startPoint.x == endPoint.x && startPoint.y == endPoint.y){
		cout << "The endPoint coinsided with startPoint !" << endl;
		return NULL;
	}

	Astar_open_list_.push_back(new AstarPoint(startPoint.x,startPoint.y)); 
  int Astar_depth_search = 0;
	while (!Astar_open_list_.empty()) {
    Astar_depth_search++;
    // cout << "Astar search num : " << Astar_depth_search << endl;
    if(Astar_depth_search >= Astar_depth_limit_) return NULL;

		auto current_point = GetAstarLeastFpoint(); 
		Astar_open_list_.remove(current_point); 
		Astar_close_list_.push_back(current_point);

		int row_num = current_point->x; 
		int col_num = costmap_.info.height - current_point->y - 1;
		costmap_.data[col_num * costmap_.info.width + row_num] = 10;

		auto surround_points = GetAstarSurroundPoints(current_point);
		for(auto &target : surround_points) {
			if(!IsInAstarList(Astar_open_list_, target)) {
 
				target->parent = current_point;
				target->G = CalculateAstarG(current_point, target);
				target->H = CalculateAstarH(target, &startPoint, &endPoint);
				target->F = CalculateAstarF(target);
 
				Astar_open_list_.push_back(target);
			} else {
				double tempG = CalculateAstarG(current_point, target);
				if(tempG < target->G) {
					target->parent = current_point;
 
					target->G = tempG;
					target->F = CalculateAstarF(target);
				}
			}
			AstarPoint *resPoint = IsInAstarList(Astar_open_list_, &endPoint); //若求最短路径，需要判断endPoint是否在Astar_close_list_
			if(resPoint) return resPoint;
		}
	}
	return NULL;
}
 
sensor_msgs::PointCloud Planning::GetPath(geometry_msgs::Point32 start_array,geometry_msgs::Point32 end_array) {
	AstarPoint startPoint(static_cast<int> (start_array.x),static_cast<int> (start_array.y));
	AstarPoint endPoint(static_cast<int> (end_array.x),static_cast<int> (end_array.y));
  
	sensor_msgs::PointCloud path_candidate;
	list<AstarPoint> array_candidate;
	AstarPoint *result = FindAstarPath(startPoint, endPoint);
	while(result) {
		array_candidate.push_back(*result);

		geometry_msgs::Point32 temp_point;
		temp_point.x = result->x;
		temp_point.y = -result->y;
		temp_point.z = 10;
		path_candidate.points.push_back(temp_point);
		
		result = result->parent;
	}

	path_candidate.header.stamp = ros::Time::now();
	path_candidate.header.frame_id = "/map";
	Astar_all_path_.push_back(path_candidate);
	Astar_all_array_.push_back(array_candidate);

	for(list<AstarPoint*>::iterator it = Astar_open_list_.begin();it != Astar_open_list_.end();it++) {
		delete *it;
		*it = NULL;
	}
	for(list<AstarPoint*>::iterator vit = Astar_close_list_.begin();vit != Astar_close_list_.end();vit++) {
		delete *vit;
		*vit = NULL;
	}
	Astar_open_list_.clear();
	Astar_close_list_.clear();

  signed char path_occupancy = 50;
  SetAstarCostmapPath(Astar_all_array_,path_occupancy);

  if(!array_candidate.empty()) ROS_INFO("Astar path was found !");
  else ROS_ERROR("Astar path could not be found !");

	return path_candidate;
}

void Planning::SetAstarCostmapPath(vector<list<AstarPoint> > path_costmap, signed char path_occupancy) {
  if(path_costmap.empty()) return;
	for(auto single_path : path_costmap) {
		for(list<AstarPoint>::iterator iter = single_path.begin(); iter != single_path.end(); iter++) {
			costmap_.data[(costmap_.info.height - (*iter).y - 1) * costmap_.info.width + (*iter).x] = path_occupancy;
			Astar_costmap_array_[(*iter).x][(*iter).y] = path_occupancy;
		}
	}
}

AstarPoint *Planning::IsInAstarList(const list<AstarPoint *> &list, const AstarPoint *point) {
	for(auto p : list)
	if(p->x == point->x && p->y == point->y)
		return p;
	return NULL;
}
 
bool Planning::IsAstarCanreach(const AstarPoint *point, const AstarPoint *target) {
	if(target->x < 0 || target->x > Astar_costmap_array_.size() - 1
		|| target->y < 0 || target->y>Astar_costmap_array_[0].size() - 1)
		return false;
	if(Astar_costmap_array_[target->x][target->y] >= 50
		|| (target->x == point->x && target->y == point->y)
		|| IsInAstarList(Astar_close_list_, target))  //如果点与当前节点重合、超出地图、是障碍物、或者在关闭列表中，返回false
		return false;
	return true;
}
 
vector<AstarPoint *> Planning::GetAstarSurroundPoints(const AstarPoint *point) {
	vector<AstarPoint *> surround_points;

	int x = point->x - 1;
	int y = point->y;
	if(IsAstarCanreach(point,new AstarPoint(x, y))) surround_points.push_back(new AstarPoint(x, y));

	x = point->x + 1;
	y = point->y;
	if(IsAstarCanreach(point,new AstarPoint(x, y))) surround_points.push_back(new AstarPoint(x, y));

	x = point->x;
	y = point->y - 1;
	if(IsAstarCanreach(point,new AstarPoint(x, y))) surround_points.push_back(new AstarPoint(x, y));

	x = point->x;
	y = point->y + 1;
	if(IsAstarCanreach(point,new AstarPoint(x, y))) surround_points.push_back(new AstarPoint(x, y));

	return surround_points;
}

bool Planning::IsAstarVaildPoint(const AstarPoint *point) {
	if(point->x >= 0 && point->x <= costmap_.info.width-1 
	  && point->y >= 0 && point->y <= costmap_.info.height-1) {
		return true;
	}
	return false;
}

nav_msgs::Path Planning::ConvertPointcloudToPath(sensor_msgs::PointCloud Input) {
	nav_msgs::Path temp_path;
	temp_path.header.frame_id = "/map";
	temp_path.header.stamp = ros::Time::now();

	geometry_msgs::PoseStamped temp_pose_stamped;
	temp_pose_stamped.header.frame_id = "/map";

	for(int i = 0; i < Input.points.size(); i++) {
		temp_pose_stamped.pose.position.x = Input.points[i].x;
		temp_pose_stamped.pose.position.y = Input.points[i].y;
		temp_path.poses.push_back(temp_pose_stamped);
	}

	return temp_path;
}

void Planning::AgentGroupPublish() {
	sensor_msgs::PointCloud temp_pointcloud;
	temp_pointcloud.header.frame_id = "/map";
	temp_pointcloud.header.stamp = ros::Time::now();

	geometry_msgs::Point32 temp_point;
	for(int i = 0; i < costmap_.info.width * costmap_.info.height; i++) {
		temp_point.x = i / costmap_.info.height;
		temp_point.y = i % costmap_.info.height;
		temp_point.y = -temp_point.y; 
		temp_pointcloud.points.push_back(temp_point);
	}
	agent_group_pub.publish(temp_pointcloud);
}