#include "libAgent.h"

Agent::Agent():pn("~") {
  pn.param<string>("agent_id",agent_id_,"");
  pn.param<double>("start_x",start_x_,0);
  pn.param<double>("start_y",start_y_,0);
  pn.param<double>("start_z",start_z_,0);

  cout << start_x_ << " " << start_y_ << " " << endl;

  goal_service = n.advertiseService("/update_goal", &Agent::GoalService, this);
  path_client = n.serviceClient<nav_msgs::GetPlan>("/get_plan", 1);

  agent_pose_pub = n.advertise<geometry_msgs::PoseStamped>("/agent_feedback", 1);
}

Agent::~Agent() {
  cout << "Agent node was closed ! " << endl;
}

void Agent::Manager() {
  ros::Rate loop_rate(ROS_RATE_HZ);

  while (ros::ok()) {
    loop_rate.sleep();
    ros::spinOnce();

    RunAgentSystem();
  }
}

void Agent::RunAgentSystem() {
  PublishAgentPose();
}

void Agent::PublishAgentPose() {
  geometry_msgs::PoseStamped start_pose;
  start_pose.header.frame_id = agent_id_;
  start_pose.pose.position.x = start_x_;
  start_pose.pose.position.y = start_y_;
  start_pose.pose.position.z = start_z_;

  geometry_msgs::PoseStamped now_pose = start_pose;
  agent_pose_pub.publish(now_pose);
}