#include "libPlanning.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "planner_node");
    
    Planning MyPlanning;

    ros::spin();

    return 0;
}