#include "libAgent.h"

int main(int argc, char **argv) {

    ros::init(argc, argv, "agent_node");
    
    Agent MyAgent;

    MyAgent.Manager();

    return 0;
}