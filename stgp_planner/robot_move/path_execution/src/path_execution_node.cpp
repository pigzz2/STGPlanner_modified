
#include "path_execution/path_execution.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "path_execution_node");

    ros::NodeHandle nh_private("~");
    ros::NodeHandle nh;

    path_execution::PathExecution execute(nh,nh_private);

    ros::spin();

    return 0;
}