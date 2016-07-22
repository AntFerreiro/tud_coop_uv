#include "tud_coop_uv/mergenode.hpp"

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "merge"); // Name of the node
    MergeNode Node;

    int32_t looprate = 1000; //hz
    ros::Rate loop_rate(looprate);

    //ros::spin();
    while(Node.nh.ok()){
        ros::spinOnce();
        loop_rate.sleep();
        }
}

