#include "tud_coop_uv/definetargetnode.hpp"

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "tracking"); // Name of the node
    DefineTargetNode Node;

    //int32_t looprate = 1000; //hz
    //ros::Rate loop_rate(looprate);

    //ros::spin();
    while(Node.nh.ok()){
        ros::spinOnce();
        //loop_rate.sleep();
        }
}
