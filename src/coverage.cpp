#include "tud_coop_uv/coveragenode.hpp"

int main(int argc,char* argv[])
{
    ros::init(argc, argv, "coverage"); // Name of the node
    CoverageNode Node;

    int32_t looprate = 1; //hz
    ros::Rate loop_rate(looprate);

    //ros::spin();
    while(Node.nh.ok()){
        ros::spinOnce();
        Node.draw_lines();
        loop_rate.sleep();
        }
}
