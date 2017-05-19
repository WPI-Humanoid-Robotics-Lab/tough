#include <val_task2/map_filter.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "map_filtering");
    ros::NodeHandle nh;
    map_filter mp(nh);
    ros::spin();
    return 0;
}
