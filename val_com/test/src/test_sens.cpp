#include <val_interface/val_interface.h>
#include <humanoid_common/com.h>

#define IMU_FORCE_SENS_TEST 1
#define COM_TEST 0

// test code
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test");
    ros::NodeHandle nh;

    ros::Rate loop_rate(50);

    valInterface interface(nh);

    while(ros::ok())
    {

        // will generate IMU data and force sensor data
        if (IMU_FORCE_SENS_TEST)
        {
            std::cout << interface.getPelvisMiddleImu().linear_acceleration.x <<"," << interface.getPelvisMiddleImu().linear_acceleration.y <<"," << interface.getPelvisMiddleImu().linear_acceleration.z <<"," << \
                         interface.getPelvisMiddleImu().orientation.x <<"," << interface.getPelvisMiddleImu().orientation.y <<"," << interface.getPelvisMiddleImu().orientation.z <<"," << interface.getPelvisMiddleImu().orientation.w <<"," << \
                         interface.getPelvisRearIMU().linear_acceleration.x <<"," << interface.getPelvisRearIMU().linear_acceleration.y <<"," << interface.getPelvisRearIMU().linear_acceleration.z <<"," << \
                         interface.getPelvisRearIMU().orientation.x <<"," << interface.getPelvisRearIMU().orientation.y <<"," << interface.getPelvisRearIMU().orientation.z <<"," << interface.getPelvisRearIMU().orientation.w <<"," << \
                         interface.getLeftTorsoIMU().linear_acceleration.x <<"," << interface.getLeftTorsoIMU().linear_acceleration.y <<"," << interface.getLeftTorsoIMU().linear_acceleration.z <<"," << \
                         interface.getLeftTorsoIMU().orientation.x <<"," << interface.getLeftTorsoIMU().orientation.y <<"," << interface.getLeftTorsoIMU().orientation.z <<"," << interface.getLeftTorsoIMU().orientation.w <<"," << \
                         interface.getForceSensLeftFoot().force.x <<"," <<interface.getForceSensLeftFoot().force.y <<"," << interface.getForceSensLeftFoot().force.z <<"," << \
                         interface.getForceSensLeftFoot().torque.x <<"," <<interface.getForceSensLeftFoot().torque.y <<"," << interface.getForceSensLeftFoot().torque.z <<"," << \
                         interface.getForceSensLeftFootOffset().force.x <<"," << interface.getForceSensLeftFootOffset().force.y <<"," << interface.getForceSensLeftFootOffset().force.z <<"," << \
                         interface.getForceSensLeftFootOffset().torque.x <<"," << interface.getForceSensLeftFootOffset().torque.y <<"," << interface.getForceSensLeftFootOffset().torque.z <<"," << \
                         interface.getForceSensRightFoot().force.x <<"," << interface.getForceSensRightFoot().force.y <<"," << interface.getForceSensRightFoot().force.z <<"," << \
                         interface.getForceSensRightFoot().torque.x <<"," << interface.getForceSensRightFoot().torque.y <<"," << interface.getForceSensRightFoot().torque.z <<"," << \
                         interface.getForceSensRightFootOffset().force.x <<"," << interface.getForceSensRightFootOffset().force.y <<"," << interface.getForceSensRightFootOffset().force.z <<"," << \
                         interface.getForceSensRightFootOffset().torque.x <<"," << interface.getForceSensRightFootOffset().torque.y <<"," << interface.getForceSensRightFootOffset().torque.z << std::endl;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
