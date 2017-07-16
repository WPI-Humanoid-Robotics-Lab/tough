#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

int main(int argc, char **argv)
{

  ros::init(argc, argv, "state_publisher");

  ros::NodeHandle n;

  ros::Publisher joint_pub=n.advertise<sensor_msgs::JointState>("hardware_joint_commands",1000);

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
      sensor_msgs::JointState joint;

      joint.header.seq = 0;
      joint.header.stamp= ros::Time::now();
      joint.header.frame_id="/pelvis";
      joint.name.resize(58);
      joint.name={"torsoYaw", "torsoPitch", "torsoRoll", "leftHipYaw", "leftHipRoll", "leftHipPitch", "leftKneePitch", "leftAnklePitch", "leftAnkleRoll", "rightHipYaw", "rightHipRoll", "rightHipPitch", "rightKneePitch", "rightAnklePitch", "rightAnkleRoll", "leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch", "rightShoulderPitch", "rightShoulderRoll", "rightShoulderYaw", "rightElbowPitch", "rightForearmYaw", "rightWristRoll", "rightWristPitch", "lowerNeckPitch", "neckYaw", "upperNeckPitch", "leftThumbRoll", "leftThumbPitch1", "leftThumbPitch2", "leftThumbPitch3", "rightThumbRoll", "rightThumbPitch1", "rightThumbPitch2", "rightThumbPitch3", "leftIndexFingerPitch1", "leftIndexFingerPitch2", "leftIndexFingerPitch3", "rightIndexFingerPitch1", "rightIndexFingerPitch2", "rightIndexFingerPitch3", "leftMiddleFingerPitch1", "leftMiddleFingerPitch2", "leftMiddleFingerPitch3", "rightMiddleFingerPitch1", "rightMiddleFingerPitch2", "rightMiddleFingerPitch3", "leftPinkyPitch1", "leftPinkyPitch2", "leftPinkyPitch3", "rightPinkyPitch1", "rightPinkyPitch2", "rightPinkyPitch3"};
      joint.position.resize(58);
      joint.position= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.5, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      joint.velocity.resize(58);
      joint.velocity= {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
      joint.effort.resize(58);
      joint.effort ={0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};


      joint_pub.publish(joint);

      loop_rate.sleep();

  }
  return 0;
}
