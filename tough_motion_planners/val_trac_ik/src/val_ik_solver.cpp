#include <val_manipulation/val_manipulation.h>

int main(int argc, char** argv)
{
  srand(1);
  ros::init(argc, argv, "val_track_ik");
  ros::NodeHandle nh("~");
  ValManipulation ik_solver(nh);
  tf::StampedTransform transform;
  tf::TransformListener listener;
  KDL::JntArray result;

  bool loop = 1;
  int num_samples;
  std::string chain_start, chain_end, urdf_param;
  double timeout;

  nh.param("num_samples", num_samples, 20);
  nh.param("chain_start", chain_start, std::string("torso"));
  nh.param("chain_end", chain_end, std::string("rightPalm"));

  if (chain_start == "" || chain_end == "")
  {
    ROS_FATAL("Missing chain info in launch file");
    exit(-1);
  }

  nh.param("timeout", timeout, 5.0);
  nh.param("urdf_param", urdf_param, std::string("/robot_description"));

  if (num_samples < 1)
    num_samples = 1;
  //    while(ros::ok() && loop==1) {
  //        try {
  //            //ROS_INFO("trying");
  //            listener.waitForTransform("/buttonFrame", "/torso",ros::Time(0), ros::Duration(10.0));
  //            listener.lookupTransform("/buttonFrame", "/torso", ros::Time(0), transform);
  //            loop=0;
  //        }
  //        catch (tf::TransformException ex) {
  //           ROS_ERROR("%s",ex.what());
  //           ros::Duration(0.3).sleep();
  //        }
  //    }

  //    Translation: [0.836, -1.850, -2.895]
  //    Rotation: in Quaternion [0.532, 0.532, 0.466, -0.466]
  //              in RPY (radian) [3.142, -1.440, 1.571]
  //              in RPY (degree) [179.998, -82.500, 90.001]

  tf::Point point;
  point.setX(0.8);
  point.setY(0.4);
  point.setZ(1.7);  //(-2.895);
  tf::Quaternion q;
  q.setX(0.0);
  q.setY(0.0);
  q.setZ(0.0);
  q.setW(1.0);

  transform.setOrigin(point);
  transform.setRotation(q);

  ik_solver.solve_ik(num_samples, chain_start, chain_end, timeout, urdf_param, transform, result);

  // Useful when you make a script that loops over multiple launch files that test different robot chains
  // std::vector<char *> commandVector;
  // commandVector.push_back((char*)"killall");
  // commandVector.push_back((char*)"-9");
  // commandVector.push_back((char*)"roslaunch");
  // commandVector.push_back(NULL);

  // char **command = &commandVector[0];
  // execvp(command[0],command);

  return 0;
}
