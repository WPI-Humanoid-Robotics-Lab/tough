#include <val_task1/move_handle.h>

move_handle::move_handle(ros::NodeHandle n) : nh_(n) {
    right_armTraj = new armTrajectory(nh_);
    left_armTraj = new armTrajectory(nh_);
    array_pub = nh_.advertise<visualization_msgs::MarkerArray>( "handle_path", 10, true);
    robot_state_ = RobotStateInformer::getRobotStateInformer(nh_);
}

void move_handle::createCircle(geometry_msgs::Point center, int side, const std::vector<float> planeCoeffs, std::vector<geometry_msgs::Pose> &points)
{
  float radius = .20,num_steps = 4;
  float hand_z_plane = 0;
  float dist = 0;
  float spin = M_PI/4; //temp
  float hand_angle;
  std::vector<double> path;
  armSide input_side;

  if (planeCoeffs.size() != 4){
      ROS_INFO("Please check the plane coefficiants");
  }

  float a = planeCoeffs.at(0);
  float b = planeCoeffs.at(1);
  float c = planeCoeffs.at(2);
  float d = planeCoeffs.at(3);

  geometry_msgs::Pose cen;
  cen.position.x = center.x;
  cen.position.y = center.y;
  cen.position.z = center.z;


  geometry_msgs::Pose current_hand_pose;
  geometry_msgs::Pose finger_pose;

  // get the hand frame
  if (side){
    robot_state_->getCurrentPose("rightMiddleFingerPitch1Link",finger_pose);
    input_side = armSide::RIGHT;
  }
  else{
    robot_state_->getCurrentPose("leftMiddleFingerPitch1Link",finger_pose);
    input_side = armSide::LEFT;
   }

  //starting angle
  hand_angle = M_PI + atan2(cen.position.y - finger_pose.position.y,cen.position.x - finger_pose.position.x );
  //geterate a traj from start angle to the finish angle
  path = linspace(hand_angle,spin, num_steps);

  dist = fabs( a*finger_pose.position.x  + b*finger_pose.position.y + c*finger_pose.position.z  + d )/sqrt( pow(a,2) + pow(b,2) + pow(c,2) );


//  std::vector<geometry_msgs::Pose> points;
  for (int i=0; i<path.size(); i++)
  {
    // circle parametric equation
    geometry_msgs::Pose point;
    point.position.x = center.x + (radius * cos( path[i] ));
    point.position.y = center.y + (radius * sin( path[i] ));

    float dir=atan2(sin((float)i*(2*M_PI/num_steps)),cos((float)i*(2*M_PI/num_steps)))+M_PI_2;
    tf::Quaternion t =tf::createQuaternionFromYaw(dir);
    point.orientation.w=t.w();
    point.orientation.x=t.x();
    point.orientation.y=t.y();
    point.orientation.z=t.z();
    std::cout<<"direction:  "<<dir<<std::endl;

    point.position.z = -(a*point.position.x  + b* point.position.y + (d - dist+.05) )/c   ;
    points.push_back(point);
  }
  visulatize(points);
//  follow_path(points, input_side,finger_pose);

}

void move_handle::follow_path(std::vector<geometry_msgs::Pose>& points, armSide input_side, geometry_msgs::Pose hand)
{

    std::vector<armTrajectory::armTaskSpaceData> *arm_data_vector = new std::vector<armTrajectory::armTaskSpaceData>();
    // for(auto input_pose : *input_poses)
    float desired_time = 10.0;
    int num_poses = points.size();
    for(int i=0;i<num_poses;i++)
    {
        geometry_msgs::Pose input_pose = points.at(i);
        armTrajectory::armTaskSpaceData* task_space_data = new armTrajectory::armTaskSpaceData();
        task_space_data->side = input_side;
        task_space_data->pose.position = input_pose.position;
        task_space_data->pose.orientation = hand.orientation;

        task_space_data->time = (i/num_poses)*desired_time; //what xyzzy
        arm_data_vector->push_back(*task_space_data);
        ROS_INFO_STREAM(task_space_data->pose);
    }
    if ( input_side == armSide::RIGHT )
        right_armTraj->moveArmInTaskSpace(*arm_data_vector);
    else
        left_armTraj->moveArmInTaskSpace(*arm_data_vector);

    ROS_INFO("Moved Handle");

}
//create a linespace vector 
std::vector<double> move_handle::linspace(double min, double max, int n)
{
    std::vector<double> result;
    int iterator = 0;

    for (int i = 0; i <= n-2; i++)
    {
     double temp = min + i*(max-min)/(floor((double)n) - 1);
     result.insert(result.begin() + iterator, temp);
     iterator += 1;
    }

    result.insert(result.begin() + iterator, max);
    return result;
}
void move_handle::visulatize(std::vector<geometry_msgs::Pose> &points)
{
  // visulation of the circle
  visualization_msgs::MarkerArray circle = visualization_msgs::MarkerArray();

  visualization_msgs::Marker marker;
  marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
  marker.header.stamp = ros::Time();
  marker.ns = "circle";
  marker.id = 0;
  marker.type = visualization_msgs::Marker::CUBE;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose = points[0];
  marker.scale.x = 0.1;
  marker.scale.y = 0.1;
  marker.scale.z = 0.1;
  marker.color.a = 0.6;
  marker.color.r = 1.0;
  marker.color.g = 0.0;
  marker.color.b = 0.0;
  marker.lifetime = ros::Duration(0);
  circle.markers.push_back(marker);

    for (int i = 1; i < points.size(); i++) {
        visualization_msgs::Marker marker;
        marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
        marker.header.stamp = ros::Time();
        marker.ns = "circle";
        marker.id = i;
        marker.type = visualization_msgs::Marker::ARROW;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose = points[i];

        ROS_INFO_STREAM(i<<"pose"<<points[i].position);
        marker.scale.x = 0.1;
        marker.scale.y = 0.01;
        marker.scale.z = 0.01;
        marker.color.a = 0.6;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;
        marker.lifetime = ros::Duration(0);
        circle.markers.push_back(marker);
    }

    array_pub.publish( circle );
    ros::Duration(0.2).sleep();
    ROS_INFO("vis");
}
