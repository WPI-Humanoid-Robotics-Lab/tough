#include <val_task1/val_task1_utils.h>
#include <std_msgs/Int8.h>

task1Utils::task1Utils(ros::NodeHandle nh):
    nh_(nh)
{
    // subscriber for the satellite message
    satellite_sub_ = nh_.subscribe("/task1/checkpoint2/satellite", 10, &task1Utils::satelliteMsgCB, this);

    // marker publisher
    marker_pub_ = nh_.advertise<visualization_msgs::MarkerArray>( "handle_path", 10, true);

    clearbox_pointcloud_pub_ = nh_.advertise<std_msgs::Int8>("/field/clearbox_pointcloud",1);
    reset_pointcloud_pub_    = nh_.advertise<std_msgs::Empty>("/field/reset_pointcloud",1);
    task1_log_pub_           = nh_.advertise<std_msgs::String>("/field/log",10);

    task_status_sub_ = nh.subscribe("/srcsim/finals/task1", 10, &task1Utils::taskStatusSubCB, this);
    visitedMapUpdaterSub_    = nh_.subscribe("/visited_map", 10, &task1Utils::visitedMapUpdateCB, this);

    logFile = ros::package::getPath("val_task1") + "/log/task1.csv";

    current_state_ = RobotStateInformer::getRobotStateInformer(nh_);
    walk_          = new ValkyrieWalker(nh_, 0.7, 0.7, 0, 0.18);
    current_checkpoint_ = 0;

    timeNow = boost::posix_time::second_clock::local_time();

    timer_ = nh_.createTimer(ros::Duration(30*60), &task1Utils::terminator, this);
}

task1Utils::~task1Utils()
{
    satellite_sub_.shutdown();
}

int task1Utils::getCurrentCheckpoint() const{
    return current_checkpoint_;
}

bool task1Utils::isPointVisited(float x, float y)
{
    size_t index = MapGenerator::getIndex(x, y);
    return visited_map_.data.at(index) == CELL_STATUS::VISITED;
}

bool task1Utils::getNextPoseToWalk(geometry_msgs::Pose2D &pose2D, bool allowVisitied)
{
    if (visited_map_.data.empty())
        return false;
    const float radius = 2.0f;

    for (float theta = 0; theta < M_PI; theta += M_PI/6){
        for (int i = -1; i < 2; i+=2){
            pose2D.x = radius*cos(i*theta);
            pose2D.y = radius*sin(i*theta);
            pose2D.theta = i*theta;
            current_state_->transformPose(pose2D, pose2D, VAL_COMMON_NAMES::PELVIS_TF, VAL_COMMON_NAMES::WORLD_TF);
            size_t index = MapGenerator::getIndex(pose2D.x, pose2D.y);
            if (visited_map_.data.at(index) == CELL_STATUS::FREE || (allowVisitied && visited_map_.data.at(index) == CELL_STATUS::VISITED)){
                ROS_INFO("task1Utils::getNextPoseToWalk : x:%f y:%f theta:%f",i, pose2D.x, pose2D.y, pose2D.theta);
                return true;
            }
        }
    }

    pose2D.x = 0;
    pose2D.y = 0;
    pose2D.theta = 0;

    return false;
}

void task1Utils::reOrientTowardsGoal(geometry_msgs::Point goal_point, float offset){

    size_t nSteps;
    armSide startStep;
    std::vector<float> y_offset;
    std::vector<float> x_offset;

    current_state_->transformPoint(goal_point,goal_point,VAL_COMMON_NAMES::WORLD_TF,
                                   VAL_COMMON_NAMES::PELVIS_TF);

    double error = goal_point.y+offset;
    double abserror = std::fabs(error);
    ROS_INFO_STREAM("Error is:" << error << "Absolute value for error is:" << abserror);
    if (abserror < 0.1 || abserror > 0.49 ){
        ROS_INFO("reOrientTowardsPanel: Not reorienting, the offset is too less or beyond control");
    }
    else
    {
        nSteps = int(((abserror/0.1)+0.5));
        ROS_INFO_STREAM("No of steps to walk is:" << nSteps);

        if (error > 0){
            startStep = LEFT;
            for(size_t i = 0; i < nSteps; ++i){
                y_offset.push_back(0.1);
                y_offset.push_back(0.1);
            }
        }
        else {
            startStep = RIGHT;
            for(size_t i = 0; i < nSteps; ++i){
                y_offset.push_back(-0.1);
                y_offset.push_back(-0.1);
            }
        }

        x_offset.resize(y_offset.size());
        ROS_INFO("reOrientTowardsPanel: Walking %d steps",int(nSteps));

        walk_->walkLocalPreComputedSteps(x_offset,y_offset,startStep);
        ros::Duration(4.0).sleep();

    }

}


void task1Utils::satelliteMsgCB(const srcsim::Satellite& msg)
{
    // update the message
    msg_ = msg;
}

void task1Utils::visitedMapUpdateCB(const nav_msgs::OccupancyGrid &msg)
{
    mtx.lock();
    visited_map_ = msg;
    mtx.unlock();
}

// satellite dish helper tasks
bool task1Utils::isPitchCorrectNow(void)
{
    return msg_.pitch_correct_now;
}

bool task1Utils::isYawCorrectNow(void)
{
    return msg_.yaw_correct_now;
}

bool task1Utils::isPitchCompleted(void)
{
    return msg_.pitch_completed;
}

bool task1Utils::isYawCompleted(void)
{
    return msg_.yaw_completed;
}

double task1Utils::getPitchDiff (void)
{
    return (msg_.target_pitch - msg_.current_pitch);
}

double task1Utils::getYawDiff (void)
{
    return (msg_.target_yaw - msg_.current_yaw);
}

double task1Utils::getPitch (void)
{
    return msg_.current_pitch;
}

double task1Utils::getYaw (void)
{
    return msg_.current_yaw;
}

valueDirection task1Utils::getGoalDirection(double current_value, controlSelection control)
{
    valueDirection ret;

    //decide the direction to move
    double goal = (control == controlSelection::CONTROL_PITCH) ? msg_.target_pitch : msg_.target_yaw;
    if (goal > current_value)
    {
        // we should increase the value
        ret = valueDirection::VALUE_INCRSING;
    }
    else
    {
        // decrease the alue
        ret = valueDirection::VALUE_DECRASING;
    }

    return ret;
}

valueDirection task1Utils::getValueStatus(double current_value, controlSelection control)
{
    valueDirection ret = valueDirection::VALUE_NOT_INITIALISED;
    // required direction
    static valueDirection required_direction = valueDirection::VALUE_NOT_INITIALISED;
    static controlSelection prev_control = controlSelection::CONTROL_NOT_INITIALISED;
    static bool execute_once = true;
    static double prev_trigger_value = 0;

    //timer
    static std::chrono::system_clock::time_point debounce_timer = std::chrono::system_clock::now();

    // flag to indicate to elapse the timer
    bool is_timer_being_checked = false;

    // on init or if the control slection is changed
    // set the trigger value
    if (execute_once || (control != prev_control))
    {
        // initialise previous value and return
        prev_trigger_value = current_value;

        //decide the direction to move
        required_direction = getGoalDirection(current_value, control);

        //reset the flah
        execute_once = false;

        ROS_INFO("initialised");

        //reset timer
        debounce_timer = std::chrono::system_clock::now();
    }
    // if value is not changed (changed with in a threshold of 1 degree)
    else if (((current_value > prev_trigger_value) && ((current_value - prev_trigger_value) <= HANDLE_CONSTANT_THRESHOLD_IN_RAD)) ||
             ((prev_trigger_value > current_value) && ((prev_trigger_value - current_value) <= HANDLE_CONSTANT_THRESHOLD_IN_RAD)) ||
             (prev_trigger_value == current_value))
    {
        // set the flag that timer is being checked
        is_timer_being_checked = true;

        //        ROS_INFO("time debounce %f sec", std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - debounce_timer).count() );
        ROS_INFO_THROTTLE(1, "valu constant debounce");

        if (std::chrono::duration_cast<std::chrono::seconds>(std::chrono::system_clock::now() - debounce_timer).count() > HANDLE_CONSTANT_DEBOUNCE_TIME_SEC){

            // update return value as constant
            ret = valueDirection::VALUE_CONSTANT;

            //reset the flag
            is_timer_being_checked = false;

            // update the trigger value
            execute_once =  true;
            ROS_INFO("value is not changing");
        }
    }
    // if the handels are moving, wait till it moves at least by 4 degree
    else if (((current_value > prev_trigger_value) && (current_value - prev_trigger_value > HANDLE_MINIMUM_MOVMENT_IN_RAD)) ||
             ((prev_trigger_value > current_value) && (prev_trigger_value - current_value > HANDLE_MINIMUM_MOVMENT_IN_RAD)))
    {
        ROS_INFO("moved by 4deg");

        if ((required_direction == valueDirection::VALUE_INCRSING && current_value > prev_trigger_value) ||
                (required_direction == valueDirection::VALUE_DECRASING && current_value < prev_trigger_value))
        {
            ret = valueDirection::VALUE_TOWARDS_TO_GOAL;
            ROS_INFO("towards goal");
        }
        else
        {
            ret = valueDirection::VALUE_AWAY_TO_GOAL;
            ROS_INFO("away from goal");
        }

        // set the once flag for next time
        execute_once = true;
    }
    else
    {
        //ROS_INFO("toggling");
        ret = valueDirection::VALUE_TOGGLING;
    }

    //update the previous control
    prev_control = control;

    // reset the timer if its not being checked
    if (is_timer_being_checked == false)
    {
        debounce_timer = std::chrono::system_clock::now();
    }

    return ret;
}

void task1Utils::getCircle3D(geometry_msgs::Point center, geometry_msgs::Point start, geometry_msgs::Quaternion orientation, const std::vector<float> planeCoeffs, std::vector<geometry_msgs::Pose> &points, handleDirection direction, float radius, int steps)
{
    if (planeCoeffs.size() != 4){
        ROS_INFO("Please check the plane coefficiants");
    }

    // plane co-effecients
    float a = planeCoeffs.at(0);
    float b = planeCoeffs.at(1);
    float c = planeCoeffs.at(2);
    float d = planeCoeffs.at(3);

    // clear the points
    points.clear();

    // starting point
    geometry_msgs::Pose circle_point_pose;
    circle_point_pose.position.x = start.x;
    circle_point_pose.position.y = start.y;
    circle_point_pose.position.z = start.z;
    circle_point_pose.orientation = orientation;
    points.push_back(circle_point_pose);

    // diatnce from plane to the handle
    float dist = fabs(a*start.x  + b*start.y + c*start.z  + d )/sqrt(pow(a,2) + pow(b,2) + pow(c,2));

    geometry_msgs::Pose start_pose;
    start_pose.position.x = start.x;
    start_pose.position.y = start.y;
    start_pose.position.z = start.z;

    for (int i=0;i<steps; i++)
    {
        // angle to the first point
        double y = (start_pose.position.y - center.y);
        double x = (start_pose.position.x - center.x);
        ROS_INFO("y %f, x %f", y, x);
        float alpha = atan2(y,x);
        ROS_INFO("alpha %f ", alpha);

        if (direction == handleDirection::CLOCK_WISE) {
            circle_point_pose.position.x = center.x + radius*cos(alpha - (float)(2*M_PI/steps));
            circle_point_pose.position.y = center.y + radius*sin(alpha - (float)(2*M_PI/steps));
        }
        else if (direction == handleDirection::ANTICLOCK_WISE) {
            circle_point_pose.position.x = center.x + radius*cos(alpha + (float)(2*M_PI/steps));
            circle_point_pose.position.y = center.y + radius*sin(alpha + (float)(2*M_PI/steps));
        }

        //point.position.z = -(a*point.position.x  + b* point.position.y + d)/c;
        circle_point_pose.position.z = -(a*circle_point_pose.position.x  + b* circle_point_pose.position.y + d)/c + dist; //(d - dist+.05) )/c;
        circle_point_pose.position.z -= 0.06;

        // orientation
        circle_point_pose.orientation = orientation;
        points.push_back(circle_point_pose);

        start_pose = circle_point_pose;
    }
}

void task1Utils::visulatise6DPoints(std::vector<geometry_msgs::Pose> &points)
{
    // visulation of the circle
    visualization_msgs::MarkerArray marker_array = visualization_msgs::MarkerArray();

    visualization_msgs::Marker marker;
    marker.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
    marker.header.stamp = ros::Time();
    marker.ns = "circle";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose = points[0];
    marker.scale.x = 0.1;
    marker.scale.y = 0.01;
    marker.scale.z = 0.01;
    marker.color.a = 0.6;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
    marker.lifetime = ros::Duration(0);
    marker_array.markers.push_back(marker);

    for (int i = 1; i < points.size(); i++) {
        visualization_msgs::Marker marker_tangent;
        marker_tangent.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
        marker_tangent.header.stamp = ros::Time();
        marker_tangent.ns = "circle";
        marker_tangent.id = i;
        marker_tangent.type = visualization_msgs::Marker::ARROW;
        marker_tangent.action = visualization_msgs::Marker::ADD;
        marker_tangent.pose = points[i];

        //        ROS_INFO_STREAM(i<<"pose"<<points[i].position);
        marker_tangent.scale.x = 0.05;
        marker_tangent.scale.y = 0.006;
        marker_tangent.scale.z = 0.006;
        marker_tangent.color.a = 0.6;
        marker_tangent.color.r = ((i==1) ? 1.0: 0.0);
        marker_tangent.color.g = ((i==1) ? 0.0: 1.0);
        marker_tangent.color.b = 0.0;
        marker_tangent.lifetime = ros::Duration(0);
        marker_array.markers.push_back(marker_tangent);

        visualization_msgs::Marker marker_point;
        marker_point.header.frame_id = VAL_COMMON_NAMES::WORLD_TF;
        marker_point.header.stamp = ros::Time();
        marker_point.ns = "circle";
        marker_point.id = i+points.size();
        marker_point.type = visualization_msgs::Marker::CUBE;
        marker_point.action = visualization_msgs::Marker::ADD;
        marker_point.pose = points[i];

        //        ROS_INFO_STREAM(i<<"pose"<<points[i].position);
        marker_point.scale.x = 0.005;
        marker_point.scale.y = 0.005;
        marker_point.scale.z = 0.005;
        marker_point.color.a = 0.6;
        marker_point.color.r = 0.0;
        marker_point.color.g = .0;
        marker_point.color.b = 1.0;
        marker_point.lifetime = ros::Duration(0);
        marker_array.markers.push_back(marker_point);

    }

    marker_pub_.publish(marker_array);
    ros::Duration(0.2).sleep();
}

void task1Utils::fixHandleArray(std::vector<geometry_msgs::Point> &handle_loc, std::vector<geometry_msgs::Point> &pclHandlePoses){

    geometry_msgs::Point temp;
    ROS_INFO_STREAM("Handle Locs before "<< handle_loc[0] << "\t" << handle_loc[1] << "\t" << handle_loc[2]<<"\t" << handle_loc[3]<< std::endl);
    double norm1_handle = std::sqrt(std::pow(handle_loc[1].x - pclHandlePoses[1].x , 2) + std::pow(handle_loc[1].y - pclHandlePoses[1].y , 2) + std::pow(handle_loc[1].z - pclHandlePoses[1].z , 2));
    double norm1_centre = std::sqrt(std::pow(handle_loc[0].x - pclHandlePoses[1].x , 2) + std::pow(handle_loc[0].y - pclHandlePoses[1].y , 2) + std::pow(handle_loc[0].z - pclHandlePoses[1].z , 2));
    double norm2_handle = std::sqrt(std::pow(handle_loc[3].x - pclHandlePoses[0].x , 2) + std::pow(handle_loc[3].y - pclHandlePoses[0].y , 2) + std::pow(handle_loc[3].z - pclHandlePoses[0].z , 2));
    double norm2_centre = std::sqrt(std::pow(handle_loc[2].x - pclHandlePoses[0].x , 2) + std::pow(handle_loc[2].y - pclHandlePoses[0].y , 2) + std::pow(handle_loc[2].z - pclHandlePoses[0].z , 2));

    ROS_INFO_STREAM("Left handle norm :  "<< norm1_handle << std::endl << " Right Handle Norm : " <<norm2_handle <<std::endl << " Left Centre Norm: "
                    << norm1_centre << std::endl << "Right Centre Norm : " << norm2_centre << std::endl);

    if(norm1_handle > 0.05 && norm1_centre < 0.05)
    {
        temp = handle_loc[1];
        handle_loc[1] = handle_loc[0];
        handle_loc[0] = temp;
    }

    if(norm2_handle > 0.05 && norm2_centre < 0.05)
    {
        temp = handle_loc[3];
        handle_loc[3] = handle_loc[2];
        handle_loc[2] = temp;
    }

    const geometry_msgs::Point &pitchHandle = handle_loc[PITCH_KNOB_HANDLE];
    geometry_msgs::Point &pitchCenter = handle_loc[PITCH_KNOB_CENTER];

    double y = (pitchHandle.y - pitchCenter.y);
    double x = (pitchHandle.x - pitchCenter.x);
    float theta = atan2(y,x);
    float currentDistance = sqrt(pow((pitchHandle.x - pitchCenter.x), 2) + pow((pitchHandle.y - pitchCenter.y),2));
    float diff = 0.125 - currentDistance;
    if (diff > 0){
        pitchCenter.x += diff*cos(theta);
        pitchCenter.y += diff*sin(theta);
    }

    const geometry_msgs::Point &yawHandle = handle_loc[YAW_KNOB_HANDLE];
    geometry_msgs::Point &yawCenter = handle_loc[YAW_KNOB_CENTER];
    y = (yawHandle.y - yawCenter.y);
    x = (yawHandle.x - yawCenter.x);
    theta = atan2(y,x);
    currentDistance = sqrt(pow((yawHandle.x - yawCenter.x), 2) + pow((yawHandle.y - yawCenter.y),2));
    diff = 0.125 - currentDistance;
    if (diff > 0){
        yawCenter.x += diff*cos(theta);
        yawCenter.y += diff*sin(theta);
    }
}

void task1Utils::clearBoxPointCloud()
{
    std_msgs::Int8 msg;
    msg.data = 0;
    clearbox_pointcloud_pub_.publish(msg);
}

void task1Utils::resetPointCloud()
{
    std_msgs::Empty msg;
    reset_pointcloud_pub_.publish(msg);
}

void task1Utils::taskStatusSubCB(const srcsim::Task &msg){

    taskMsg = msg;
    if (msg.current_checkpoint != current_checkpoint_){
        current_checkpoint_ = msg.current_checkpoint;

        std::ofstream outfile(logFile, std::ofstream::app);
        std::stringstream data;

        data << boost::posix_time::to_simple_string(timeNow) << "," << msg.task << ","
             << msg.current_checkpoint << "," << msg.elapsed_time << std::endl;
        ROS_INFO("task1Utils::taskStatusCB : Current checkpoint : %d", current_checkpoint_);

        outfile << data.str();
        outfile.close();
    }
}


void task1Utils::terminator(const ros::TimerEvent& e){

    ROS_INFO("Killing! Kill! Kill! Fire in the Hole! Headshot!");
    int status = system("killall roscore rosmaster rosout gzserver gzclient roslaunch");
}

void task1Utils::taskLogPub(std::string data){

    std_msgs::String ms;
    ms.data = data;
    task1_log_pub_.publish(ms);
}
