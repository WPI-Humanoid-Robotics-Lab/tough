#pragma once

/*******************************************************
 * !!!!! important other wise created collision with std
 * *****************************************************/
#define DISABLE_DECISION_MAKING_LOG true

#include <iostream>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <ros/ros.h>
#include <decision_making/FSM.h>
#include <decision_making/ROSTask.h>
#include <decision_making/DecisionMaking.h>
#include <tough_footstep/RobotWalker.h>
#include <nav_msgs/OccupancyGrid.h>

#include "src_task_common/val_walk_tracker.h"
#include "src_task_common/panel_detection.h"
#include "tough_controller_interface/chest_control_interface.h"
#include "tough_controller_interface/pelvis_control_interface.h"
#include "tough_controller_interface/head_control_interface.h"
#include "tough_controller_interface/gripper_control_interface.h"
#include "tough_controller_interface/robot_state.h"
#include "src_task2/val_rover_detection.h"
#include "src_task2/val_solar_detection.h"
#include "src_task2/solar_panel_grasp.h"
#include "src_task2/button_detector.h"
#include "src_task2/array_table_detector.h"
#include "src_task2/cable_detector.h"
#include <src_task2/val_task2_utils.h>
#include <src_task2/button_press.h>
#include <src_task2/cable_task.h>
#include "src_task2/plug_detector.h"
#include "src_task_common/finish_box_detector.h"
#include "navigation_common/map_generator.h"
#include "perception_common/MultisensePointCloud.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>

using namespace decision_making;

#define foreach BOOST_FOREACH

FSM_HEADER(src_task2)

class valTask2 {

    private:
    ros::NodeHandle nh_;
    // default constructor
    valTask2(ros::NodeHandle nh);
    //utils
    task2Utils* task2_utils_;

    // object for the walker api
    RobotWalker* walker_;
    // object for tracking robot walk
    walkTracking* walk_track_;


    //Rover detector
    RoverDetector* rover_detector_;
    //Rover detector
    RoverDetector* rover_detector_fine_;
    // Block rover in /map
    SolarArrayDetector* rover_in_map_blocker_;
    //solar panel detector
    SolarPanelDetect* solar_panel_detector_;
    // panel grabber
    solar_panel_handle_grabber* panel_grabber_;
    // Solar array detector is also used for blocking map.
    SolarArrayDetector* solar_array_detector_;
    // Solar array detector is also used for blocking map.
    ArrayTableDetector* solar_array_fine_detector_;
    // button detector
    ButtonDetector* button_detector_;
    // cable detector
    CableDetector* cable_detector_;
    // Button press
    ButtonPress* button_press_;
    // Pick Cable Task
    CableTask* cable_task_;
    // socket detector
    SocketDetector* socket_detector_;
    // finish box detector
    FinishBoxDetector* finish_box_detector_;

    // chest controller
    chestTrajectory* chest_controller_;
    //pelvis controller
    pelvisTrajectory* pelvis_controller_;
    //head controller
    HeadTrajectory* head_controller_;
    //grippers
    gripperControl* gripper_controller_;
    //arm controllers
    ArmControlInterface*  arm_controller_;

    //robot state informer
    RobotStateInformer* robot_state_;

    RobotDescription *rd_;
    valControlCommon* control_common_;

    ros::Subscriber occupancy_grid_sub_;
    ros::Subscriber nudge_sub_;
    unsigned int map_update_count_;
    void occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg);
    void nudge_pose_cb(const std_msgs::Float64MultiArray msg);

    // goal location for the panel
    geometry_msgs::Pose2D panel_walk_goal_;

    std::vector<geometry_msgs::Pose2D> rover_walk_goal_waypoints_;
    bool is_rover_on_right_;

    //solar panel handle grasp pose
    geometry_msgs::Pose solar_panel_handle_pose_;

    geometry_msgs::Pose2D solar_array_walk_goal_;
    geometry_msgs::Pose2D solar_array_fine_walk_goal_;
    bool is_array_on_right_;

    ros::Subscriber panel_handle_offset_sub_;
    ros::Publisher  panel_handle_offset_pub_;
    void panelHandleOffsetCB(const std_msgs::Float32 msg);
    geometry_msgs::Point button_coordinates_;
    geometry_msgs::Point button_coordinates_temp_; // this is used for deciding if rotation of panel is required

    geometry_msgs::Point socket_coordinates_;

    geometry_msgs::Point cable_point_temp_; // used to array detection
    geometry_msgs::Pose cable_pose_;

    RobotSide panel_grasping_hand_;

    void initControllers();
    void initDetectors();

    static valTask2* currentObject;

    ros::Subscriber set_panel_rotation_flag_sub_;
    void setPanelRotationFlagCB(const std_msgs::Bool msg);
    bool is_rotation_required_;
    // Visited map
    ros::Subscriber visited_map_sub_;
    void visited_map_cb(const nav_msgs::OccupancyGrid::Ptr msg);
    nav_msgs::OccupancyGrid visited_map_;

    geometry_msgs::Pose2D next_finishbox_center_;

    tough_perception::MultisenseImage* ms_sensor_;

    // skip checkpoint global variables
    bool skip_3, skip_4, skip_6 ;

    public:

    // default destructor
    ~valTask2();
    static valTask2* getValTask2(ros::NodeHandle nh);
    bool preemptiveWait(double ms, decision_making::EventQueue& queue);
    decision_making::TaskResult pre_initTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult initTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToRoverTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult graspPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult pickPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectSolarArrayFineTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult findCableIntermediateTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult alignSolarArrayTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult placePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectButtonTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult deployPanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult pickCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectSocketTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult plugCableTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult detectFinishBoxTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult endTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult rotatePanelTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult skipCheckPointTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult skipToCP3Task(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult skipToCP4Task(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult skipToCP6Task(string name, const FSMCallContext& context, EventQueue& eventQueue);
    decision_making::TaskResult manualExecutionTask(string name, const FSMCallContext& context, EventQueue& eventQueue);
    geometry_msgs::Pose2D getPanelWalkGoal();

    void setRoverWalkGoal(const std::vector<geometry_msgs::Pose2D> &rover_walk_goal);
    void setRoverSide(const bool isRoverOnRight);
    void setSolarPanelHandlePose(const geometry_msgs::Pose &pose);
    void setPanelWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal);
    void setSolarArrayWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal);
    void setSolarArrayFineWalkGoal(const geometry_msgs::Pose2D &panel_walk_goal);
    void setSolarArraySide(const bool isSolarArrayOnRight);
    void setPanelGraspingHand(RobotSide side);
    void setIsRotationRequired(bool value);
    void setTempCablePoint(geometry_msgs::Point value);
};
