#include <iostream>
#include <time.h>
#include <numeric>
#include <boost/bind.hpp>
#include <boost/thread.hpp>
#include <boost/foreach.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/date_time.hpp>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <val_task3/val_task3.h>
#include <srcsim/StartTask.h>
#include <queue>


valTask3* valTask3::currentObject = nullptr;

valTask3* valTask3::getValTask3(ros::NodeHandle nh){

    if  (currentObject = nullptr){
         currentObject = new valTask3(nh);
         return currentObject;
    }

    ROS_ERROR("Object already exists");
    assert(false && "Object already exists");
}


valTask3::valTask3(ros::NodeHandle nh):nh_(nh){

    walker_            = new ValkyrieWalker(nh_,0.7,0.7,0,0.18);
    pelvis_controller_ = new pelvisTrajectory(nh_);
    walk_track_        = new walkTracking(nh_);

    task3_utils_       = new task3Utils(nh_);
    robot_state_       = RobotStateInformer::getRobotStateInformer(nh_);
    map_update_count_  = 0;
    occupancy_grid_sub_= nh_.subscribe("/map",10,&valTask3::occupancy_grid_cb,this);
}

valTask3::~valTask3(){

    delete walker_;
    delete pelvis_controller_;
    delete walk_track_;
    delete task3_utils_;
}

void valTask3::occupancy_grid_cb(const nav_msgs::OccupancyGrid::Ptr msg){

    ++map_update_count_;
}

bool valTask3::preemptiveWait(double ms, decision_making::EventQueue &queue){

    for(size_t i = 0; i<100 && !queue.isTerminated(); ++i){
        boost::this_thread::sleep(boost::posix_time::milliseconds(ms/100.0));
    }

    return queue.isTerminated();
}


decision_making::TaskResult valTask3::initTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectStairsTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::climbStairsTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectDoorHandleTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::openDoorTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkthroughDoorTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectLeakToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToLeakToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::pickLeakTool(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToDetectLeak(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectLeakTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

    ROS_INFO_STREAM("executing " << name);

    while(!preemptiveWait(1000, eventQueue)){

        ROS_INFO("waiting for transition");
    }

    return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectRepairToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToRepairToolTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::pickRepairTool(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToLeakTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::leakRepairTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToTableTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::detectFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::walkToFinishTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::endTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}

decision_making::TaskResult valTask3::errorTask(string name, const FSMCallContext& context, EventQueue& eventQueue){

  ROS_INFO_STREAM("executing " << name);

  while(!preemptiveWait(1000, eventQueue)){

      ROS_INFO("waiting for transition");
  }

  return TaskResult::SUCCESS();
}























//enum sm {
//    PREPARE_START = 0,
//    CLIMB_STAIRS,
//    OPEN_DOOR,
//    WALK_THROUGH_DOOR,
//    WALK_TO_TABLE_LEAKDETECTOR,
//    PICK_LEAKDETECTOR,
//    FIND_LEAK,
//    WALK_TO_TABLE_LEAKREPAIRTOOL,
//    PICK_LEAKREPAIRTOOL,
//    WALK_TO_LEAKPOSITION,
//    REPAIR_LEAK,
//    WALK_TO_FINISHBOX
//};

//int main(int argc, char** argv)
//{
//    ros::init(argc, argv, "val_task3_node");
//    ros::NodeHandle nh;

//    sm state = PREPARE_START;
//    switch (state)
//    {
//    case PREPARE_START:
//    {
//        /*
//        We come here from Task 2, starting from the finish box of the past task.
//        R5 will start here and finish at the beginning of the stairs performing the following tasks
//        1.- Detect the stair location (Primary)
//        2.- Go to the pre-defined stair location
//        3.- Walk to the stair location
//        bool flag_stairs_detected = false;
//        bool destination_reached = false;
//        float stairs_params[];
//        float robot_params[];
//        while(destination_reached == false){
//            while(flag_stairs_detected == false){
//                stair_params = detect_stairs(); //returns position and orientation of the stairs
//                if(stair_params){
//                    //we detected the stairs
//                    flag_stairs_detected = true;
//                    robot_params = walk_to_stairs(); //update the position of the robot as we walk
//                    if(stairs_params.position == robot_params.position){
//                        //we reached our destination
//                        destination_reached = true;
//                    }
//                }
//            }
//        }
//        */
//        state = CLIMB_STAIRS;
//    }
//    case CLIMB_STAIRS:
//    {

//        /*
//      // Note: task is to climb the stairs.
//      // (i)  check the minimum number of the stairs -  use PCL and opencv
//      //      if no stairs found the task is completed.
//      // (ii) get the stairs parameters like width and height  - opencv - only once
//      // (iii)now compare the two height u got from the above two steps - only once
//      // (iv) generate_trajectory based on the stairs parameters
//      // (v) execute the trajectory
//      // (vi) repeat step (i)

//       bool flag_stairs_detect = true;
//       bool state_status = false;
//       bool once = true;
//       float  stairs_param[4];
//      //  0 - minimum no of stairs
//      //  1 - height h1
//      //  2 - height h2
//      //  3 - width
//      while(!state_status)
//      {
//        while(flag_stairs_detect)
//        {
//          stairs_param[0,1] = detect_stairs();   // idententifies the stairs and computes  minimum number of stairs and height (h1) of the stairs
//          if ((int)(stairs_param[0])==0)
//          {
//            flag_stairs_detect = false;
//            break;
//          }
//          if(once){

//            stairs_param[2,3] = stairs_param_fn(); // computes the width and height (h2) of the stairs_param
//            float mean_height = (stairs_param[1]+stairs_param[2])/2; // take mean or something else to get the height of the stairs obtained from two different fn.
//            once = false;
//          }
//          float climb_stairs[] = generate_trajectory(n, width, mean_height);
//          state_finished = execute_trajectory(climb_stairs); // returns true if trajectory is executed successfully
//        }
//      }*/
//        state = OPEN_DOOR;
//    }
//    case OPEN_DOOR:
//    {
//        //Write pseudo code here
//        /*Task is to open the habitat door
//            1.	Given that the robot is in front of the door
//            2.	Get the location of the valve using functions from Perception package
//            3.	After getting the location of the valve, plan the arm trajectory for grasping the valve
//            4.	Check if the valve is grasped
//            5.	Turn the valve until the door opens
    
//        while status_of_the_state = 0
//        {
//            while robot_in_front_of_the door
//            {
//	            location_of_the_valve = detect_object_function();
//	            if location_found
//	            {
//		            grasp_the_valve = grasp_object_function(location_coordinates)
//		            if grasp_done
//			            turn_the_valve_until_door_opens()
//	                    status_of_the_state = 1;
//	            }
//            }
//        }*/

//        state = WALK_THROUGH_DOOR;
//    }
//    case WALK_THROUGH_DOOR:
//    {
//        // Use IHMC robotpose topic to get your current location in world frame and robotstatus topic to see your current motion status
//        // Use perception / predefined values /user defined values to calculate goal position.
//        // Use 2D foot step planner to generate footsteps.
//        // Walk to the goal position : Use ihmc foottrajectoryros messages or whole body ros message

//        //TODO: Write CODE here
        
//        /* Task is to push open the door
//            1.	Push the door by holding the rotary valve
//                (Force to be applied on the door – unknown)
//            2.	Check if door is open using Perception package
//            3.	If door is open, go to next state
        
//        while status_of_the_state = 0
//        {
//            while valve_turned
//            {
//		        push_the_door()
//		        if door_open
//			        status_of_the_state = 1;
//            }
//        }*/
        
//        state = WALK_TO_TABLE_LEAKDETECTOR;
//    }
//    case WALK_TO_TABLE_LEAKDETECTOR:
//    {
//        // Use IHMC robotpose topic to get your current location in world frame and robotstatus topic to see your current motion status
//        // Use perception / predefined values /user defined values to calculate goal position.
//        // Use 2D foot step planner to generate footsteps.
//        // Walk to the goal position : Use ihmc foottrajectoryros messages or whole body ros message

//        /* Summary of this state:
//        (i) Find the table with the tool on it
//        (ii) Walk to the table
//        (iii) Once at the table, go to the next state
        
//        Pseudo-code:
//        tool_location = null;
//        while (tool_location == null) {
//            tool_location = detect_tool();
//        }
        
//        walk_to(tool_location);
//        */
//        state = PICK_LEAKDETECTOR;
//    }
//        /* State Action : To pick the leak detector tool from the table */
//    case PICK_LEAKDETECTOR:
//    {

//        /* Summary of this state:
//        (i) Ensure that the Robot is in front of the table.
//        (ii)Get the location of the leak detector tool on the table using Perception packages which implements the object detection code. : Perception Task
//        (iii)Once the tool location is obtained, plan the arm trajectory for grasping and picking up the tool. : Motion Planning and Grasping
//        (iv)Once tool is grasped,go to next state

//            Pseudo-code:
//            bool state_status = 0;
//            bool grasp_done = 0;
//            bool location_found = 0;
//            while !(state_status)
//            {
//                while robot_is_in_front_of_table
//                {
//                    //find the location of the tool on the table
//                    location = detect_object_fn();
//                    //location contains if location is found as the first argument and location coordinates as the second argument
//                    location_found = location[0];
//                    if location_found
//                    {
//                        grasp_done = grasp_object_fn(location[1])
//                        if grasp_done
//                            state_status = 1;
//                    }
//                }
//            }
//        */
//        state = FIND_LEAK;
//    }
//        /* State Action : To find the leak location in the prescribed wall */
//    case FIND_LEAK:
//    {

//        /* Summary of this state:
//          (i)Ensure that the Robot has the tool in its hand
//          (ii)The robot is now given the predefined wall location and it has to plan footsteps to the desired location in front of the wall : Footstep Planning
//          (iii)Now the robot has to plan a trajectory to move the tool over the entire area of the wall : Motion Planning Coverage Problem
//          (iv)Leak detection message status need to be monitored to identify the leak coordinates and publish the leak coordinates with respect to the world: TBD
//          (v)Once leak is detected , go to the next state.

//            Pseudo-code:
//            bool state_status = 0;
//            bool walk_done = 0;
//            bool location_found = 0;
//            bool leak_found = 0;
//            while !(state_status)
//            {
//                while tool_is_in_robots_hand
//                {
//                    //find the location of the predefined wall
//                    location = get_location();
//                    //location contains if location is found as the first argument and location coordinates as the second argument
//                    location_found = location[0];
//                    if location_found
//                    {
//                        walk_done = walk_robot_fn(location[1]) // move robot to the specified location
//                        if walk_done
//                        {
//                            // moving arm around the entire wall which is done in find leakage function
//                           leak_found = detect_leak_fn(); // this function will move the arm and detects the leak and publishes the location to a ROS topic
//                           if leak_found
//                            state_status = 1;
//                        }
//                    }
//                }
//            }
//        */
//        state = WALK_TO_TABLE_LEAKREPAIRTOOL;
//    }
//    case WALK_TO_TABLE_LEAKREPAIRTOOL:
//    {
//        // Use IHMC robotpose topic to get your current location in world frame and robotstatus topic to see your current motion status
//        // Use perception / predefined values /user defined values to calculate goal position.
//        // Use 2D foot step planner to generate footsteps.
//        // Walk to the goal position : Use ihmc foottrajectoryros messages or whole body ros message

//      // Summary:
//      // - If the location of the repair tool/table is known, just walk towards it.
//      // - If location is not known, turn around and look for a table nearby: feautre detection using images/point cloud
//      // - If you detect the old table which had the detector tool, ignore it and continue looking
//      // - Save location of table w.r.t world
//      // - Walk towards the table
//      // - Stop at the table and localize the repair tool
//      // - If tool is not reachable (i.e it is above a predefined distance measure), walk around to minimize the distance between hand-tool
//      // - optiionally check (somehow) if you have the detector in the hand before you continue to the next state


//      // located_table=false
//      // if table_location_predefined:
//      //   located_table=true

//      // if facing_wall:
//      //   turn_around()

//      // while not located_table:
//      //   call_locate_table()
//      //   if table!=initial_table_with_detector_tool:
//      //       located_table=true

//      //   returned_points_to_table=detct_table_hrizontal_edges(table)
//      //   walk_to_table(returned_points_to_table)

//      //   while detect_distance_to_tool()>threshold:
//      //     walk_around_table_to_min_tool_hand_dist(detect_distance_to_tool())

      
//      //// Checking if detector is still in the hand using perception
//      // raise_arm_with_detector_to_view()
//      // verify_with_repair_tool=false

//      // if tool_in_hand:
//      //   verify_with_repair_tool = true

//      // lower_arm_with_detector()
//        state = PICK_LEAKREPAIRTOOL;
//    }
//    case PICK_LEAKREPAIRTOOL:
//    {
//        /* State Summary:
//         * 1. Current robot location is facing towards table which contains the leak repair tool
//         * 2. Identify the leak repair tool on the table and get coordinates using perception package to detect objects (should include a routine to identity the leak repair tool)
//         * 3. Setup the hand to pick the leak repair tool into grasp pose
//         * 4. Move arm to the tool coordinates
//         * 5. Move hand to close teh grasp around the tool
//         * 6. Pick the tool by moving the hand up from the table
//         * 7. Verify the tool is picked, and not left on table : Perception task
//         * 8. Respawn if the tool was not picked
//         * 9. Goto next state if the tool was picked correclty
//         *
//         *    This state might include putting back the leak detect tool. Assumption for now is that we keep both tools in our hands so that we can
//         *    verify later if the leak has been fixed or not.
//         *
//         *
//         *    Pseudo-code  (keeping it on similar lines as pick leak detector for consistency
//         *
//         *    bool location_found = 0;
//         *    bool state_status = 0;
//         *    bool grasp_done = 0;
//         *    bool hand_pose = 0;
//         *    bool arm_pos = 0;
//         *    while !(state_status)
//         *    {
//         *        if ~robot_is_in_front_of_table
//         *            state = PICK_LEAKREPAIRTOOL;
//         *            respawn;
//         *        else
//         *            while robot_is_in_front_of_table
//         *            {
//         *                //Find the location of leak repair tool on the table
//         *                loc_tool = detect_leak_rep_tool();
//         *
//         *                //location contains if location is found as the first argument and location coordinates as the second argument
//         *                location_found = loc_tool[0];
//         *                if location_found
//         *                {
//         *                    //Set the hand in  grasping position
//         *                    hand_pose = set_hand_pose();
//         *
//         *                    //If the pose is set, move arm to tool location
//         *                    if hand_pose
//         *                        arm_pos = arm_trajecory();
//         *
//         *                        //If the arm is in coorect position to pick the repair tool, go for grasping
//         *                        if arm_pos
//         *                        grasp_done = grasp_object_fn(location[1]);
//         *
//         *                       //Verify that the repair tool has been picked
//         *                       if grasp_done
//         *                           state_status = verify_grasping();  //Perception task
//         *                   }
//         *
//         *                   if ~state_status
//         *                   {
//         *                       state = PICK_LEAKREPAIRTOOL;
//         *                       respawn;
//         *                   }
//         *              }
//         *     }
//         */
//        state = WALK_TO_LEAKPOSITION;
//    }
//    case WALK_TO_LEAKPOSITION:
//    {
//        // Use IHMC robotpose topic to get your current location in world frame and robotstatus topic to see your current motion status
//        // Use perception / predefined values /user defined values to calculate goal position.
//        // Use 2D foot step planner to generate footsteps.
//        // Walk to the goal position : Use ihmc foottrajectoryros messages or whole body ros message

//        //TODO: Write CODE here
//        state = REPAIR_LEAK;
//    }
//    case REPAIR_LEAK:
//    {
//        // Compute trajectory from current tool position to leak position and orientation : decide between trajopt / moveit
//        // Orient the repair tool with respect to the leak position and orientation : execute the trajectories. you could use ihmc whole body messages or individual messages
//        // Press the button on leak repair tool to fix leak: you could use ihmc armtrajectory message or handtrajectory messages
//        // Release the button on leak repair tool: you could use ihmc armtrajectory message or handtrajectory messages
//        // You could use visual feedback /user control to detect if the leak is fixed or add new states/ jump back to old states to verify this

//        //TODO: Write CODE here
//        state = WALK_TO_FINISHBOX;
//    }
//    case WALK_TO_FINISHBOX:
//    {
//        // Use IHMC robotpose topic to get your current location in world frame and robotstatus topic to see your current motion status
//        // Use perception / predefined values /user defined values to calculate goal position.
//        // Use 2D foot step planner to generate footsteps.
//        // Walk to the goal position : Use ihmc foottrajectoryros messages or whole body ros message

//        //TODO: Write CODE here
//        break;
//    }

//    default:
//        break;
//    }
//    ros::spinOnce();
//    return 0;
//}

//*/*/
