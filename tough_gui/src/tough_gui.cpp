#include <iostream>
#include "tough_gui/tough_gui.h"
#include "ui_tough_gui.h"
#include "rviz/view_manager.h"
#include "rviz/tool_manager.h"
#include "rviz/properties/property_tree_model.h"
#include "tough_gui/configurationreader.h"
#include "ros/package.h"
#include <boost/lexical_cast.hpp>
#include <map>

/**
 * This class creates the GUI using rviz APIs.
 */

ToughGUI::ToughGUI(QWidget* parent) : QMainWindow(parent), ui(new Ui::ToughGUI), it_(nh_)
{
  /**
   * Set up the QT related UI components.
   */
  ui->setupUi(this);

  // set all the controller pointers to null
  chestController_ = nullptr;
  pelvisHeightController_ = nullptr;
  armJointController_ = nullptr;
  walkingController_ = nullptr;
  headController_ = nullptr;
  gripperController_ = nullptr;

  // clickedpoint is used for moving arm in taskspace
  clickedPoint_ = nullptr;

  rd_ = RobotDescription::getRobotDescription(nh_);
  currentState_ = RobotStateInformer::getRobotStateInformer(nh_);

  // initialize everything
  initVariables();
  initJointLimits();
  initToughControllers();
  initDisplayWidgets();
  initTools();
  initActionsConnections();
  initDefaultValues();
}

ToughGUI::~ToughGUI()
{
  delete ui;
  delete mapManager_;
  delete mapRenderPanel_;
  delete manager_;
  delete renderPanel_;
  delete status_label_;
  delete chestController_;
  delete pelvisHeightController_;
  delete armJointController_;
  delete headController_;
  delete gripperController_;
  delete clickedPoint_;
}

void ToughGUI::initVariables()
{
  /**
   *Initialize default values of all the variables. Push these definitions to xml/config file in future
   */

  // Read the configuration file
  std::string configFile = ros::package::getPath("tough_gui") + "/config/config.ini";
  ROS_INFO("config file : %s", configFile.c_str());
  ConfigurationReader configfile(configFile.c_str());

  std::string robot_name = rd_->getRobotName();

  // Assign topic names to corresponding variables
  fixedFrame_ = QString::fromStdString(configfile.currentTopics["fixedFrame"]);
  mapTopic_ = QString::fromStdString(configfile.currentTopics["mapTopic"]);
  imageTopic_ = QString::fromStdString(configfile.currentTopics["imageTopic"]);
  pointCloudTopic_ = QString::fromStdString(robot_name + "/" + configfile.currentTopics["pointCloudTopic"]);
  octomapTopic_ = QString::fromStdString(robot_name + "/" + configfile.currentTopics["octomapTopic"]);
  targetFrame_ = QString::fromStdString(configfile.currentTopics["targetFrame"]);
  robotType_ = QString::fromStdString(configfile.currentTopics["robotType"]);
  goalTopic_ = QString::fromStdString(configfile.currentTopics["goalTopic"]);
  footstepTopic_ = QString::fromStdString(configfile.currentTopics["footstepTopic"]);
  jointStatesTopic_ = QString::fromStdString(configfile.currentTopics["jointStatesTopic"]);
  approveStepsTopic_ = QString::fromStdString(configfile.currentTopics["approveStepsTopic"]);

  resetPointcloudTopic_ = QString::fromStdString(robot_name + "/" + configfile.currentTopics["resetPoincloudTopic"]);
  pausePointcloudTopic_ = QString::fromStdString(robot_name + "/" + configfile.currentTopics["pausePoincloudTopic"]);

  try
  {
    flipImage_ = boost::lexical_cast<bool>(configfile.currentTopics["flip"]);
  }
  catch (const boost::bad_lexical_cast& e)
  {
    std::cerr << "flip parameter is incorrectly set in config.ini. setting flip to false" << std::endl;
  }

  // subscribers
  liveVideoSub = it_.subscribe(imageTopic_.toStdString(), 1, &ToughGUI::liveVideoCallback, this,
                               image_transport::TransportHints("raw"));
  jointStatesUpdater_ = nh_.createTimer(ros::Duration(0.25), &ToughGUI::jointStateCallBack, this);
  //    @todo: add timer based callback here to call jointStateCallBack method
  clickedPointSub_ = nh_.subscribe("clicked_point", 1, &ToughGUI::getClickedPoint, this);

  approveStepsPub_ = nh_.advertise<std_msgs::Empty>(approveStepsTopic_.toStdString(), 1, true);
  centerOfMassPub_ = nh_.advertise<visualization_msgs::Marker>(COM_TOPIC, 1, false);

  // initialize a onetime map to lookup for joint values
  rd_->getLeftArmJointNames(leftArmJointNames_);
  rd_->getRightArmJointNames(rightArmJointNames_);
  rd_->getChestJointNames(chestJointNames_);

  std::vector<std::string> joints;
  joints.insert(joints.end(), leftArmJointNames_.begin(), leftArmJointNames_.end());
  joints.insert(joints.end(), rightArmJointNames_.begin(), rightArmJointNames_.end());
  joints.insert(joints.end(), chestJointNames_.begin(), chestJointNames_.end());

  // the values of labels and sliders is hardcoded to match with jointnames at the same index
  std::vector<QLabel*> jointLabels = { ui->lblLeftShoulderPitch, ui->lblLeftShoulderRoll,   ui->lblLeftShoulderYaw,
                                       ui->lblLeftElbowPitch,    ui->lblLeftForearmYaw,     ui->lblLeftWristRoll,
                                       ui->lblLeftWristPitch,    ui->lblRightShoulderPitch, ui->lblRightShoulderRoll,
                                       ui->lblRightShoulderYaw,  ui->lblRightElbowPitch,    ui->lblRightForearmYaw,
                                       ui->lblRightWristRoll,    ui->lblRightWristPitch,    ui->lblChestYaw,
                                       ui->lblChestPitch,        ui->lblChestRoll };

  std::vector<QSlider*> jointSliders = { ui->sliderShoulderPitch, ui->sliderShoulderRoll,  ui->sliderShoulderYaw,
                                         ui->sliderElbow,         ui->sliderWristYaw,      ui->sliderWristRoll,
                                         ui->sliderWristPitch,    ui->sliderShoulderPitch, ui->sliderShoulderRoll,
                                         ui->sliderShoulderYaw,   ui->sliderElbow,         ui->sliderWristYaw,
                                         ui->sliderWristRoll,     ui->sliderWristPitch,      ui->sliderChestYaw,
                                         ui->sliderChestPitch,    ui->sliderChestRoll };

  assert(joints.size() == jointLabels.size() && "joints and jointlabels must be of same size");

  for (size_t i = 0; i < joints.size(); ++i)
  {
    jointLabelMap_[joints[i]] = jointLabels[i];
    jointSliderMap_[joints[i]] = jointSliders[i];
  }

  // moveArmCommand is a flag used to check if user intends to move arm or just publish a point
  moveArmCommand_ = false;

  mode_map = { { "BASIC", GripperControlInterface::GRIPPER_MODES::BASIC },
               { "PINCH", GripperControlInterface::GRIPPER_MODES::PINCH },
               { "WIDE", GripperControlInterface::GRIPPER_MODES::WIDE },
               { "SCISSOR", GripperControlInterface::GRIPPER_MODES::SCISSOR } };

  prev_mode_map = { { "BASIC", 0 }, { "PINCH", 1 }, { "WIDE", 2 }, { "SCISSOR", 3 } };

  resetPointcloudPub_ = nh_.advertise<std_msgs::Empty>(resetPointcloudTopic_.toStdString(), 1, true);
  pausePointcloud_ = nh_.advertise<std_msgs::Bool>(pausePointcloudTopic_.toStdString(), 1, true);
  pausePointcloudMsg_.data = false;

  // CoM Marker
  comMarker.header.frame_id = rd_->getWorldFrame();
  comMarker.id = 0;
  comMarker.type = visualization_msgs::Marker::SPHERE;
  comMarker.action = visualization_msgs::Marker::ADD;
  comMarker.pose.orientation.w = 1.0;
  comMarker.scale.x = 0.05;
  comMarker.scale.y = 0.05;
  comMarker.scale.z = 0.05;
  comMarker.color.a = 1.0;  // Don't forget to set the alpha!
  comMarker.color.r = 0.0;
  comMarker.color.g = 1.0;
  comMarker.color.b = 0.0;

  std::string prefix = TOUGH_COMMON_NAMES::TOPIC_PREFIX + rd_->getRobotName() + TOUGH_COMMON_NAMES::OUTPUT_TOPIC_PREFIX;
  L_FOOT_FORCE_TOPIC = prefix + TOUGH_COMMON_NAMES::LEFT_FOOT_FORCE_SENSOR_TOPIC;
  R_FOOT_FORCE_TOPIC = prefix + TOUGH_COMMON_NAMES::RIGHT_FOOT_FORCE_SENSOR_TOPIC;
  L_WRIST_FORCE_TOPIC = prefix + TOUGH_COMMON_NAMES::LEFT_WRIST_FORCE_SENSOR_TOPIC;
  R_WRIST_FORCE_TOPIC = prefix + TOUGH_COMMON_NAMES::RIGHT_WRIST_FORCE_SENSOR_TOPIC;
}

void ToughGUI::initActionsConnections()
{
  /**
   * Set up the status Bar and display messages emitted from each of the tools.
   * All the tools in rviz API has updateStatus function to emit messages to the status bar.
   */
  status_label_ = new QLabel("");
  statusBar()->addPermanentWidget(status_label_, 1);
  connect(manager_, SIGNAL(statusUpdate(const QString&)), status_label_, SLOT(setText(const QString&)));

  /**
   * Setup Signals and slots for different buttons/sliders in UI.
   */
  // Tool and display selection
  connect(ui->btnGroupRvizTools, SIGNAL(buttonClicked(int)), this, SLOT(setCurrentTool(int)));
  connect(ui->controlTabs, SIGNAL(currentChanged(int)), this, SLOT(updateJointStateSub(int)));
  connect(ui->tab_display, SIGNAL(currentChanged(int)), this, SLOT(updateDisplay(int)));

  connect(ui->chkBoxPointcloud, static_cast<void (QCheckBox::*)(int)>(&QCheckBox::stateChanged), this,
          [this](int state) { toggleDisplay(cloudDisplay_, state); });
  connect(ui->chkBoxOctomap, static_cast<void (QCheckBox::*)(int)>(&QCheckBox::stateChanged), this,
          [this](int state) { toggleDisplay(octomapDisplay_, state); });
  connect(ui->chkBoxCoM, static_cast<void (QCheckBox::*)(int)>(&QCheckBox::stateChanged), this,
          [this](int state) { toggleDisplay(capturabilityMarkerDisplay_, state); });
  connect(ui->chkBoxFootForces, static_cast<void (QCheckBox::*)(int)>(&QCheckBox::stateChanged), this, [=](int state) {
    toggleDisplay(leftFootForceDisplay_, state);
    toggleDisplay(rightFootForceDisplay_, state);
  });
  connect(ui->chkBoxFootsteps, static_cast<void (QCheckBox::*)(int)>(&QCheckBox::stateChanged), this,
          [this](int state) {
            toggleDisplay(footstepMarkersDisplay_, state);
            toggleDisplay(footstepMarkersMainDisplay_, state);
          });
  connect(ui->chkBoxWristForces, static_cast<void (QCheckBox::*)(int)>(&QCheckBox::stateChanged), this,
          [this](int state) {
            toggleDisplay(leftWristForceDisplay_, state);
            toggleDisplay(rightWristForceDisplay_, state);
          });

  // pointCloud
  connect(ui->btnResetPointcloud, SIGNAL(clicked()), this, SLOT(resetPointcloud()));
  connect(ui->btnPausePointcloud, SIGNAL(clicked()), this, SLOT(pausePointcloud()));

  // nudge
  connect(ui->btnMoveToPoint, SIGNAL(clicked()), this, SLOT(moveToPoint()));
  connect(ui->btnGroupNudge, SIGNAL(buttonClicked(int)), this, SLOT(nudgeArm(int)));

  // arm control
  connect(ui->btnGroupArm, SIGNAL(buttonClicked(int)), this, SLOT(updateArmSide(int)));
  connect(ui->sliderShoulderRoll, SIGNAL(sliderReleased()), this, SLOT(moveArmJoints()));
  connect(ui->sliderShoulderPitch, SIGNAL(sliderReleased()), this, SLOT(moveArmJoints()));
  connect(ui->sliderShoulderYaw, SIGNAL(sliderReleased()), this, SLOT(moveArmJoints()));
  connect(ui->sliderWristRoll, SIGNAL(sliderReleased()), this, SLOT(moveArmJoints()));
  connect(ui->sliderWristPitch, SIGNAL(sliderReleased()), this, SLOT(moveArmJoints()));
  connect(ui->sliderWristYaw, SIGNAL(sliderReleased()), this, SLOT(moveArmJoints()));
  connect(ui->sliderElbow, SIGNAL(sliderReleased()), this, SLOT(moveArmJoints()));
  connect(ui->btnResetArm, SIGNAL(clicked()), this, SLOT(resetArm()));

  // gripper control
  connect(ui->btnGroupGripper, SIGNAL(buttonClicked(int)), this, SLOT(updateGripperSide(int)));
  connect(ui->cmbBoxGripMode, SIGNAL(currentIndexChanged(int)), this, SLOT(setMode()));
  connect(ui->btnResetGrippers, SIGNAL(clicked()), this, SLOT(resetGrippers()));
  connect(ui->btnCloseBothHands, SIGNAL(clicked()), this, SLOT(closeBothGrippers()));
  connect(ui->btnCloseHand, SIGNAL(clicked()), this, SLOT(closeGrippers()));
  connect(ui->btnOpenHand, SIGNAL(clicked()), this, SLOT(openGrippers()));
  connect(ui->btnCloseFingers, SIGNAL(clicked()), this, SLOT(closeFingers()));
  connect(ui->btnOpenFingers, SIGNAL(clicked()), this, SLOT(openFingers()));
  connect(ui->btnCloseThumb, SIGNAL(clicked()), this, SLOT(closeThumb()));
  connect(ui->btnOpenThumb, SIGNAL(clicked()), this, SLOT(openThumb()));

  // chest control
  connect(ui->sliderChestRoll, SIGNAL(sliderReleased()), this, SLOT(moveChestJoints()));
  connect(ui->sliderChestPitch, SIGNAL(sliderReleased()), this, SLOT(moveChestJoints()));
  connect(ui->sliderChestYaw, SIGNAL(sliderReleased()), this, SLOT(moveChestJoints()));
  connect(ui->btnChestReset, SIGNAL(clicked()), this, SLOT(resetChestOrientation()));

  // neck control
  connect(ui->sliderUpperNeckPitch, SIGNAL(sliderReleased()), this, SLOT(moveHeadJoints()));
  connect(ui->sliderLowerNeckPitch, SIGNAL(sliderReleased()), this, SLOT(moveHeadJoints()));
  connect(ui->sliderNeckYaw, SIGNAL(sliderReleased()), this, SLOT(moveHeadJoints()));

  // walk
  connect(ui->btnWalk, SIGNAL(clicked()), this, SLOT(walkSteps()));
  connect(ui->sliderPelvisHeight, SIGNAL(sliderReleased()), this, SLOT(changePelvisHeight()));
  connect(ui->btnApproveSteps, SIGNAL(clicked()), this, SLOT(approveSteps()));
  connect(ui->btnAbortWalk, SIGNAL(clicked()), this, SLOT(abortSteps()));
  connect(ui->btnAlignFeet, SIGNAL(clicked()), this, SLOT(alignFeet()));

  // reset robot
  connect(ui->btnResetRobot, SIGNAL(clicked()), this, SLOT(resetRobot()));
}

void ToughGUI::initDisplayWidgets()
{
  // Setup the UI elements for displaying 2D map
  /**
   * VisualizationManager is used to control different displays that are shown in a widget.
   * Renderpanel is a widget that provides a 3D space in the visualizationmanager.
   * startUpdate() function starts the timers and subscribes to defined topics at 30Hz.
   */
  mapRenderPanel_ = new rviz::RenderPanel();
  ui->map_layout->addWidget(mapRenderPanel_);
  mapManager_ = new rviz::VisualizationManager(mapRenderPanel_);
  mapRenderPanel_->initialize(mapManager_->getSceneManager(), mapManager_);
  mapManager_->setFixedFrame(fixedFrame_);
  mapManager_->initialize();
  mapManager_->startUpdate();

  // Create and assign FixedOrientationOrthoViewController to the existing viewmanager of the visualization manager
  /**
   * VisualisationManager has a manager for most of its children. ViewManager is responsible for setting the
   * viewController.
   * Default View Controller is rviz/Orbit, for map we are changing it to rviz/TopDownOrtho
   * To set properties of most of the rviz objects, use subProp and setValue functions as shown below
   * New displays can be created and added to the visualization manager using createDisplay function as used below
   *
   * @todo Create an xml/config file to define objects to be displayed in GUI alongwith their parameters
   */

  mapViewManager_ = mapManager_->getViewManager();
  mapViewManager_->setCurrentViewControllerType("rviz/TopDownOrtho");

  mapViewController_ = mapViewManager_->getCurrent();

  // Set parameters of the view controller to show map correctly
  mapViewController_->subProp("X")->setValue(4.52);
  mapViewController_->subProp("Y")->setValue(0);
  mapViewController_->subProp("Angle")->setValue(0);
  mapViewController_->subProp("Scale")->setValue(100);

  // Create a map display
  mapDisplay_ = mapManager_->createDisplay("rviz/Map", "2D Map view", true);
  ROS_ASSERT(mapDisplay_ != NULL);

  mapDisplay_->subProp("Topic")->setValue(mapTopic_);

  QString robotModelTopic = QString::fromStdString(rd_->getURDFParameter());
  // add Robot model
  mapManager_->createDisplay("rviz/RobotModel", robotType_, true)
      ->subProp("Robot Description")
      ->setValue(robotModelTopic);

  mapManager_->createDisplay("rviz/Grid", "Grid", true);
  mapManager_->createDisplay("rviz/MarkerArray", "Footstep markers", true)
      ->subProp("Marker Topic")
      ->setValue(footstepTopic_);

  // Initialize GUI elements for main panel
  renderPanel_ = new rviz::RenderPanel();
  ui->display3d_layout->addWidget(renderPanel_);

  manager_ = new rviz::VisualizationManager(renderPanel_);
  renderPanel_->initialize(manager_->getSceneManager(), manager_);

  // set the fixed frame before initializing Visualization Manager. pointcloud2 will not work with this
  manager_->setFixedFrame(fixedFrame_);
  manager_->initialize();
  manager_->startUpdate();

  // Create a main display to show pointcloud and octomap
  manager_->createDisplay("rviz/Grid", "Grid", true);
  manager_->createDisplay("rviz/RobotModel", robotType_, true)->subProp("Robot Description")->setValue(robotModelTopic);

  cloudDisplay_ = manager_->createDisplay("rviz/PointCloud2", "3D Pointcloud view", false);
  assert(cloudDisplay_ != NULL && "Could not create a display");

  cloudDisplay_->subProp("Topic")->setValue(pointCloudTopic_);
  cloudDisplay_->subProp("Selectable")->setValue("true");
  cloudDisplay_->subProp("Style")->setValue("Boxes");
  cloudDisplay_->subProp("Alpha")->setValue(0.5);
  cloudDisplay_->subProp("Color Transformer")->setValue("AxisColor");

  octomapDisplay_ = manager_->createDisplay("rviz/MarkerArray", "Octomap view", false);
  ROS_ASSERT(octomapDisplay_ != NULL);

  octomapDisplay_->subProp("Marker Topic")->setValue(octomapTopic_);

  // footsteps
  createFootstepDisplay(manager_, mapManager_);

  // Assign Target Frame to the existing viewmanager of the visualization manager
  // rviz::ViewManager* viewManager_ = manager_->getViewManager();
  // rviz::ViewController* viewController_ = viewManager_->getCurrent();
  // viewController_->subProp("Target Frame")->setValue(targetFrame_);

  // add support polygon - This needs changes to Java code

  // add CoM projection and 3D
  capturabilityMarkerDisplay_ = manager_->createDisplay("rviz/Marker", "CoM", true);
  capturabilityMarkerDisplay_->subProp("Marker Topic")->setValue(QString::fromStdString(COM_TOPIC));
  capturabilityMarkerDisplay_->subProp("Queue Size")->setValue("1");

  // add Forces
  leftFootForceDisplay_ = createWrenchDisplay(manager_, L_FOOT_FORCE_TOPIC);
  rightFootForceDisplay_ = createWrenchDisplay(manager_, R_FOOT_FORCE_TOPIC);
  leftWristForceDisplay_ = createWrenchDisplay(manager_, L_WRIST_FORCE_TOPIC);
  rightWristForceDisplay_ = createWrenchDisplay(manager_, R_WRIST_FORCE_TOPIC);

  QString imagePath = QString::fromStdString(ros::package::getPath("tough_gui") + "/resources/coordinates.png");
  QImage qImage(imagePath);
  ui->lblAxes->setPixmap(QPixmap::fromImage(qImage));
  moveitDisplay_ = nullptr;
}
void ToughGUI::createFootstepDisplay(rviz::VisualizationManager* manager, rviz::VisualizationManager* map_manager)
{
  footstepMarkersDisplay_ = map_manager->createDisplay("rviz/MarkerArray", "Footsteps", true);
  footstepMarkersDisplay_->subProp("Marker Topic")->setValue(footstepTopic_);
  footstepMarkersDisplay_->subProp("Queue Size")->setValue("100");

  footstepMarkersMainDisplay_ = manager->createDisplay("rviz/MarkerArray", "Footsteps", true);
  footstepMarkersMainDisplay_->subProp("Marker Topic")->setValue(footstepTopic_);
  footstepMarkersMainDisplay_->subProp("Queue Size")->setValue("10");
}

rviz::Display* ToughGUI::createWrenchDisplay(rviz::VisualizationManager* manager, const std::string& topic)
{
  rviz::Display* display;
  display = manager->createDisplay("rviz/WrenchStamped", QString::number(qrand()), true);
  assert(display != NULL && "Could not create a display");
  display->subProp("Topic")->setValue(QString::fromStdString(topic));
  display->subProp("Force Arrow Scale")->setValue("0.001");
  display->subProp("Torque Arrow Scale")->setValue("0");
  display->subProp("Arrow Width")->setValue("0.5");
  return display;
}

// This doesn';t work as Moveit loads a panel which ends up with seg fault :(
// I'll fix it some day. Until then we will try a different solution
void ToughGUI::createMoveitDisplay()
{
  moveitDisplay_ = manager_->createDisplay("moveit_rviz_plugin/MotionPlanning", "MoveIt", false);
  moveitDisplay_->subProp("Planning Request")->subProp("Planning Group")->setValue("leftMiddleFingerGroup");
  moveitDisplay_->subProp("Planning Request")->subProp("Interactive Marker Size")->setValue("0.2");
  moveitDisplay_->setEnabled(true);
}

void ToughGUI::deleteMoveitDisplay()
{
  rviz::Display* tempDisplay = moveitDisplay_;
  moveitDisplay_ = nullptr;
  delete tempDisplay;
}

void ToughGUI::toggleDisplay(rviz::Display* display, int state)
{
  display->setEnabled(state == Qt::Checked);
}

void ToughGUI::initTools()
{
  /**
   * ToolManager is similar to ViewManager. It can be used to add new tools and change the current or default tool.
   * Properties of tools are stored in a PropertyTreeModel. To set/modify any property of a tool use
   * getPropertyContainer function.
   */
  toolManager_ = manager_->getToolManager();
  pointTool_ = toolManager_->addTool("rviz/PublishPoint");
  measureTool_ = toolManager_->addTool("rviz/Measure");
  setGoalTool_ = toolManager_->addTool("rviz/SetGoal");
  setInitialPoseTool_ = toolManager_->addTool("rviz/SetInitialPose");
  interactTool_ = toolManager_->addTool("rviz/Interact");
  mapToolManager_ = mapManager_->getToolManager();
  mapInteractTool_ = mapToolManager_->addTool("rviz/Interact");
  setMapGoalTool_ = mapToolManager_->addTool("rviz/SetGoal");
  setMapInitialPoseTool_ = mapToolManager_->addTool("rviz/SetInitialPose");

  // Find the entry in propertytreemodel and set the value for Topic
  setGoalTool_->getPropertyContainer()->subProp("Topic")->setValue(goalTopic_);
  setMapGoalTool_->getPropertyContainer()->subProp("Topic")->setValue(goalTopic_);

  // set the initial rviz tool to be "interact"
  changeToolButtonStatus(-2);
}

void ToughGUI::initJointLimits()
{
  rd_->getLeftArmJointLimits(leftArmJointLimits_);
  rd_->getRightArmJointLimits(rightArmJointLimits_);
  rd_->getChestJointLimits(chestJointLimits_);

  // reduce the joint limits by 1cm to avoid excceeding limits at higher precision of float. Assuming right and left arm
  // have same number of joints
  for (size_t i = 0; i < leftArmJointLimits_.size(); i++)
  {
    leftArmJointLimits_[i] = { leftArmJointLimits_[i].first + 0.01, leftArmJointLimits_[i].second - 0.01 };
    rightArmJointLimits_[i] = { rightArmJointLimits_[i].first + 0.01, rightArmJointLimits_[i].second - 0.01 };
  }
}

void ToughGUI::initDefaultValues()
{
  // 3D view. select Pointcloud by default
  //  ui->radioBtnPointcloud->setEnabled(true);
  //  ui->radioBtnPointcloud->setChecked(true);
  ui->chkBoxFootsteps->setCheckState(Qt::Checked);
  ui->chkBoxCoM->setCheckState(Qt::Unchecked);
  ui->chkBoxFootForces->setCheckState(Qt::Unchecked);
  ui->chkBoxWristForces->setCheckState(Qt::Unchecked);
  ui->chkBoxPointcloud->setCheckState(Qt::Checked);
  ui->chkBoxOctomap->setCheckState(Qt::Unchecked);

  octomapDisplay_->setEnabled(false);
  cloudDisplay_->setEnabled(true);
  leftFootForceDisplay_->setEnabled(false);
  rightFootForceDisplay_->setEnabled(false);
  leftWristForceDisplay_->setEnabled(false);
  rightWristForceDisplay_->setEnabled(false);
  capturabilityMarkerDisplay_->setEnabled(false);

  // Arms select left arm by default
  ui->radioArmSideLeft->setChecked(true);
  ui->radioNudgeSideLeft->setChecked(true);

  // Gripper select left Gripper by default
  ui->radioGripSideLeft->setChecked(true);

  // Set the default values for the offset
  ui->lineEditNumSteps->setText("2");
  ui->lineEditXOffset->setText("0.3");
  ui->lineEditYOffset->setText("0.0");
  ui->lineEditSwingTime->setText(QString::number(swingTime_));
  ui->lineEditTransferTime->setText(QString::number(transferTime_));
  ui->lineEditSwingHeight->setText(QString::number(swingHeight_));

  // check the right foot start button
  ui->radioRightFoot->setChecked(true);

  // Neck control . Replace these defaults with actual values from robot
  float zeroUpperPitch = fabs(UPPER_NECK_PITCH_MIN / ((UPPER_NECK_PITCH_MAX - UPPER_NECK_PITCH_MIN) / 100.0));
  float zeroLowerPitch = fabs(LOWER_NECK_PITCH_MIN / ((LOWER_NECK_PITCH_MAX - LOWER_NECK_PITCH_MIN) / 100.0));
  float zeroYaw = fabs(NECK_YAW_MIN / ((NECK_YAW_MAX - NECK_YAW_MIN) / 100.0));
  ui->sliderUpperNeckPitch->setValue(zeroUpperPitch);
  ui->sliderLowerNeckPitch->setValue(zeroLowerPitch);
  ui->sliderNeckYaw->setValue(zeroYaw);

  // configure arm sliders
  getArmState();
  getPelvisState();
  getChestState();
}

void ToughGUI::initToughControllers()
{
  // create a chest trajectory controller object
  chestController_ = new ChestControlInterface(nh_);

  // create pelvis height controller object
  pelvisHeightController_ = new PelvisControlInterface(nh_);

  // create walking controller object
  swingTime_ = 1.5f;
  transferTime_ = 1.5f;
  swingHeight_ = 0.18f;
  walkingController_ = new RobotWalker(nh_, transferTime_, swingTime_, 0, swingHeight_);

  // create arm joint controller object
  armJointController_ = new ArmControlInterface(nh_);

  // create a chest trajectory controller object
  headController_ = new HeadControlInterface(nh_);

  // create a gripper controller object
  gripperController_ = new GripperControlInterface(nh_);

  // create a wholebody controller object
  wholeBodyController_ = new WholebodyControlInterface(nh_);

  taskspacePlanner_ = nullptr;
  // new TaskspacePlanner(nh_);
}

void ToughGUI::resetPointcloud()
{
  resetPointcloudPub_.publish(resetPointcloudMsg_);
}

void ToughGUI::pausePointcloud()
{
  // if button is not flat, then pause the pointcloud, otherwise resume
  bool command = !(ui->btnPausePointcloud->isFlat());
  pausePointcloudMsg_.data = command;
  pausePointcloud_.publish(pausePointcloudMsg_);
  ui->btnPausePointcloud->setFlat(command);
}

void ToughGUI::getArmState()
{
  if (jointStateMap_.empty())
  {
    ros::TimerEvent e;
    this->jointStateCallBack(e);
  }
  RobotSide side = ui->radioArmSideLeft->isChecked() ? LEFT : RIGHT;
  mtx_.lock();

  if (side == LEFT)
  {
    for (size_t i = 0; i < leftArmJointNames_.size(); i++)
    {
      double value = jointStateMap_[leftArmJointNames_.at(i)];
      value = (value - leftArmJointLimits_.at(i).first) * 100 /
              (leftArmJointLimits_.at(i).second - leftArmJointLimits_.at(i).first);
      jointSliderMap_[leftArmJointNames_.at(i)]->setValue(value);
    }
  }
  else
  {
    for (size_t i = 0; i < rightArmJointNames_.size(); i++)
    {
      double value = jointStateMap_[rightArmJointNames_.at(i)];
      value = (value - rightArmJointLimits_.at(i).first) * 100 /
              (rightArmJointLimits_.at(i).second - rightArmJointLimits_.at(i).first);
      jointSliderMap_[rightArmJointNames_.at(i)]->setValue(value);
    }
  }

  mtx_.unlock();
}

void ToughGUI::getChestState()
{
  for (size_t i = 0; i < chestJointNames_.size(); i++)
  {
    double value = (jointStateMap_[chestJointNames_[i]] - chestJointLimits_[i].first) * 100.0f /
                   (chestJointLimits_[i].second - chestJointLimits_[i].first);
    jointSliderMap_[chestJointNames_[i]]->setValue(value);
  }

  return;
}

void ToughGUI::getPelvisState()
{
  geometry_msgs::Pose pose;
  pelvisHeightController_->getTaskSpaceState(pose, LEFT);
  ui->txtPelvisHeight->setValue(pose.position.z);
  float pelvisHeight = (pose.position.z - PELVIS_HEIGHT_MIN) * 100 / (PELVIS_HEIGHT_MAX - PELVIS_HEIGHT_MIN);
  ui->sliderPelvisHeight->setValue(pelvisHeight);
  ui->lblRobotPositionX->setText(QString::number(pose.position.x, 'f', 2));
  ui->lblRobotPositionY->setText(QString::number(pose.position.y, 'f', 2));
  ui->lblRobotPositionTheta->setText(QString::number(tf::getYaw(pose.orientation), 'f', 2));
}

void ToughGUI::getNeckState()
{
}

void ToughGUI::getGripperState()
{
}

void ToughGUI::getCoMPosition()
{
  currentState_->getCenterOfMass(centerOfMassPosition_);
  comMarker.header.stamp = ros::Time();
  comMarker.pose.position = centerOfMassPosition_;
  comMarker.pose.position.z = 0;
  centerOfMassPub_.publish(comMarker);
}

void ToughGUI::getClickedPoint(const geometry_msgs::PointStamped::Ptr msg)
{
  if (!moveArmCommand_)
    return;

  if (clickedPoint_ != nullptr)
    delete clickedPoint_;
  ROS_INFO("Creating new point");
  clickedPoint_ = new geometry_msgs::Pose();
  clickedPoint_->orientation.w = 1.0;
  clickedPoint_->position = msg->point;

  RobotSide side = ui->radioNudgeSideLeft->isChecked() ? LEFT : RIGHT;
  ROS_INFO("Moving arm");
  armJointController_->moveArmInTaskSpace(side, *clickedPoint_, 3.0f);
  moveArmCommand_ = false;
}

void ToughGUI::jointStateCallBack(const ros::TimerEvent& e)
{
  static std::vector<std::string> jointNames;
  static std::vector<double> jointValues;

  if (jointNames.empty())
  {
    currentState_->getJointNames(jointNames);
  }
  currentState_->getJointPositions(jointValues);

  for (size_t i = 0; i < jointNames.size(); i++)
  {
    jointStateMap_[jointNames.at(i)] = jointValues.at(i);
    if (jointLabelMap_.count(jointNames.at(i)) > 0)
    {
      QLabel* label = jointLabelMap_[jointNames.at(i)];
      QString text;
      label->setText(text.sprintf("%.2f", jointValues.at(i)));
    }
  }

  // update pelvis height
  if (!ui->txtPelvisHeight->hasFocus())
    getPelvisState();

  if (capturabilityMarkerDisplay_->isEnabled())
    getCoMPosition();

  updateJointStateSub(ui->controlTabs->currentIndex());
}

void ToughGUI::updateJointStateSub(int tabID)
{
  // 0 = nudge
  // 1 = arm
  // 2 = gripper
  // 3 = chest
  // 4 = neck
  // 5 = walk
  //    jointStateSub_ = nh_.subscribe("/joint_states",1, &ValkyrieGUI::jointStateCallBack, this);
  switch (tabID)
  {
    case 0:
      getArmState();
      getGripperState();
      break;
    case 1:
      getArmState();
      break;
    case 2:
      getGripperState();
      break;
    case 3:
      getChestState();
      break;
    case 4:
      getNeckState();
      break;
    case 5:
      getPelvisState();
      break;
    default:
      break;
  }

  //    jointStateSub_.shutdown();
}

void ToughGUI::updateArmSide(int btnID)
{
  //-2 = Left arm
  //-3 = right arm
  getArmState();
}

void ToughGUI::resetChestOrientation()
{
  chestController_->resetPose();
  getChestState();
}

void ToughGUI::resetArm()
{
  RobotSide side = ui->radioArmSideLeft->isChecked() ? LEFT : RIGHT;
  armJointController_->moveToDefaultPose(side);
  getArmState();
}

void ToughGUI::resetRobot()
{
  resetChestOrientation();
  ros::Duration(0.2).sleep();
  armJointController_->moveToDefaultPose(LEFT);
  ros::Duration(0.2).sleep();
  armJointController_->moveToDefaultPose(RIGHT);
  ros::Duration(0.2).sleep();
  pelvisHeightController_->controlPelvisHeight(0.717);
  getArmState();
}

void ToughGUI::moveToPoint()
{
  toolManager_->setCurrentTool(pointTool_);
  moveArmCommand_ = true;
  //    pointTool_->getPropertyContainer()->subProp("Topic")->setValue("clicked_point");
  //    armSide side = ui->radioArmSideLeft->isChecked() ? LEFT : RIGHT;
  //    armJointController_->moveArmInTaskSpace(side, *clickedPoint_);
}

void ToughGUI::nudgeArm(int btnID)
{
  //    down -2
  //    up  -3
  //    back  -4
  //    front -7
  //    left  -5
  //    right -6
  RobotSide side = ui->radioNudgeSideLeft->isChecked() ? LEFT : RIGHT;
  geometry_msgs::Pose end_effector_pose;
  ihmc_msgs::SE3TrajectoryPointRosMessage end_effector_command;

  if (side == RobotSide::RIGHT)
  {
    end_effector_frame_ = rd_->getRightEEFrame();
  }
  else
  {
    end_effector_frame_ = rd_->getLeftEEFrame();
  }

  currentState_->getCurrentPose(end_effector_frame_, end_effector_pose, rd_->getPelvisFrame());
  end_effector_command.position = end_effector_pose.position;
  end_effector_command.orientation = end_effector_pose.orientation;
  end_effector_command.unique_id = 1;

  switch (btnID)
  {
    case -2:  // down
      end_effector_command.position.z -= 0.05;
      break;
    case -3:  // up
      end_effector_command.position.z += 0.05;
      break;
    case -4:  // back
      end_effector_command.position.x -= 0.05;
      break;
    case -7:  // front
      end_effector_command.position.x += 0.05;
      break;
    case -5:  // left
      end_effector_command.position.y += 0.05;
      break;
    case -6:  // right
      end_effector_command.position.y -= 0.05;
      break;
    default:
      return;
  }
  armJointController_->moveArmInTaskSpaceMessage(side, end_effector_command, rd_->getPelvisZUPFrameHash());
}

void ToughGUI::updateDisplay(int tabID)
{
  // 0 = 3d scene
  // 1 = map
  switch (tabID)
  {
    case 0:
      ui->chkBoxCoM->setEnabled(true);
      ui->chkBoxFootForces->setEnabled(true);
      ui->chkBoxFootsteps->setEnabled(true);
      ui->chkBoxOctomap->setEnabled(true);
      ui->chkBoxPointcloud->setEnabled(true);
      ui->chkBoxWristForces->setEnabled(true);
      // change current tool to interact when changing tabs
      setCurrentTool(-2);
      break;
    case 1:
      ui->chkBoxCoM->setEnabled(false);
      ui->chkBoxFootForces->setEnabled(false);
      ui->chkBoxFootsteps->setEnabled(true);
      ui->chkBoxOctomap->setEnabled(false);
      ui->chkBoxPointcloud->setEnabled(false);
      ui->chkBoxWristForces->setEnabled(false);
      // change current tool to interact when changing tabs
      setCurrentTool(-2);
      break;
    default:
      break;
  }
}

void ToughGUI::keyPressEvent(QKeyEvent* event)
{
  switch (event->key())
  {
    case Qt::Key_W:
      closeGrippers();
      ROS_INFO("key W pressed");
      break;
    case Qt::Key_A:
      ROS_INFO("key A pressed");
      break;
    case Qt::Key_D:
      ROS_INFO("key D pressed");
      break;
    case Qt::Key_S:
      openGrippers();
      ROS_INFO("key S pressed");
      break;
    default:
      QWidget::keyPressEvent(event);
      break;
  }
}

void ToughGUI::liveVideoCallback(const sensor_msgs::ImageConstPtr& msg)
{
  /**
   * Adding Image display opens up the image in a new window.
   * As a workaround to show image in the same GUI window, OpenCV is being used to display image on a Qlabel
   */
  cv_bridge::CvImagePtr cv_ptr, cv_ptr_big;
  bool is_rgb = false;
  try
  {
    if (msg->encoding == sensor_msgs::image_encodings::RGB8 || msg->encoding == sensor_msgs::image_encodings::RGB16)
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
      is_rgb = true;
    }

    else
    {
      cv_ptr = cv_bridge::toCvCopy(msg, msg->encoding);
    }
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  //  convert cv image into RGB image and resize it to the size of available layout
  setVideo(ui->liveVideoLabel, cv_ptr, is_rgb);
}

void ToughGUI::setVideo(QLabel* label, cv_bridge::CvImagePtr cv_ptr, bool is_RGB)
{
  cv::Mat RGBImg;
  QLabel* liveVideoLabel = label;

  // To avoid auto expansion of QLabel,keep the video dimensions slightly less than the label dimension
  int height = liveVideoLabel->height() - 1;
  int width = liveVideoLabel->width() - 1;

  if (liveVideoLabel->height() - 1 >= (liveVideoLabel->width() - 1) * IMAGE_HEIGHT / IMAGE_WIDTH)
    height = (liveVideoLabel->width() - 1) * IMAGE_HEIGHT / IMAGE_WIDTH;
  else
    width = (liveVideoLabel->height() - 1) * IMAGE_WIDTH / IMAGE_HEIGHT;
  if (is_RGB)
  {
    RGBImg = cv_ptr->image;
  }
  else
  {
    cv::cvtColor(cv_ptr->image, RGBImg, CV_BGR2RGB);
  }
  cv::resize(RGBImg, RGBImg, cvSize(width, height));
  // flip the image
  if (flipImage_)
    cv::flip(RGBImg, RGBImg, -1);
  //  convert RGB image into QImage and publish that on the label for livevideo
  QImage qImage_ = QImage((uchar*)RGBImg.data, RGBImg.cols, RGBImg.rows, RGBImg.cols * 3, QImage::Format_RGB888);
  liveVideoLabel->setPixmap(QPixmap::fromImage(qImage_));
  liveVideoLabel->show();
}

void ToughGUI::updateGripperSide(int btnID)
{
  ui->cmbBoxGripMode->setCurrentIndex((ui->radioGripSideLeft->isChecked()) ? prev_mode_map[PREVIOUS_MODE_LEFT] :
                                                                             prev_mode_map[PREVIOUS_MODE_RIGHT]);
}

void ToughGUI::setMode()
{
  if (ui->radioGripSideLeft->isChecked())
  {
    if (!(PREVIOUS_MODE_LEFT == ui->cmbBoxGripMode->currentText()))
    {
      gripperController_->setMode(RobotSide::LEFT, mode_map[ui->cmbBoxGripMode->currentText()]);
      PREVIOUS_MODE_LEFT = ui->cmbBoxGripMode->currentText();
    }
  }
  else
  {
    if (!(PREVIOUS_MODE_RIGHT == ui->cmbBoxGripMode->currentText()))
    {
      gripperController_->setMode(RobotSide::RIGHT, mode_map[ui->cmbBoxGripMode->currentText()]);
      PREVIOUS_MODE_RIGHT = ui->cmbBoxGripMode->currentText();
    }
  }
}

void ToughGUI::closeGrippers()
{
  // call close grppiers function here
  setMode();
  RobotSide side = ui->radioGripSideLeft->isChecked() ? LEFT : RIGHT;
  gripperController_->closeGripper(side);
}

void ToughGUI::openGrippers()
{
  // call open grippers function here
  setMode();
  RobotSide side = ui->radioGripSideLeft->isChecked() ? LEFT : RIGHT;
  gripperController_->openGripper(side);
}

void ToughGUI::closeFingers()
{
  setMode();
  RobotSide side = ui->radioGripSideLeft->isChecked() ? LEFT : RIGHT;
  gripperController_->closeFingers(side);
}

void ToughGUI::openFingers()
{
  setMode();
  RobotSide side = ui->radioGripSideLeft->isChecked() ? LEFT : RIGHT;
  gripperController_->openFingers(side);
}

void ToughGUI::closeThumb()
{
  setMode();
  RobotSide side = ui->radioGripSideLeft->isChecked() ? LEFT : RIGHT;
  gripperController_->closeThumb(side);
}

void ToughGUI::openThumb()
{
  setMode();
  RobotSide side = ui->radioGripSideLeft->isChecked() ? LEFT : RIGHT;
  gripperController_->openThumb(side);
}

void ToughGUI::resetGrippers()
{
  gripperController_->resetGripper(RobotSide::LEFT);
  gripperController_->resetGripper(RobotSide::RIGHT);

  PREVIOUS_MODE_LEFT = "BASIC";
  PREVIOUS_MODE_RIGHT = "BASIC";

  updateGripperSide(-1);
}

void ToughGUI::closeBothGrippers()
{
  // Putting both grippers in BASIC mode.
  gripperController_->setMode(RobotSide::LEFT, GripperControlInterface::BASIC);
  PREVIOUS_MODE_LEFT = "BASIC";
  ros::Duration(0.1).sleep();

  gripperController_->setMode(RobotSide::RIGHT, GripperControlInterface::BASIC);
  PREVIOUS_MODE_RIGHT = "BASIC";
  ros::Duration(0.1).sleep();

  // Closing both hands
  gripperController_->closeGripper(RobotSide::LEFT);
  ros::Duration(0.1).sleep();
  gripperController_->closeGripper(RobotSide::RIGHT);

  updateGripperSide(-1);
}
void ToughGUI::setCurrentTool(int btnID)
{
  if (btnID == -2)
  {
    toolManager_->setCurrentTool(interactTool_);
    mapToolManager_->setCurrentTool(mapInteractTool_);
  }
  else if (btnID == -3)
  {
    toolManager_->setCurrentTool(measureTool_);
  }
  else if (btnID == -4)
  {
    toolManager_->setCurrentTool(setInitialPoseTool_);
    mapToolManager_->setCurrentTool(setMapInitialPoseTool_);
  }
  else if (btnID == -5)
  {
    toolManager_->setCurrentTool(setGoalTool_);
    mapManager_->getToolManager()->setCurrentTool(setMapGoalTool_);
    ui->controlTabs->setCurrentIndex(5);
  }
  else if (btnID == -6)
  {
    toolManager_->setCurrentTool(pointTool_);
  }

  changeToolButtonStatus(btnID);
}

void ToughGUI::changeToolButtonStatus(int btnID)
{
  ui->btnRvizInteract->setFlat(true);
  ui->btnRvizMeasure->setFlat(true);
  ui->btnRvizNavGoal->setFlat(true);
  ui->btnRvizPoseEstimate->setFlat(true);
  ui->btnRvizPublishPoint->setFlat(true);

  switch (btnID)
  {
    case -2:
      ui->btnRvizInteract->setFlat(false);
      break;
    case -3:
      ui->btnRvizMeasure->setFlat(false);
      break;
    case -4:
      ui->btnRvizPoseEstimate->setFlat(false);
      break;
    case -5:
      ui->btnRvizNavGoal->setFlat(false);
      break;
    case -6:
      ui->btnRvizPublishPoint->setFlat(false);
  }
}

void ToughGUI::alignFeet()
{
  RobotSide side = ui->radioLeftFoot->isChecked() ? LEFT : RIGHT;
  walkingController_->alignFeet(side);
}

void ToughGUI::walkSteps()
{
  RobotSide side = ui->radioLeftFoot->isChecked() ? LEFT : RIGHT;
  int numOfSteps = ui->lineEditNumSteps->text().toInt();
  float xOffset = ui->lineEditXOffset->text().toFloat();
  float yOffset = ui->lineEditYOffset->text().toFloat();
  float swingTime = ui->lineEditSwingTime->text().toFloat();
  float transferTime = ui->lineEditTransferTime->text().toFloat();
  float swingHeight = ui->lineEditSwingHeight->text().toFloat();
  if (walkingController_ != nullptr)
  {
    if (swingTime != swingTime_ || transferTime != transferTime_)
    {
      walkingController_->setWalkParams(transferTime, swingTime, 0);
      swingTime_ = swingTime;
      transferTime_ = transferTime;
    }

    if (swingHeight != swingHeight_)
    {
      walkingController_->setSwingHeight(swingHeight);
      swingHeight_ = swingHeight;
    }

    walkingController_->walkNStepsWRTPelvis(numOfSteps, xOffset, yOffset, side, false);
  }
}

void ToughGUI::approveSteps()
{
  std_msgs::Empty msg;
  approveStepsPub_.publish(msg);
}

void ToughGUI::abortSteps()
{
  walkingController_->abortWalk();
}
void ToughGUI::changePelvisHeight()
{
  float height = ui->sliderPelvisHeight->value() * (PELVIS_HEIGHT_MAX - PELVIS_HEIGHT_MIN) / 100.0f + PELVIS_HEIGHT_MIN;
  if (pelvisHeightController_ != nullptr)
  {
    pelvisHeightController_->controlPelvisHeight(height);
  }
}

void ToughGUI::moveArmJoints()
{
  if (armJointController_ == nullptr)
  {
    return;
  }

  std::vector<ArmControlInterface::ArmJointData> data;
  ArmControlInterface::ArmJointData msg;

  RobotSide side = ui->radioArmSideLeft->isChecked() ? LEFT : RIGHT;
  msg.side = side;
  msg.time = 1.0;

  // all joint limits should be saved in a map, that can be queried by jointName
  if (side == LEFT)
  {
    for (size_t i = 0; i < leftArmJointNames_.size(); i++)
    {
      double value = jointSliderMap_[leftArmJointNames_.at(i)]->value() *
                         (leftArmJointLimits_.at(i).second - leftArmJointLimits_.at(i).first) / 100.0f +
                     leftArmJointLimits_.at(i).first;
      msg.arm_pose.push_back(value);
    }
  }
  else
  {
    for (size_t i = 0; i < rightArmJointNames_.size(); i++)
    {
      double value = jointSliderMap_[rightArmJointNames_.at(i)]->value() *
                         (rightArmJointLimits_.at(i).second - rightArmJointLimits_.at(i).first) / 100.0f +
                     rightArmJointLimits_.at(i).first;
      msg.arm_pose.push_back(value);
    }
  }

  data.push_back(msg);

  // this is a non-blocking call
  armJointController_->moveArmJoints(data);
}

void ToughGUI::moveChestJoints()
{
  if (chestController_ == nullptr)
  {
    return;
  }

  auto getAngle = [this](int index) -> float {
    return jointSliderMap_[chestJointNames_.at(index)]->value() *
               (chestJointLimits_.at(index).second - chestJointLimits_.at(index).first) / 100.0f +
           chestJointLimits_.at(index).first;
  };

  enum joint
  {
    YAW = 0,
    PITCH,
    ROLL
  };

  chestController_->controlChest(getAngle(ROLL), getAngle(PITCH), getAngle(YAW));
  ros::spinOnce();
}

void ToughGUI::moveHeadJoints()
{
  float upperNeckPitchSliderValue =
      ui->sliderUpperNeckPitch->value() * (UPPER_NECK_PITCH_MAX - UPPER_NECK_PITCH_MIN) / 100.0f + UPPER_NECK_PITCH_MIN;
  float lowerNeckPitchSliderValue =
      ui->sliderLowerNeckPitch->value() * (LOWER_NECK_PITCH_MAX - LOWER_NECK_PITCH_MIN) / 100.0f + LOWER_NECK_PITCH_MIN;
  float neckYawSliderValue = -1 * (ui->sliderNeckYaw->value() * (NECK_YAW_MAX - NECK_YAW_MIN) / 100.0f + NECK_YAW_MIN);
  if (headController_ != nullptr)
  {
    headController_->moveHead(lowerNeckPitchSliderValue, upperNeckPitchSliderValue, neckYawSliderValue);
    ros::spinOnce();
  }
}
