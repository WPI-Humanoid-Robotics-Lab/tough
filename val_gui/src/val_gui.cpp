#include "val_gui.h"
#include "ui_val_gui.h"
#include <iostream>
#include "rviz/view_manager.h"
#include "rviz/tool_manager.h"
#include "rviz/properties/property_tree_model.h"
#include "configurationreader.h"
#include "ros/package.h"
#include "val_common/val_common_defines.h"

/**
 * This class creates the GUI using rviz APIs.
 */

ValkyrieGUI::ValkyrieGUI(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ValkyrieGUI),it_(nh_)
{
    /**
     * Set up the QT related UI components.
     */
    ui->setupUi(this);
    //    ui->sliderLinearVel->setValue(75);
    //    ui->sliderAngularVel->setValue(75);
    changeToolButtonStatus(-2); //set the initial rviz tool to be "interact"

    //set all the controller pointers to null
    chestController_        = nullptr;
    pelvisHeightController_ = nullptr;
    armJointController_     = nullptr;
    walkingController_      = nullptr;

    //initialize everything
    initVariables();
    initDisplayWidgets();
    initTools();
    initActionsConnections();
    initDefaultValues();
    initValkyrieControllers();
}

ValkyrieGUI::~ValkyrieGUI()
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
}

void ValkyrieGUI::initVariables()
{
    /**
     *Initialize default values of all the variables. Push these definitions to xml/config file in future
     */

    //Read the configuration file
    std::string configFile = ros::package::getPath("val_gui") + "/config/config.ini";
    ROS_INFO("config file : %s", configFile.c_str());
    ConfigurationReader configfile(configFile.c_str());

    //Assign topic names to corresponding variables
    fixedFrame_     = QString::fromStdString(configfile.currentTopics["fixedFrame"]);
    mapTopic_       = QString::fromStdString(configfile.currentTopics["mapTopic"]);
    imageTopic_     = QString::fromStdString(configfile.currentTopics["imageTopic"]);
    pointCloudTopic_= QString::fromStdString(configfile.currentTopics["pointCloudTopic"]);
    octomapTopic_   = QString::fromStdString(configfile.currentTopics["octomapTopic"]);
    baseSensorTopic_= QString::fromStdString(configfile.currentTopics["baseSensorTopic"]);
    velocityTopic_  = QString::fromStdString(configfile.currentTopics["velocityTopic"]);
    pathTopic_      = QString::fromStdString(configfile.currentTopics["pathTopic"]);
    targetFrame_    = QString::fromStdString(configfile.currentTopics["targetFrame"]);
    robotType_      = QString::fromStdString(configfile.currentTopics["robotType"]);

    moveBaseCmdPub  = nh_.advertise<geometry_msgs::Twist>(velocityTopic_.toStdString(),1);
    //    baseSensorStatus = nh_.subscribe(baseSensorTopic_.toStdString(),1,&FallRiskGUI::baseStatusCheck,this);
    liveVideoSub    = it_.subscribe(imageTopic_.toStdString(),1,&ValkyrieGUI::liveVideoCallback,this,image_transport::TransportHints("raw"));

    jointStateSub_ = nh_.subscribe("/joint_states",1, &ValkyrieGUI::jointStateCallBack, this);

}

void ValkyrieGUI::initActionsConnections()
{
    /**
     * Set up the status Bar and display messages emitted from each of the tools.
     * All the tools in rviz API has updateStatus function to emit messages to the status bar.
     */
    status_label_ = new QLabel("");
    statusBar()->addPermanentWidget( status_label_,1);
    connect( manager_, SIGNAL( statusUpdate( const QString& )), status_label_, SLOT( setText( const QString& )));

    /**
     * Setup Signals and slots for different buttons/sliders in UI.
     */
    // Tool and display selection
    connect(ui->btnGroupRvizTools,       SIGNAL(buttonClicked(int)),   this, SLOT(setCurrentTool(int)));
    connect(ui->btnGroupDisplays,        SIGNAL(buttonClicked(int)),   this, SLOT(displayPointcloud(int)));
    connect(ui->controlTabs,             SIGNAL(currentChanged(int)),  this, SLOT(updateJointStateSub(int)));

    //arm control
    connect(ui->btnCloseHand,            SIGNAL(clicked()),            this, SLOT(closeGrippers()));
    connect(ui->btnOpenHand,             SIGNAL(clicked()),            this, SLOT(openGrippers()));
    connect(ui->sliderShoulderRoll,      SIGNAL(sliderReleased()),     this, SLOT(moveArmJoints()));
    connect(ui->sliderShoulderPitch,     SIGNAL(sliderReleased()),     this, SLOT(moveArmJoints()));
    connect(ui->sliderShoulderYaw,       SIGNAL(sliderReleased()),     this, SLOT(moveArmJoints()));
    connect(ui->sliderWristRoll,         SIGNAL(sliderReleased()),     this, SLOT(moveArmJoints()));
    connect(ui->sliderWristPitch,        SIGNAL(sliderReleased()),     this, SLOT(moveArmJoints()));
    connect(ui->sliderWristYaw,          SIGNAL(sliderReleased()),     this, SLOT(moveArmJoints()));
    connect(ui->sliderElbow,             SIGNAL(sliderReleased()),     this, SLOT(moveArmJoints()));

    // chest control
    connect(ui->sliderChestRoll,         SIGNAL(sliderReleased()),    this, SLOT(moveChestJoints()));
    connect(ui->sliderChestPitch,        SIGNAL(sliderReleased()),    this, SLOT(moveChestJoints()));
    connect(ui->sliderChestYaw,          SIGNAL(sliderReleased()),    this, SLOT(moveChestJoints()));

    // neck control
    //    connect(ui->sliderNeckRoll,          SIGNAL(valueChanged(int)),    this, SLOT(moveChestJoints(int)));
    //    connect(ui->sliderNeckPitch,         SIGNAL(valueChanged(int)),    this, SLOT(moveChestJoints(int)));
    //    connect(ui->sliderNeckYaw,           SIGNAL(valueChanged(int)),    this, SLOT(moveChestJoints(int)));

    //walk
    connect(ui->btnWalk,                 SIGNAL(clicked()),            this, SLOT(walkSteps()));
    connect(ui->sliderPelvisHeight,      SIGNAL(sliderReleased()),     this, SLOT(changePelvisHeight()));


}

void ValkyrieGUI::initDisplayWidgets()
{

    //Setup the UI elements for displaying 2D map
    /**
     * VisualizationManager is used to control different displays that are shown in a widget.
     * Renderpanel is a widget that provides a 3D space in the visualizationmanager.
     * startUpdate() function starts the timers and subscribes to defined topics at 30Hz.
     */
    mapRenderPanel_ = new rviz::RenderPanel();
    ui->map_layout->addWidget(mapRenderPanel_);
    mapManager_     = new rviz::VisualizationManager( mapRenderPanel_ );
    mapRenderPanel_->initialize( mapManager_->getSceneManager(), mapManager_);
    mapManager_->setFixedFrame(fixedFrame_);
    mapManager_->initialize();
    mapManager_->startUpdate();

    //Create and assign FixedOrientationOrthoViewController to the existing viewmanager of the visualization manager
    /**
     * VisualisationManager has a manager for most of its children. ViewManager is responsible for setting the viewController.
     * Default View Controller is rviz/Orbit, for map we are changing it to rviz/TopDownOrtho
     * To set properties of most of the rviz objects, use subProp and setValue functions as shown below
     * New displays can be created and added to the visualization manager using createDisplay function as used below
     *
     * @todo Create an xml/config file to define objects to be displayed in GUI alongwith their parameters
     */

    mapViewManager_     = mapManager_->getViewManager();
    mapViewManager_->setCurrentViewControllerType("rviz/TopDownOrtho");
    mapViewController_  = mapViewManager_->getCurrent();

    //Set parameters of the view controller to show map correctly
    mapViewController_->subProp("X")->setValue(0);
    mapViewController_->subProp("Y")->setValue(0);
    mapViewController_->subProp("Angle")->setValue(0);
    mapViewController_->subProp("Scale")->setValue(20);

    // Create a map display
    mapDisplay_ = mapManager_->createDisplay( "rviz/Map", "2D Map view", true );
    ROS_ASSERT( mapDisplay_ != NULL );

    mapDisplay_->subProp( "Topic" )->setValue( mapTopic_ );
    mapManager_->createDisplay( "rviz/RobotModel", robotType_, true );
    mapManager_->createDisplay("rviz/Path","Global path",true)->subProp( "Topic" )->setValue(pathTopic_);


    // Initialize GUI elements for main panel
    renderPanel_ = new rviz::RenderPanel();
    ui->display3d_layout->addWidget(renderPanel_);

    manager_ = new rviz::VisualizationManager( renderPanel_ );
    renderPanel_->initialize( manager_->getSceneManager(), manager_ );

    //set the fixed frame before initializing Visualization Manager. pointcloud2 will not work with this
    manager_->setFixedFrame(fixedFrame_);
    manager_->initialize();
    manager_->startUpdate();

    // Create a main display to show pointcloud and octomap
    manager_->createDisplay( "rviz/Grid", "Grid", true );
    manager_->createDisplay( "rviz/RobotModel", robotType_, true );

    cloudDisplay_ = manager_->createDisplay( "rviz/PointCloud2", "3D Pointcloud view", false );
    assert( cloudDisplay_ != NULL && "Could not create a display");

    cloudDisplay_->subProp( "Topic" )->setValue( pointCloudTopic_ );
    cloudDisplay_->subProp( "Selectable" )->setValue( "true" );
    cloudDisplay_->subProp( "Style" )->setValue( "Boxes" );
    cloudDisplay_->subProp("Alpha")->setValue(0.5);

    octomapDisplay_ = manager_->createDisplay( "rviz/MarkerArray", "Octomap view", false );
    ROS_ASSERT( octomapDisplay_ != NULL );

    octomapDisplay_->subProp( "Marker Topic" )->setValue(octomapTopic_);

    //Assign Target Frame to the existing viewmanager of the visualization manager
    rviz::ViewManager*    viewManager_    = manager_->getViewManager();
    rviz::ViewController* viewController_ = viewManager_->getCurrent();
    viewController_->subProp("Target Frame")->setValue(targetFrame_);
    manager_->createDisplay("rviz/Path","Global path",true)->subProp( "Topic" )->setValue(pathTopic_);


    //fix things that cannot be changed in UI
    //    ui->tabArmControlPage1->setObjectName("Shoulder");
    //    ui->tabArmControlPage2->setObjectName("Wrist");
}

void ValkyrieGUI::initTools(){
    /**
     * ToolManager is similar to ViewManager. It can be used to add new tools and change the current or default tool.
     * Properties of tools are stored in a PropertyTreeModel. To set/modify any property of a tool use getPropertyContainer function.
     */
    toolManager_           = manager_->getToolManager();
    pointTool_             = toolManager_->addTool("rviz/PublishPoint");
    measureTool_           = toolManager_->addTool("rviz/Measure");
    setGoalTool_           = toolManager_->addTool("rviz/SetGoal");
    setInitialPoseTool_    = toolManager_->addTool("rviz/SetInitialPose");
    interactTool_          = toolManager_->addTool("rviz/Interact");
    mapToolManager_        = mapManager_->getToolManager();
    mapInteractTool_       = mapToolManager_->addTool("rviz/Interact");
    setMapGoalTool_        = mapToolManager_->addTool("rviz/SetGoal");
    setMapInitialPoseTool_ = mapToolManager_->addTool("rviz/SetInitialPose");

    // Find the entry in propertytreemodel and set the value for Topic
    setGoalTool_->getPropertyContainer()->subProp("Topic")->setValue("/move_base_simple/goal");
    setMapGoalTool_->getPropertyContainer()->subProp("Topic")->setValue("/move_base_simple/goal");

}

void ValkyrieGUI::initDefaultValues() {
    // 3D view. select none by default
    ui->radioBtnNone->setChecked(true);

    //Arms select left arm by default
    ui->radioArmSideLeft->setChecked(true);

    //Set the default values for the offset
    ui->lineEditNumSteps->setText("2");
    ui->lineEditXOffset->setText("0.3");
    ui->lineEditYOffset->setText("0.0");

    //check the right foot start button
    ui->radioRightFoot->setChecked(true);

    //query current state of robot and update sliders accordingly
    //Chest control . Replace these defaults with actual values from robot
    float zeroRoll  = fabs(CHEST_ROLL_MIN/((CHEST_ROLL_MAX - CHEST_ROLL_MIN)/100.0));
    float zeroPitch = fabs(CHEST_PITCH_MIN/((CHEST_PITCH_MAX - CHEST_PITCH_MIN)/100.0));
    float zeroYaw   = fabs(CHEST_YAW_MIN/((CHEST_YAW_MAX - CHEST_YAW_MIN)/100.0));
    ui->sliderChestRoll->setValue(zeroRoll);
    ui->sliderChestPitch->setValue(zeroPitch);
    ui->sliderChestYaw->setValue(zeroYaw);

    //PelvisHeight . Replace these defaults with actual values from robot
    float defaultPelvisHeight = (0.9 - PELVIS_HEIGHT_MIN) *100/ (PELVIS_HEIGHT_MAX - PELVIS_HEIGHT_MIN);
    ui->sliderPelvisHeight->setValue(defaultPelvisHeight);

    //configure arm sliders
    getArmState();

//    jointStateSub_.shutdown();
}

void ValkyrieGUI::initValkyrieControllers() {

    //create a chest trajectory controller object
    chestController_ = new chestTrajectory(nh_);

    //create pelvis height controller object
    pelvisHeightController_ = new pelvisTrajectory(nh_);

    //create walking controller object
    walkingController_ = new ValkyrieWalker(nh_, 1.0, 1.0, 0, 0.18);

    //create arm joint controller object
    armJointController_ = new armTrajectory(nh_);
}

void ValkyrieGUI::getArmState()
{

    armSide side = ui->radioArmSideLeft->isChecked() ? LEFT : RIGHT;
    mtx_.lock();
    for (size_t i = 0; i < jointNames_.size(); i++){
        if(side == LEFT) {
            if (jointNames_[i] == "leftShoulderPitch"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                jointVal = (jointVal-LEFT_SHOULDER_PITCH_MIN) *(100/(LEFT_SHOULDER_PITCH_MAX-LEFT_SHOULDER_PITCH_MIN));
                ui->sliderShoulderPitch->setValue(jointVal);
                continue;

            }
            else if (jointNames_[i] == "leftShoulderRoll"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                jointVal = (jointVal - LEFT_SHOULDER_ROLL_MIN) *(100/(LEFT_SHOULDER_ROLL_MAX-LEFT_SHOULDER_ROLL_MIN));
                ui->sliderShoulderRoll->setValue(jointVal);
                continue;
            }
            else if (jointNames_[i] == "leftShoulderYaw"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                jointVal = (jointVal - LEFT_SHOULDER_YAW_MIN) *(100/(LEFT_SHOULDER_YAW_MAX-LEFT_SHOULDER_YAW_MIN));
                ui->sliderShoulderYaw->setValue(jointVal);
                continue;
            }
            else if (jointNames_[i] == "leftElbowPitch"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                ui->sliderElbow->setValue(jointVal *(100/(LEFT_ELBOW_MAX-LEFT_ELBOW_MIN)));
                continue;
            }
            else if (jointNames_[i] == "leftForearmYaw"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                ui->sliderWristYaw->setValue(jointVal *(100/(LEFT_WRIST_YAW_MAX-LEFT_WRIST_YAW_MIN)));
                continue;
            }
            else if (jointNames_[i] == "leftWristRoll"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                ui->sliderWristRoll->setValue(jointVal *(100/(LEFT_WRIST_ROLL_MAX-LEFT_WRIST_ROLL_MIN)));
                continue;
            }
            else if (jointNames_[i] == "leftWristPitch"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                ui->sliderWristPitch->setValue(jointVal *(100/(LEFT_WRIST_PITCH_MAX-LEFT_WRIST_PITCH_MIN)));
                continue;
            }
        }
        else{

            if (jointNames_[i] == "rightShoulderPitch"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                jointVal = jointVal *(100/(RIGHT_SHOULDER_PITCH_MAX-RIGHT_SHOULDER_PITCH_MIN));
                ui->sliderShoulderPitch->setValue(jointVal);
                ROS_INFO("Set the shoulder pitch to %.2f rad and %.2f degrees",jointValues_[i], jointVal);
                continue;
            }
            else if (jointNames_[i] == "rightShoulderRoll"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                jointVal = jointVal *(100/(RIGHT_SHOULDER_ROLL_MAX-RIGHT_SHOULDER_ROLL_MIN));
                ui->sliderShoulderRoll->setValue(jointVal);
                continue;
            }
            else if (jointNames_[i] == "rightShoulderYaw"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                ui->sliderShoulderYaw->setValue(jointVal *(100/(RIGHT_SHOULDER_YAW_MAX-RIGHT_SHOULDER_YAW_MIN)));
                continue;
            }
            else if (jointNames_[i] == "rightElbowPitch"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                ui->sliderElbow->setValue(jointVal *(100/(RIGHT_ELBOW_MAX-RIGHT_ELBOW_MIN)));
                continue;
            }
            else if (jointNames_[i] == "rightForearmYaw"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                ui->sliderWristYaw->setValue(jointVal *(100/(RIGHT_WRIST_YAW_MAX-RIGHT_WRIST_YAW_MIN)));
                continue;
            }
            else if (jointNames_[i] == "rightWristRoll"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                ui->sliderWristRoll->setValue(jointVal *(100/(RIGHT_WRIST_ROLL_MAX-RIGHT_WRIST_ROLL_MIN)));
                continue;
            }
            else if (jointNames_[i] == "rightWristPitch"){
                double jointVal = jointValues_[i] * TO_DEGREES;
                ui->sliderWristPitch->setValue(jointVal *(100/(RIGHT_WRIST_PITCH_MAX-RIGHT_WRIST_PITCH_MIN)));
                continue;
            }
        }

    }

    mtx_.unlock();

}

void ValkyrieGUI::getChestState()
{

}

void ValkyrieGUI::getPelvisState()
{

}

void ValkyrieGUI::getNeckState()
{

}

void ValkyrieGUI::getGripperState()
{

}

void ValkyrieGUI::jointStateCallBack(const sensor_msgs::JointStatePtr &state)
{
    jointNames_ = state->name;
    jointValues_ = state->position;

}

void ValkyrieGUI::updateJointStateSub(int tabID){
    //0 = arm
    //1 = chest
    //2 = neck
    //3 = walk
//    jointStateSub_ = nh_.subscribe("/joint_states",1, &ValkyrieGUI::jointStateCallBack, this);
    switch (tabID) {
    case 0:
        getArmState();
        getGripperState();
        break;
    case 1:
        getChestState();
        break;
    case 2:
        getNeckState();
        break;
    case 3:
        getPelvisState();
        break;
    default:
        break;
    }

//    jointStateSub_.shutdown();

}

void ValkyrieGUI::keyPressEvent(QKeyEvent *event)
{
    switch(event->key())
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

void ValkyrieGUI::liveVideoCallback(const sensor_msgs::ImageConstPtr& msg)
{

    /**
     * Adding Image display opens up the image in a new window.
     * As a workaround to show image in the same GUI window, OpenCV is being used to display image on a Qlabel
     */
    cv_bridge::CvImagePtr cv_ptr, cv_ptr_big;
    bool is_rgb = false;
    try{
        if(msg->encoding == sensor_msgs::image_encodings::RGB8 || msg->encoding == sensor_msgs::image_encodings::RGB16) {
            cv_ptr     = cv_bridge::toCvCopy(msg, msg->encoding);
            is_rgb = true;
        }

        else{
            cv_ptr     = cv_bridge::toCvCopy(msg, msg->encoding);
        }
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    //  convert cv image into RGB image and resize it to the size of available layout
    setVideo(ui->liveVideoLabel,cv_ptr, is_rgb);
}

void ValkyrieGUI::setVideo(QLabel* label, cv_bridge::CvImagePtr cv_ptr, bool is_RGB){
    cv::Mat RGBImg;
    QLabel* liveVideoLabel = label;

    // To avoid auto expansion of QLabel,keep the video dimensions slightly less than the label dimension
    int height = liveVideoLabel->height()-1;
    int width  = liveVideoLabel->width()-1;

    if(liveVideoLabel->height()-1 >= (liveVideoLabel->width()-1)*544/1024)
        height = (liveVideoLabel->width()-1)*544/1024;
    else
        width  = (liveVideoLabel->height()-1)*1024/544;
    if (is_RGB){
        RGBImg = cv_ptr->image;
    }
    else{
        cv::cvtColor(cv_ptr->image, RGBImg, CV_BGR2RGB);
    }
    cv::resize(RGBImg, RGBImg, cvSize(width, height));
    //flip the image
    cv::flip(RGBImg, RGBImg, -1);
    //  convert RGB image into QImage and publish that on the label for livevideo
    QImage qImage_= QImage((uchar*) RGBImg.data, RGBImg.cols, RGBImg.rows, RGBImg.cols*3, QImage::Format_RGB888);
    liveVideoLabel->setPixmap(QPixmap::fromImage(qImage_));
    liveVideoLabel->show();

}

void ValkyrieGUI::closeGrippers()
{
    ROS_INFO("closing Grippers");
    //call close grppiers function here

}

void ValkyrieGUI::openGrippers()
{
    ROS_INFO("Opening Grippers");
    //call open grippers function here
}

void ValkyrieGUI::setCurrentTool(int btnID)
{
    if(btnID == -2)
    {
        ROS_INFO("Interact Tool Selected");
        toolManager_->setCurrentTool(interactTool_);
        mapToolManager_->setCurrentTool(mapInteractTool_);

    }
    else if(btnID == -3)
    {
        ROS_INFO("Measure Tool Selected");
        toolManager_->setCurrentTool(measureTool_);

    }
    else if(btnID == -4)
    {
        ROS_INFO("2DPoseEstimate Tool Selected");
        toolManager_->setCurrentTool(setInitialPoseTool_);
        mapToolManager_->setCurrentTool(setMapInitialPoseTool_);
    }
    else if(btnID == -5)
    {
        ROS_INFO("2DNavGoal Tool Selected");
        toolManager_->setCurrentTool(setGoalTool_);
        mapManager_->getToolManager()->setCurrentTool(setMapGoalTool_);
    }
    else if(btnID == -6)
    {
        ROS_INFO("PublishPoint Tool Selected");
        toolManager_->setCurrentTool(pointTool_);
    }

    changeToolButtonStatus(btnID);
}

void ValkyrieGUI::changeToolButtonStatus(int btnID)
{
    ui->btnRvizInteract->setFlat(true);
    ui->btnRvizMeasure->setFlat(true);
    ui->btnRvizNavGoal->setFlat(true);
    ui->btnRvizPoseEstimate->setFlat(true);
    ui->btnRvizPublishPoint->setFlat(true);

    switch(btnID)
    {
    case -2: ui->btnRvizInteract->setFlat(false);
        break;
    case -3: ui->btnRvizMeasure->setFlat(false);
        break;
    case -4: ui->btnRvizPoseEstimate->setFlat(false);
        break;
    case -5: ui->btnRvizNavGoal->setFlat(false);
        break;
    case -6: ui->btnRvizPublishPoint->setFlat(false);
    }
}

void ValkyrieGUI::setActiveRvizToolBtns(int tabID)
{
    //    ROS_INFO("TAB:%d",tabID);

    ui->btnRvizInteract->setDisabled(false);
    ui->btnRvizMeasure->setDisabled(false);
    ui->btnRvizPoseEstimate->setDisabled(false);
    ui->btnRvizNavGoal->setDisabled(false);
    ui->btnRvizPublishPoint->setDisabled(false);

    if(tabID == 1)
    {
        ui->btnRvizMeasure->setDisabled(true);
        ui->btnRvizPublishPoint->setDisabled(true);
    }
    else if(tabID == 2)
    {
        ui->btnRvizInteract->setDisabled(true);
        ui->btnRvizMeasure->setDisabled(true);
        ui->btnRvizPoseEstimate->setDisabled(true);
        ui->btnRvizNavGoal->setDisabled(true);
        ui->btnRvizPublishPoint->setDisabled(true);
    }
}

void ValkyrieGUI::displayPointcloud(int btnID)
{

    switch (btnID) {
    case -4:    // button ID of octomap
        octomapDisplay_->setEnabled(true);
        cloudDisplay_->setEnabled(false);
        break;
    case -2:    // button ID of Pointcloud
        octomapDisplay_->setEnabled(false);
        cloudDisplay_->setEnabled(true);
        break;
    case -3:    //button ID of None
        octomapDisplay_->setEnabled(false);
        cloudDisplay_->setEnabled(false);
    default:
        break;
    }
}

void ValkyrieGUI::walkSteps()
{

    armSide side = ui->radioLeftFoot->isChecked() ? LEFT : RIGHT;
    int numOfSteps = ui->lineEditNumSteps->text().toInt();
    float xOffset = ui->lineEditXOffset->text().toFloat();
    float yOffset = ui->lineEditYOffset->text().toFloat();
    if(walkingController_ != nullptr){
        walkingController_->walkNSteps(numOfSteps, xOffset, yOffset, false, side, false);
    }
}

void ValkyrieGUI::changePelvisHeight(){

    float height = ui->sliderPelvisHeight->value()*(PELVIS_HEIGHT_MAX-PELVIS_HEIGHT_MIN)/100+PELVIS_HEIGHT_MIN;
    if(pelvisHeightController_ != nullptr){
        pelvisHeightController_->controlPelvisHeight(height);
    }

}

void ValkyrieGUI::moveArmJoints(){
    if(armJointController_ == nullptr) {
        return;
    }
    float shoulderRollValue ;
    float shoulderPitchValue;
    float shoulderYawValue  ;

    float wristRollValue    ;
    float wristPitchValue   ;
    float wristYawValue     ;

    float elbowValue        ;

    armSide side = ui->radioArmSideLeft->isChecked() ? LEFT : RIGHT;
    if(side == LEFT)
    {
        shoulderRollValue  = ui->sliderShoulderRoll->value()*(LEFT_SHOULDER_ROLL_MAX-LEFT_SHOULDER_ROLL_MIN)/100+LEFT_SHOULDER_ROLL_MIN;
        shoulderPitchValue = ui->sliderShoulderPitch->value()*(LEFT_SHOULDER_PITCH_MAX-LEFT_SHOULDER_PITCH_MIN)/100+LEFT_SHOULDER_PITCH_MIN;
        shoulderYawValue   = ui->sliderShoulderYaw->value()*(LEFT_SHOULDER_YAW_MAX-LEFT_SHOULDER_YAW_MIN)/100+LEFT_SHOULDER_YAW_MIN;

        wristRollValue     = ui->sliderWristRoll->value()*(LEFT_WRIST_ROLL_MAX-LEFT_WRIST_ROLL_MIN)/100+LEFT_WRIST_ROLL_MIN;
        wristPitchValue    = ui->sliderWristPitch->value()*(LEFT_WRIST_PITCH_MAX-LEFT_WRIST_PITCH_MIN)/100+LEFT_WRIST_PITCH_MIN;
        wristYawValue      = ui->sliderWristYaw->value()*(LEFT_WRIST_YAW_MAX-LEFT_WRIST_YAW_MIN)/100+LEFT_WRIST_YAW_MIN;

        elbowValue         = ui->sliderElbow->value()*(LEFT_ELBOW_MAX-LEFT_ELBOW_MIN)/100+LEFT_ELBOW_MIN;
    }
    else {
        shoulderRollValue  = ui->sliderShoulderRoll->value()*(RIGHT_SHOULDER_ROLL_MAX-RIGHT_SHOULDER_ROLL_MIN)/100+RIGHT_SHOULDER_ROLL_MIN;
        shoulderPitchValue = ui->sliderShoulderPitch->value()*(RIGHT_SHOULDER_PITCH_MAX-RIGHT_SHOULDER_PITCH_MIN)/100+RIGHT_SHOULDER_PITCH_MIN;
        shoulderYawValue   = ui->sliderShoulderYaw->value()*(RIGHT_SHOULDER_YAW_MAX-RIGHT_SHOULDER_YAW_MIN)/100+RIGHT_SHOULDER_YAW_MIN;

        wristRollValue     = ui->sliderWristRoll->value()*(RIGHT_WRIST_ROLL_MAX-RIGHT_WRIST_ROLL_MIN)/100+RIGHT_WRIST_ROLL_MIN;
        wristPitchValue    = ui->sliderWristPitch->value()*(RIGHT_WRIST_PITCH_MAX-RIGHT_WRIST_PITCH_MIN)/100+RIGHT_WRIST_PITCH_MIN;
        wristYawValue      = ui->sliderWristYaw->value()*(RIGHT_WRIST_YAW_MAX-RIGHT_WRIST_YAW_MIN)/100+RIGHT_WRIST_YAW_MIN;

        elbowValue         = ui->sliderElbow->value()*(RIGHT_ELBOW_MAX-RIGHT_ELBOW_MIN)/100+RIGHT_ELBOW_MIN;
    }


    std::vector<armTrajectory::armJointData> data;
    armTrajectory::armJointData msg;
    //sequence of joints for sending arm data
    std::vector<std::string> joints = {"leftShoulderPitch", "leftShoulderRoll", "leftShoulderYaw", "leftElbowPitch", "leftForearmYaw", "leftWristRoll", "leftWristPitch"};
    msg.arm_pose = {shoulderPitchValue * TO_RADIANS, shoulderRollValue* TO_RADIANS, shoulderYawValue* TO_RADIANS,
                    elbowValue* TO_RADIANS, wristYawValue* TO_RADIANS, wristRollValue* TO_RADIANS, wristPitchValue* TO_RADIANS};
    msg.side = side;
    msg.time = 0.0;
    for (size_t i = 0; i< msg.arm_pose.size(); i++){
        ROS_INFO("%s : %0.2f", joints[i].c_str(), msg.arm_pose[i]);
    }
    data.push_back(msg);
    armJointController_->moveArmJoints(data);

}

void ValkyrieGUI::moveChestJoints()
{
    //    ROS_INFO("TAB:%d",tabID);
    //    if (tabID == 1)
    //    {
    //        ROS_INFO("TAB:%d",tabID);
    //        shoulderrollslider_ = ui->sliderShoulderRoll->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    //        shoulderpitchslider_ = ui->sliderShoulderPitch->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    //        shoulderyawslider_ = ui->sliderShoulderYaw->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;

    //        wristrollslider_ = ui->sliderWristRoll->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    //        wristpitchslider_ = ui->sliderWristPitch->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    //        wristyawslider_ = ui->sliderWristYaw->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;

    //        elbowslider_ = ui->sliderElbow->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    //    }

    //    else if(tabID == 2)
    //    {
    float chestRollSliderValue = ui->sliderChestRoll->value()*(CHEST_ROLL_MAX-CHEST_ROLL_MIN)/100+CHEST_ROLL_MIN;
    float chestPitchSliderValue = ui->sliderChestPitch->value()*(CHEST_PITCH_MAX-CHEST_PITCH_MIN)/100 + CHEST_PITCH_MIN;
    float chestYawSliderValue = -1.0f * (ui->sliderChestYaw->value()*(CHEST_YAW_MAX-CHEST_YAW_MIN)/100 + CHEST_YAW_MIN); // this is to align yaw with sliding direction
    if(chestController_ != nullptr){
        chestController_->controlChest(chestRollSliderValue, chestPitchSliderValue, chestYawSliderValue);
        ros::spinOnce();
    }
    //    }

    //    else if(tabID == 3)
    //    {
    //        ROS_INFO("TAB:%d",tabID);
    //        neckrollslider_ = ui->sliderNeckRoll->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    //        neckpitchslider_ = ui->sliderNeckPitch->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    //        neckyawslider_ = ui->sliderNeckYaw->value()*(LIN_VEL_MAX-LIN_VEL_MIN)/100+LIN_VEL_MIN;
    //    }
}

/*
//Image :
    grid_ = manager_->createDisplay( "rviz/Image", "Image View", true );
    ROS_ASSERT( grid_ != NULL );
    grid_->subProp( "Image Topic" )->setValue( "/camera/rgb/image_raw" );
    grid_->subProp( "Transport Hint" )->setValue( "theora" );


//Depth Cloud :
    grid_ = manager_->createDisplay( "rviz/DepthCloud", "Image View", true );
    ROS_ASSERT( grid_ != NULL );

    grid_->subProp( "Depth Map Topic" )->setValue( "/camera/depth/image_raw" );
    grid_->subProp( "Depth Map Transport Hint" )->setValue( "raw" );
    grid_->subProp( "Color Image Topic" )->setValue( "/camera/rgb/image_raw" );
    grid_->subProp( "Color Transport Hint" )->setValue( "raw" );
    grid_->subProp("Queue Size")->setValue(5);
    grid_->subProp("Style")->setValue("Flat Squares");

//    mainDisplay_ = manager_->createDisplay( "rviz/PointCloud2", "3D Pointcloud view", true );
//    ROS_ASSERT( mainDisplay_ != NULL );

//    mainDisplay_->subProp( "Topic" )->setValue( pointCloudTopic_ );
//    mainDisplay_->subProp( "Selectable" )->setValue( "true" );
//    mainDisplay_->subProp( "Style" )->setValue( "Boxes" );
//    mainDisplay_->subProp("Alpha")->setValue(0.5);

  manager_->createDisplay( "rviz/Grid", "Grid", true );

//MarkerArray :
rviz::Display* octomapDisplay_ = manager_->createDisplay( "rviz/MarkerArray", "Octomap view", true );
ROS_ASSERT( octomapDisplay_ != NULL );

octomapDisplay_->subProp( "Marker Topic" )->setValue( "/occupied_cells_vis_array" );


*/

