#include <humanoid_common/standing_stable_controller.h>
#include <val_interface/val_JointState.h>


//////////
// test file for the standing controller
//////////

class testsatndingController {
public:
    testsatndingController(ros::NodeHandle nh);
    ~testsatndingController();
    void test(void);

protected:
    ros::NodeHandle nh_;
    stableController* controller_;
    valJointState* joint;
    std::vector<double> thetas;
};

testsatndingController::testsatndingController(ros::NodeHandle nh)
    : nh_(nh)
{
    joint = new valJointState(nh);
    controller_ = new stableController(nh, "true");
}

testsatndingController::~testsatndingController(){

}

void testsatndingController::test(void){

}

// test code
int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_com");
    ros::NodeHandle nh;

    ros::Rate loop_rate(100);

    testsatndingController T(nh);

    while(ros::ok())
    {
        T.test();
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}

