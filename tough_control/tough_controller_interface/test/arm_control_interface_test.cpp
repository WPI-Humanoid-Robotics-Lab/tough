#include <gtest/gtest.h>
#include <math.h>
#include <climits>
#include <ros/ros.h>
#include <tough_controller_interface/arm_control_interface.h>

ros::NodeHandle* nh=nullptr;

int add(int a, int b){
    std::cout<<"add executed!!!!!!! \n";
    return a + b;
}

bool initialize()
{
    if(nh !=nullptr){
        ArmControlInterface armTraj(*nh);
        std::cout<<"code is working \n";
        armTraj.moveToZeroPose(RobotSide::LEFT);
//        armTraj.testPrint();
        return true;
    }
    return false;
}

// EXPECT_EQ and ASSERT_EQ are also macros—in the former case test execution continues even if there is a failure while in the latter case test execution aborts

class myTestFixture1: public ::testing::Test {
public:
    myTestFixture1( ) {
        // initialization code here
        std::cout<<"initialization code is being executed \n";



    }

    void SetUp( ) {
        std::cout<<"set up code is being executed \n";
    }

    void TearDown( ) {
        // code here will be called just after the test completes
        // ok to through exceptions from here if need be
        std::cout<<"tear down code is being executed \n";
    }

    ~myTestFixture1( )  {
        std::cout<<"destructor code is being executed \n";
        // cleanup any pending stuff, but no exceptions allowed
    }

    // put in any custom data members that you need
};

TEST_F (myTestFixture1, UnitTest1) {
    initialize();
//    ASSERT_TRUE(initialize());
    ASSERT_EQ(3, add(1,2));
}


int main(int argc, char **argv) {
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "test_arm_unit_test");
    ros::NodeHandle nh_;
    nh = &nh_;
    std::cout<<"init executed!!!!!!! \n";
    // do not forget to init ros because this is also a node
    return RUN_ALL_TESTS();
}
