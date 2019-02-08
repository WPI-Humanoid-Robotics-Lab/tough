#include <gtest/gtest.h>
#include <math.h>
#include <climits>
#include <ros/ros.h>

// bad function:
// for example: how to deal with overflow?
int add(int a, int b)
{
  return a + b;
}

// a dummy test calling gtest's assert_srteq
// this test is not related with any ros nodeHandle
TEST(AddNumbeRtest, ShouldPass)
{
  ASSERT_EQ(3, add(1, 2));
}

// TEST(NumberCmpTest, ShouldFail){
//    ASSERT_EQ(INT_MAX, add(INT_MAX, 1));
//}

int main(int argc, char** argv)
{
  testing::InitGoogleTest(&argc, argv);
  // do not forget to init ros because this is also a node
  ros::init(argc, argv, "test_pelvis");
  return RUN_ALL_TESTS();
}
