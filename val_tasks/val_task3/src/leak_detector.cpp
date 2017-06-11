#include "val_task3/leak_detector.h"


leakDetector::leakDetector(ros::NodeHandle nh):
    nh_(nh)
{
    leak_sb_ = nh_.subscribe("/task3/checkpoint5/leak", 10, &leakDetector::leakMsgCB, this);
}

leakDetector::~leakDetector()
{
    //shutdown subscribers
    leak_sb_.shutdown();
}

void leakDetector::leakMsgCB(const srcsim::Leak &leakmsg)
{
  leak_value_ = leakmsg.value;
}


void leakDetector::generateSearchWayPoints(void)
{
  // generate way points with the dimension of the leak tool
}

// helper functions
double leakDetector::getLeakValue() const
{
    return leak_value_;
}

void leakDetector::setLeakValue(double leak_value)
{
    leak_value_ = leak_value;
}

