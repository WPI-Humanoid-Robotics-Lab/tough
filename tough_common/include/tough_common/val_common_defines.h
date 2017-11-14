# pragma once

// defines for arms
enum RobotSide
{
    LEFT = 0,
    RIGHT
};

enum class direction{
    LEFT = 0,  //Positive Y 0
    RIGHT,     //Negative Y 1
    UP,        //Positive Z 2
    DOWN,      //Negative Z 3
    FRONT,     //Positive X 4
    BACK       //Negative X 5
};

enum class EE_LOADING{
    LOAD=0,
    UNLOAD
};

