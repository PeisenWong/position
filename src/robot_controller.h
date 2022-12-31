#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include <cmath>
#include <cstdlib>

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

// #include <move_base_msgs/MoveBaseAction.h>
// #include <actionlib/client/simple_action_client.h>
// #include "actionlib_msgs/GoalID.h"

class Controller
{
public:
    Controller();

    // uint8_t receive[2], instruction, data;
};

enum
{
    NAV_TEST,
    NAV_SEND_GOAL,
    NAV_STOP,
    NAV_CANCEL_GOAL,
    NAV_TUNE,
    NAV_INFO
};