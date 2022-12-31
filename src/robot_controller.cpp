// #include "robot_controller.h"

int s, status;
uint8_t receive[16], instruction, data;
float target_x = 1.0, target_y = 1.0, target_yaw = 90.0;

actionlib_msgs::GoalID id;
move_base_msgs::MoveBaseGoal goal;

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv)
{
    // To enable ps4 control from mainboard
    struct sockaddr_rc addr = { 0 };
    char dest[18] = "98:DA:60:02:CD:A6";

    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );

    // connect to server
    while(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
        connect(s, (struct sockaddr *)&addr, sizeof(addr));

    ros::init(argc, argv, "Robot_Controller");
    ros::NodeHandle nh;
    ros::Publisher cancel_plan = nh.advertise<actionlib_msgs::GoalID>("move_base/cancel", 1000);
    ros::Rate loopRate(20);

    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Done");

    while(ros::ok())
    {
        if(recv(s, receive, 16, MSG_WAITALL) > 0)
        {
            memcpy(&instruction, &receive[0], 4);

            if(instruction == NAV_SEND_GOAL)
            {
                memcpy(&target_x, &receive[4], 4);
                memcpy(&target_y, &receive[8], 4);
                memcpy(&target_yaw, &receive[12], 4);

                goal.target_pose.header.frame_id = "base_link";
                goal.target_pose.header.stamp = ros::Time::now();

                goal.target_pose.pose.position.x = target_y;
                goal.target_pose.pose.position.y = -target_x;
                goal.target_pose.pose.orientation = tf::createQuaternionMsgFromYaw(target_yaw * -3.142 / 180.0);

                ROS_INFO("Sending goal, X: %.2f, Y: %.2f, Yaw: %.2f", target_x, target_y, target_yaw);
                ac.sendGoal(goal);

                ac.waitForResult();

                if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("Hooray, the base moved 1 meter forward");
                else
                    ROS_INFO("The base failed to move forward 1 meter for some reason");
            }

            if(instruction == NAV_CANCEL_GOAL)
            {
                id.stamp = ros::Time::now();
                id.id = "";
                cancel_plan.publish(id);

                ROS_INFO("Cancelled goal");
            }
        }

        loopRate.sleep();
    }

    close(s);

    return 0;
}