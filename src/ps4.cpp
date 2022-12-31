#include "ps4.h"

using namespace std;

PS4::PS4()
{
    ps4_receiver = nh.subscribe("joy", 1000, &PS4::ps4Callback, this);
    imu_receiver = nh.subscribe("imu", 1000, &PS4::imuCallback, this);
    navi = nh.advertise<geometry_msgs::Twist>("ps4/cmd_vel", 1000);
}

void PS4::imuCallback(const sensor_msgs::Imu::ConstPtr msg)
{
    yaw = tf::getYaw(msg->orientation);
}

void PS4::ps4Callback(const sensor_msgs::Joy::ConstPtr msg)
{
    square = msg->buttons[0];
    cross = msg->buttons[1];
    circle = msg->buttons[2];
    triangle = msg->buttons[3];
    l1 = msg->buttons[4];
    r1 = msg->buttons[5];
    dg_l2 = msg->buttons[6]; // 1 when pressed
    dg_r2 = msg->buttons[7];
    select = msg->buttons[8];
    option = msg->buttons[9];
    leftjoy_button = msg->buttons[10];
    rightjoy_button = msg->buttons[11];
    play = msg->buttons[12];
    touchpad = msg->buttons[13];

    // Assume leftjoy for navi, following ROS coordinate
    left_y = msg->axes[0]; // Right is -1
    left_x = msg->axes[1];
    right_x = msg->axes[2]; // Right is -1
    right_y = msg->axes[5];
    an_l2 = msg->axes[3]; // from 1 to -1
    an_r2 = msg->axes[4]; // From 1 to -1
    x_button = (int)msg->axes[9]; // Left is 1, right is -1
    y_button = (int)msg->axes[10]; // Up is 1, down is -1
    
    calibrateL2R2();
}

void PS4::calibrateL2R2()
{
    an_l2--;
    an_l2 /= 2;
    an_l2 = abs(an_l2);

    an_r2--;
    an_r2 /= 2;
    an_r2 = abs(an_r2);
}

void PS4::checkPS4()
{
    if(square == 1)
    {
        while(square == 1);
        ROS_INFO("Square is pressed");
    }

    if(l1 == 1)
    {
        while(l1 == 1);
        move_speed++;
    }

    if(r1 == 1)
    {
        while(r1 == 1);
        move_speed--;
    }
}

void PS4::publishNavi()
{
    x_vel = left_x * move_speed; // Front n back
    y_vel = left_y * move_speed; // Left n right
    w_vel = (an_l2 - an_r2) * turn_speed;

    vel.linear.x = x_vel;
    vel.linear.y = y_vel;
    vel.angular.z = w_vel;

    navi.publish(vel);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "PS4_Control");

    PS4 ps4;

    ros::AsyncSpinner spinner(4);
    spinner.start();

    ros::Rate loopRate(20);

    while(ros::ok())
    {
        ps4.checkPS4();
        ps4.publishNavi();

        loopRate.sleep();
    }

    ros::waitForShutdown();
    return 0;
}