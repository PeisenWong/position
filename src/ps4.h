#include "ros/ros.h"
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Imu.h"
#include "tf/transform_broadcaster.h"
#include <stdlib.h>

class PS4
{
    public:
    PS4();
    void ps4Callback(const sensor_msgs::Joy::ConstPtr msg);
    void imuCallback(const sensor_msgs::Imu::ConstPtr msg);
    void checkPS4(void);
    void calibrateL2R2(void);
    void publishNavi(void);

    private:
    
    // PS4
    float left_y, left_x, right_y, right_x, an_l2, an_r2;
    int cross, triangle, circle, square, option, select, x_button, y_button, l1, r1, dg_l2, dg_r2;
    int touchpad, play, leftjoy_button, rightjoy_button;

    // Navi
    // Follow ROS coordinate
    geometry_msgs::Twist vel;
    float x_vel, y_vel, w_vel, yaw;
    float move_speed = 2.0, turn_speed = 1.0;

    // ROS
    ros::Subscriber ps4_receiver, imu_receiver;
    ros::Publisher navi;
    ros::NodeHandle nh;
};