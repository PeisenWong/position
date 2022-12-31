#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include <cmath>
#include <cstdlib>

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

using namespace std;

void VelCallBack(const geometry_msgs::Twist msg);

int s, status;
float x_vel = 0, y_vel = 0, w_vel = 0;
uint8_t ack_vel, vel[12];

int main(int argc, char** argv)
{
    // To receive coordinates from mainboard
    struct sockaddr_rc addr = { 0 };
    char dest[18] = "98:D3:51:FD:E3:FE";

    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );

    // connect to server
    while(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
        connect(s, (struct sockaddr *)&addr, sizeof(addr));

    ros::init(argc, argv, "robot_vel");

    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/nav/cmd_vel", 10, VelCallBack);
    
    ros::spin();
    ros::waitForShutdown();
    close(s);
}

// Configure the controller frequency at move_base.yaml to 20Hz
void VelCallBack(const geometry_msgs::Twist msg)
{
    ROS_INFO("X_Vel: %.2f Y_Vel: %.2f W_Vel: %.2f", msg.linear.x, msg.linear.y, msg.angular.z);
    // Changed to RBC Coordinate
    x_vel = -msg.linear.y;
    y_vel = msg.linear.x;
    w_vel = -msg.angular.z;

    memcpy(&vel[0], &x_vel, 4);
    memcpy(&vel[4], &y_vel, 4);
    memcpy(&vel[8], &w_vel, 4);
    
    do
    {
        send(s, vel, 12, MSG_WAITALL);
        cout << "Sending vel" << endl;
        usleep(1000);
    } while (recv(s, &ack_vel, 1, MSG_WAITALL) <= 0);
}