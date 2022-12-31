#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "sensor_msgs/Imu.h"
#include <cmath>
#include <cstdlib>

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

// #define newMap
using namespace std;

void process();
void sendTF();
void sendOdom();

// Need receive x, y, yaw, Vx, Vy, w
uint8_t receive[24], sends[4], vel[12];
float XEnc = 0, YEnc = 0, yaw = 0, XVel, YVel, WVel;
int s, status;
uint8_t ack = 0x01;

int main(int argc, char **argv)
{
    // To receive coordinates from mainboard
    struct sockaddr_rc addr = { 0 };
    char dest[18] = "98:DA:60:01:F0:E7";

    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );

    // connect to server
    while(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
        connect(s, (struct sockaddr *)&addr, sizeof(addr));

    ros::init(argc, argv, "robot");
    
    ros::NodeHandle n, nh;
    ros::Publisher odom_pub = nh.advertise<nav_msgs::Odometry>("odom", 1000);
    ros::Publisher imu_pub = nh.advertise<sensor_msgs::Imu>("/imu/data", 1000);

    tf::TransformBroadcaster broadcaster;
    geometry_msgs::TransformStamped coor;
    nav_msgs::Odometry odom;
    sensor_msgs::Imu imu;

    ros::Rate loop_rate(20);

    #ifndef newMap 
    coor.header.frame_id = "odom";
    coor.child_frame_id = "base_link";
    #else
    coor.header.frame_id = "GameFieldFrame";
    coor.child_frame_id = "base_footprint";
    #endif

    // Configure to 20Hz
    while(ros::ok())
    {
        // process();
        do
        {
            send(s, &ack, 1, MSG_WAITALL);
            std::cout << "Sending" << std::endl;
        }while(recv(s, receive, 24, MSG_WAITALL) <= 0);

        memcpy(&XEnc, &receive[0], 4);
        memcpy(&YEnc, &receive[4], 4);
        memcpy(&XVel, &receive[8], 4);
        memcpy(&YVel, &receive[12], 4);
        memcpy(&yaw, &receive[16], 4);
        memcpy(&WVel, &receive[20], 4);
        printf("X: %2f Y: %.2f Yaw: %.2f XVel: %.2f YVel: %.2f WVel: %.2f\n", XEnc, YEnc, yaw, XVel, YVel, WVel);

        // sendTF();
        coor.header.stamp = ros::Time::now();
        #ifndef newMap
        coor.transform.translation.x = YEnc;
        coor.transform.translation.y = XEnc * -1;
        #else
        coor.transform.translation.x = XEnc;
        coor.transform.translation.y = YEnc;
        #endif
        coor.transform.translation.z = 0;
        coor.transform.rotation = tf::createQuaternionMsgFromYaw(yaw * -1);

        broadcaster.sendTransform(coor);

        // Send odometry
        odom.header.stamp = ros::Time::now();
        odom.header.frame_id = "base_link";
        odom.pose.pose.position.x = YEnc;
        odom.pose.pose.position.y = -XEnc;
        odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(-yaw);

        odom.twist.twist.linear.x = YVel;
        odom.twist.twist.linear.y = -XVel;
        odom.twist.twist.linear.z = 0;
        odom.twist.twist.angular.z = -WVel;

        odom_pub.publish(odom);

        // Publish IMU data
        imu.header.frame_id = "base_link";
        imu.orientation = tf::createQuaternionMsgFromYaw(-yaw);
        imu.angular_velocity.z = -WVel;

        imu_pub.publish(imu);

        loop_rate.sleep();    
    }
    
    close(s);
    return 0;
}

void process()
{
    // Communication
    /*
    1. Send ack to mainboard to notify ready to receive
    2. Mainboard send the data
    3. Send ack again after update the position
    */

    // do
    // {
    //     send(s, &ack, 1, MSG_WAITALL);
    //     std::cout << "Sending" << std::endl;
    //     usleep(1000);
    // }while(recv(s, receive, 12, MSG_WAITALL) <= 0);

    // memcpy(&XEnc, &receive[0], 4);
    // memcpy(&YEnc, &receive[4], 4);
    // memcpy(&yaw, &receive[8], 4);
    // std::cout << "Received X: " << XEnc << " Y: " << YEnc << " Yaw: " << yaw << std::endl;

    // sendTF();
}

void sendTF()
{
        // coor.header.stamp = ros::Time::now();
        // #ifndef newMap
        // coor.transform.translation.x = YEnc;
        // coor.transform.translation.y = XEnc * -1;
        // #else
        // coor.transform.translation.x = XEnc;
        // coor.transform.translation.y = YEnc;
        // #endif
        // coor.transform.translation.z = 0;
        // coor.transform.rotation = tf::createQuaternionMsgFromYaw(yaw * -1);

        // broadcaster.sendTransform(coor);
}

void sendOdom()
{
    // Maybe send twist using ekf
    // odom.pose.pose.position.x = YEnc;
    // odom.pose.pose.position.y = -XEnc;
    // odom.pose.pose.position.z = 0;
    // odom.pose.pose.orientation = tf::createQuaternionMsgFromYaw(yaw * -1);

    // odom.twist.twist.linear.x = linear_y;
    // odom.twist.twist.linear.y = linear_x;
    // odom.twist.twist.angular.z = orientation_w;
}
