#include "ros/ros.h"
#include "std_msgs/Float64.h"
#include <iostream>

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

using namespace std;

int s, status;
uint8_t receive[16], ack = 0x01;
float x_mea = 0, y_mea = 0, x_est = 0, y_est = 0;

std_msgs::Float64 X_mea;
std_msgs::Float64 Y_mea;
std_msgs::Float64 X_est;
std_msgs::Float64 Y_est;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "plot");
    ros::NodeHandle n;

    ros::Publisher X_Mea = n.advertise<std_msgs::Float64>("X_Mea", 1000);
    ros::Publisher Y_Mea = n.advertise<std_msgs::Float64>("Y_Mea", 1000);
    ros::Publisher X_Est = n.advertise<std_msgs::Float64>("X_Est", 1000);
    ros::Publisher Y_Est = n.advertise<std_msgs::Float64>("Y_Est", 1000);
    ros::Rate loopRate(1);

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
    status = connect(s, (struct sockaddr *)&addr, sizeof(addr));

    while(ros::ok())
    {
        do
        {
            send(s, &ack, 1, MSG_WAITALL);
            loopRate.sleep();
            cout << "Sent" << endl;
        } while (recv(s, receive, 16, MSG_WAITALL) <= 0);
        
        memcpy(&x_mea, &receive[0], 4);
        memcpy(&y_mea, &receive[4], 4);
        memcpy(&x_est, &receive[8], 4);
        memcpy(&y_est, &receive[12], 4);

        X_mea.data = x_mea;
        Y_mea.data = y_mea;
        X_est.data = x_est;
        Y_est.data = y_est;

        X_Mea.publish(X_mea);
        Y_Mea.publish(Y_mea);
        X_Est.publish(X_est);
        Y_Est.publish(Y_est);

        ros::spinOnce();
    }

    return 0;
}