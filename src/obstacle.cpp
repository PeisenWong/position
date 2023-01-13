#include "ros/ros.h"
#include "obstacle_detector/Obstacles.h"
#include <cmath>
#include <vector>

#include <stdio.h>
#include <unistd.h>
#include <sys/socket.h>
#include <bluetooth/bluetooth.h>
#include <bluetooth/rfcomm.h>

#include "position/serialib.h"

// #define SERIALLIB
// #define CONTINUOUS
#define SERIAL_PORT "/dev/ttyUSB1"
serialib serial;

using namespace std;

int received_counts, counts;
float received_x, received_y, received_distance, sent_x, sent_y, sent_distance;
int s;
double distances;
uint8_t receive[500], sending[500], ack = 0x01;
double p_min_r1, p_max_r1, p_min_r2, p_max_r2;
unsigned char instruction, response;

typedef struct
{
    float x;
    float y;
    float distance;
} Pole;

Pole pole;
vector<Pole> PoleList;

enum Instruction
{
    FAR,
    NEAR,
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX
};

enum Response
{
    OK,
    NO
};

// Format Communication
// [0x01][Total No.][x distance][y distance][distance]...
// Total bytes: (Total_No * 3 + 3) * 4

void ObstacleCallback(const obstacle_detector::Obstacles obs)
{
    ros::NodeHandle nh;
    counts = 0;
    if(nh.getParam("/obstacle_extractor/min_circle_radius", p_min_r1) 
    && nh.getParam("/obstacle_extractor/max_circle_radius", p_max_r1) 
    && nh.getParam("/obstacle_extractor/min_circle_radius2", p_min_r2)
    && nh.getParam("/obstacle_extractor/max_circle_radius2", p_max_r2))
    {
        for(const auto& circle : obs.circles)
        {
            if((circle.true_radius >= p_min_r1 && circle.true_radius <= p_max_r1) 
            || (circle.true_radius >= p_min_r2 && circle.true_radius <= p_max_r2))
            {
                pole.x = circle.center.x;
                pole.y = circle.center.y;
                pole.distance = sqrt(pow(circle.center.x, 2) + pow(circle.center.y, 2));
                PoleList.push_back(pole);
                counts++;
                ROS_INFO("Circle %d at X: %lf Y: %lf Distance: %lf", counts, circle.center.x, circle.center.y, sqrt(pow(circle.center.x, 2) + pow(circle.center.y, 2)));
            }
        }
    }
    else
    {
        ROS_INFO("Bruh, no param");
    }

#ifdef CONTINUOUS
    if(counts > 0 && counts <= 10)
    {
        memcpy(&sending[0], &ack, 1);
        memcpy(&sending[1], &counts, 4);

#ifndef SERIALLIB
        do
        {
            send(s, sending, 5, MSG_WAITALL);
        } while (recv(s, receive, 4, MSG_WAITALL) < 0);
#else
        serial.writeBytes(sending, 5);
        serial.readBytes(receive, 4, 100);
#endif
        
        memcpy(&received_counts, &receive, 4);

        if(received_counts != counts)
            ROS_INFO("U fked up, received %d", received_counts);
        else
            ROS_INFO("Received %d", received_counts);

        for(int i = 0; i < PoleList.size(); i++)
        {
            memcpy(&sending[i * 12], &PoleList.at(i).x, 4);
            memcpy(&sending[4 + i * 12], &PoleList.at(i).y, 4);
            memcpy(&sending[8 + i * 12], &PoleList.at(i).distance, 4);
        }
#ifndef SERIALLIB
        do
        {
            send(s, sending, PoleList.size() * 3 * 4, MSG_WAITALL);
        } while (recv(s, receive, PoleList.size() * 3 * 4, MSG_WAITALL) < 0);
#else
        serial.writeBytes(sending, PoleList.size() * 3 * 4);
        serial.readBytes(receive, PoleList.size() * 3 * 4, 100);
#endif
        for(int i = 0; i < received_counts; i++)
        {
            memcpy(&received_x, &receive[i * 12], 4);
            memcpy(&received_y, &receive[i * 12 + 4], 4);
            memcpy(&received_distance, &receive[i * 12 + 8], 4);
            ROS_INFO("Received X: %.2f Y: %.2f Dist: %.2f", received_x, received_y, received_distance);
        }
        ROS_INFO("Done\n");
        PoleList.clear();
    }
#endif
}

int main(int argc, char** argv)
{
#ifndef SERIALLIB
    // To enable ps4 control from mainboard
    struct sockaddr_rc addr = { 0 };
    char dest[18] = "98:D3:31:FD:5D:98";

    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );    

    // connect to server
    while(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    {
        ROS_INFO("Havent Connect");
        connect(s, (struct sockaddr *)&addr, sizeof(addr));
    }

#else
    char errorOpening = serial.openDevice(SERIAL_PORT, 115200);
    if (errorOpening != 1)
    {
        printf("Haven't or cannot connect to COM\n");
        return errorOpening;
    }
#endif
    
    ros::init(argc, argv, "obstacle");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe<obstacle_detector::Obstacles>("/tracked_obstacles", 10, ObstacleCallback);

    ros::AsyncSpinner spinner(3); 
    spinner.start();

    // ros::waitForShutdown();
    while(1)
    {
        // Wait for instruction
        static int state = 0;
        switch(state)
        {
            case 0: 
                recv(s, receive, 2, MSG_WAITALL);
                if(receive[0] = 0x01)
                {
                    memcpy(&instruction, &receive[1], 1);

                    switch(instruction)
                    {
                        case NEAR: // Give the nearest pole data
                            if(PoleList.size())
                            {
                                Pole temp = PoleList.at(0);
                                for(int i = 0; i < PoleList.size(); i++)
                                {
                                    if(fabs(Polelist.at(i).distance) < fabs(temp.distance))
                                        temp = PoleList.at(i);
                                }
                                response = OK;
                                state = 1;
                            }
                            else
                            {
                                response = NO;
                                state = 0;
                            }
                        break;

                        case FAR:
                        // Process pole data
                        break;

                        case ONE:
                        // Process pole data
                        break;

                        case TWO:
                        // Process pole data
                        break;

                        case THREE:
                        // Process pole data
                        break;

                        case FOUR:
                        // Process pole data
                        break;

                        case FIVE:
                        // Process pole data
                        break;

                        case SIX:
                        // Process pole data
                        break;
                    }
                }
        }
    }

#ifndef SERIALLIB
    close(s);
#endif
}