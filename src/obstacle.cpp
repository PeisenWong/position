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
#define SERIAL_PORT "/dev/ttyUSB2"
serialib serial;

using namespace std;

int received_counts, counts;
double received_x, received_y, received_distance, sent_x, sent_y, sent_distance, zero = 0;
int s, i = 0;
double distances;
uint8_t receive[500], sending[500], ack = 0x01;
double p_min_r1, p_max_r1, p_min_r2, p_max_r2;
uint8_t inst, res, sent_res, received_res;

typedef struct
{
    double x;
    double y;
    double distance;
} Pole;

Pole pole, temp;
vector<Pole> PoleList;

typedef enum
{
    FAR,
    NEAR,
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX
}Instruction_t;

typedef enum
{
    OK,
    NO
}Response_t;

Instruction_t instruction;
Response_t response, received_response, sent_response;

// Format Communication
// [0x01][Total No.][x distance][y distance][distance]...
// Total bytes: (Total_No * 3 + 3) * 4

void ObstacleCallback(const obstacle_detector::Obstacles obs)
{
    ros::NodeHandle nh;
    counts = 0;
    PoleList.clear();
    if(nh.getParam("/obstacle_extractor/min_circle_radius", p_min_r1) 
    && nh.getParam("/obstacle_extractor/max_circle_radius", p_max_r1) 
    && nh.getParam("/obstacle_extractor/min_circle_radius2", p_min_r2)
    && nh.getParam("/obstacle_extractor/max_circle_radius2", p_max_r2))
    {
        for(const auto& circle : obs.circles)
        {
            if((circle.true_radius >= p_min_r1 && circle.true_radius <= p_max_r1)||(circle.true_radius >= p_min_r2 && circle.true_radius <= p_max_r2))
            {
                pole.x = circle.center.x;
                pole.y = circle.center.y;
                auto temp = pole.x;
                pole.x = pole.y*-1;
                pole.y = temp ;
                
                pole.distance = sqrt(pow(circle.center.x, 2) + pow(circle.center.y, 2));
                PoleList.push_back(pole);
                counts++;
                ROS_INFO("Circle %d at X: %.2lf Y: %.2lf D: %.2lf R: %.4lf", counts, circle.center.x, circle.center.y, sqrt(pow(circle.center.x, 2) + pow(circle.center.y, 2)), circle.true_radius);
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
    char dest[18] = "98:DA:60:01:F0:E7";

    // allocate a socket
    s = socket(AF_BLUETOOTH, SOCK_STREAM, BTPROTO_RFCOMM);

    // set the connection parameters (who to connect to)
    addr.rc_family = AF_BLUETOOTH;
    addr.rc_channel = (uint8_t) 1;
    str2ba( dest, &addr.rc_bdaddr );    

    // connect to server
   // while(connect(s, (struct sockaddr *)&addr, sizeof(addr)) != 0)
    //{
    //    ROS_INFO("Havent Connect");
   //     connect(s, (struct sockaddr *)&addr, sizeof(addr));
    //W}

    // Check for boot up
    sending[0] = 0x01;
    response = OK;
    res = response;
    sending[1] = res;

    // do
    // {
    //     send(s, sending, 2, MSG_WAITALL);
    //     ROS_INFO("Booting");
    // }
    // while(recv(s, receive, 1, MSG_WAITALL) < 0);

#else
    char errorOpening = serial.openDevice(SERIAL_PORT, 115200);
    if (errorOpening != 1)
    {
        printf("Haven't or cannot connect to COM\n");
        return errorOpening;
    }

    // Check for boot up
    sending[0] = 0x01;
    response = OK;
    res = response;
    sending[1] = res;

    serial.writeBytes(sending, 2);
    ROS_INFO("Booting");

    while(i < 1)
    {
        if(serial.readChar(receive + i, 0) > 0)
            i++;
        else  
            ROS_INFO("Bruh, Boot fail");
    }

    ROS_INFO("Boot Done");
    i = 0;

#endif
    
    ros::init(argc, argv, "obstacle");
    ros::NodeHandle n;

    // Wait for the node to synchronize with the system clock
    while (!ros::Time::isSystemTime())
    {
        ROS_INFO("Waiting for system time synchronization...");
        ros::Duration(0.5).sleep();
    }

    // Node is now time synchronized
    ROS_INFO("Node is time synchronized!");
    
    ros::Subscriber sub = n.subscribe<obstacle_detector::Obstacles>("/tracked_obstacles", 10, ObstacleCallback);

    ros::AsyncSpinner spinner(2); 
    spinner.start();

    while(ros::ok())
    {
        // Wait for instruction
#ifndef SERIALLIB
        recv(s, receive, 2, MSG_WAITALL);
#else
        // serial.readBytes(receive, 2, 0, 0);
        while(i < 2)
        {
            if(serial.readChar(receive + i, 0) > 0)
                i++;
        }
        ROS_INFO("OK");
        i = 0;
#endif

        if(receive[0] == 0x01)
        {
            memcpy(&instruction, &receive[1], 1);
            switch(instruction)
            {
                case FAR: // Give the far pole data
                    ROS_INFO("Far");
                    if(PoleList.size())
                    {
                        temp = PoleList.at(0);
                        for(int i = 0; i < PoleList.size(); i++)
                        {
                            if(fabs(PoleList.at(i).distance) > fabs(temp.distance))
                                temp = PoleList.at(i);
                        }
                        response = OK;
                    }
                    else
                    {
                        response = NO;
                    }
                break;

                case NEAR:
                ROS_INFO("Near");
                    if(PoleList.size())
                    {
                        temp = PoleList.at(0);
                        for(int i = 0; i < PoleList.size(); i++)
                        {
                            if(fabs(PoleList.at(i).distance) < fabs(temp.distance))
                                temp = PoleList.at(i);
                        }
                        response = OK;
                    }
                    else
                    {
                        response = NO;
                    }
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

                default:
                    ROS_INFO("Bruh %d", instruction);
                break;
            }

            if(response == OK)
            {
                res = response;
                memcpy(&sending[0], &res, 1);
                memcpy(&sending[1], &temp.x, 8);
                memcpy(&sending[9], &temp.y, 8);
                memcpy(&sending[17], &temp.distance, 8);
            }
            else
            {
                res = response;
                memcpy(&sending[0], &res, 1);
                memcpy(&sending[1], &zero, 8);
                memcpy(&sending[9], &zero, 8);
                memcpy(&sending[17], &zero, 8);
            }

            memcpy(&sent_res, &sending[0], 1);
            memcpy(&sent_x, &sending[1], 8);
            memcpy(&sent_y, &sending[9], 8);
            memcpy(&sent_distance, &sending[17], 8);
            
            ROS_INFO("Sending: Response: %d, X: %.2lf, Y: %.2lf, Distance: %.2lf", sent_res, sent_x, sent_y, sent_distance);

#ifndef SERIALLIB
            do
            {
                send(s, sending, 25, MSG_WAITALL);
            }
            while(recv(s, receive, 25, MSG_WAITALL) < 0);
#else
            serial.writeBytes(sending, 25);
            ROS_INFO("Sending");
            
            while(i < 25)
            {
                if(serial.readChar(receive + i, 0) > 0)
                    i++;
            }     
            i = 0;  
#endif

            memcpy(&received_res, &receive[0], 1);
            memcpy(&received_x, &receive[1], 8);
            memcpy(&received_y, &receive[9], 8);
            memcpy(&received_distance, &receive[17], 8);
            ROS_INFO("Received Response: %d X: %.2lf Y: %.2lf Dist: %.2lf", received_res, received_x, received_y, received_distance);
        
            ROS_INFO("Done\n");
        }
    }

#ifndef SERIALLIB
    close(s);
#else
    serial.closeDevice();
#endif
}

// Testing for pi