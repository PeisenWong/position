# Lidar Application to detect pole

Tested on laptop Ubuntu 2004 and on raspberry pi 4 model B.

If you want to customize the code, its better to test locally before transfer your code to Raspberry pi

## Usage
Communication between laptop/Raspberry pi and mainboard to get the distance of poles

There are 2 mode tested: **Continuous**(Get the distance of all pole detected continuously in array form) and **Discontinuous** mode(Only get certain distance of pole based on
instruction such as near and far). 

Change the macro at obstacle.cpp and LidarInit() to define the mode you want.

I have also developed serialLib at obstacle.cpp which communicate through tty cable, but I tested bluetooth is the most stable communication method.

![image](https://user-images.githubusercontent.com/87217044/223488289-dc97152a-605a-4de6-817c-8a882b8ba1b0.png)

I recommend using discontinuous mode because sending data continuously(Interrupt) will affect the encoder value and other stuff.
You can also customize the code based on your instruction from mainboard.
If use discontinuous mode, just declare the specific uart as highest priority.

## Working with PC and Pi
OS in Pi: Ubuntu 2004, Username: ubuntu, Password: utmrobocon

Make sure your pc and pi are in same local network, either UTMWiFi or P10MakmalFMS

Pi will connect to UTMWiFi once boot up, but you may need to check the ip for the first time using monitor, since I didn't download any desktop app in pi, all command should be done in 
command line. Open a new session with Ctrl-Alt-F3. (Or F4, F5... up to you).

Use `ifconfig` to get the ip

Back to your pc, use `ssh` to connect to pi
```
# Format: ssh {username}@{Ip of pi} , then it will ask for password: utmrobocon
ssh ubuntu@{ip}
```

Use `w3m` to login your userid and password if you are using UTMWiFi to get internet access at pi

You should able to work with PC and Pi without monitor using `ssh`.

In case you want to connect using your own hotspot
```
# Check list of wifi
nmcli dev wifi list

# Connect wifi
nmcli dev wifi connect {Hotspot_Name} password "{Your Password}"
```

To verify internet access: `ping google.com`

To verify both machine in same local network: `ping {other_machine_ip}`

## Install
This app is develop under Ubuntu 20.04 with ROS Noetic. [ROS Noetic Installation](http://wiki.ros.org/noetic/Installation/Ubuntu)
```
# I assume you don't have your own ROS workspace
mkdir -p ros1_ws/src && cd ros1_ws/src

git clone https://github.com/PeisenWong/position.git
git clone https://github.com/PeisenWong/obstacle_detector.git
git clone https://github.com/PeisenWong/rplidar_ros.git
git clone https://github.com/PeisenWong/robot_upstart.git

# Remember to check out different branch if you are testing on Raspberry pi
# If testing locally, just use main branch
cd position
git checkout pi

cd ~/ros1_ws 
catkin_make
source ~/ros1_ws/devel/setup.bash

# If running on pc
roslaunch position obstacle_detect.launch

#If running on pi
roslaunch obstacle_detector obstacle_detect.launch
```

## Visualization
You should able to visualize the pole if you are running locally

If running on pi, you also able to visualize the data using local network but you need to do some configuration setups

Make sure both system is using g++ version 10. (I have already downloaded for pi)

Check using command `g++ --version`

If your g++ version is below 10:
```
sudo apt update -y
sudo apt upgrade -y
sudo apt install -y build-essential
sudo apt install -y gcc-10 g++-10 cpp-10
sudo update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-10 100 --slave /usr/bin/g++ g++ /usr/bin/g++-10 --slave /usr/bin/gcov gcov /usr/bin/gcov-10
```

Get the IP address of pi and pc using `ifconfig`

![image](https://user-images.githubusercontent.com/87217044/223455701-5cf14836-66b4-49aa-a380-07083ed70281.png)

In laptop ~/.bashrc
```
export ROS_MASTER_URI=http://{pc ip here}:11311
export ROS_HOSTNAME={pc ip here}
```

In pi ~/.bashrc
```
export ROS_MASTER_URI=http://{pc ip here}:11311
export ROS_HOSTNAME={pi ip here}
```
If you are just working on single machine, just put `localhost` at the IP section

Remember to `source ~/.bashrc` after you made any modification in bashrc file. You should be ready for visualization.

Run `roscore` at pc

Run `roslaunch obstacle_detector obstacle_detect.launch` at pi

Run `roslaunch position pc_obstacle_detect.launch` at pc

### Debugging
Run `rostopic list` to check list of topic, you should see quite many topics

Run ` rostopic echo {topic_name}` to check the message in the topic

Run `rqt_graph` to check connection between nodes and topic

Sometime the visualization at pc is quite faulty and lagging, just close with Ctrl-C and rerun `roslaunch position pc_obstacle_detect.launch` at pc

## Tuning 
There are quite many parameters in obstacle detector, you can check the README of the package at [here](https://github.com/tysik/obstacle_detector)

For better tuning guide, you can tune using the QT page in the rviz when you launched visualization. he tuning is real time.
![image](https://user-images.githubusercontent.com/87217044/223476714-0f8cf101-b501-4a3a-9f05-f22716f6b7b0.png)

Xmin, Xmax, Ymin, Ymax is the range of lidar, you need to tune these to ignore people legs

rmin, rmax is the radius of pole, anything radius between these 2 parameters will be sensed as pole (include human leg)

rmin2, rmax2: same things as above, but designed for biggest pole

You can check the raw data from `rostopic echo /tracked_obstacle` to get the **true radius**

Change the address of bluebee at obstacle.cpp, current address is UTM Cargo 4

![image](https://user-images.githubusercontent.com/87217044/223478909-8cd86148-e012-4b56-a28b-b23aea4a20c1.png)

## Data Usage
Note that ROS Coordinate is different with RBC Coordinate

![image](https://user-images.githubusercontent.com/87217044/223485773-87ff0e39-54df-4d55-9cbb-fc54cda13067.png)

In ROS, red axis is x, green is y. However, the distance is quite accurate. 


## Running roslaunch at background
The concept is based on linux service that will bring up at during booting.

If you want to try locally 
```
# Customize your own directory
rosrun robot_upstart install obstacle_detector/launch/obstacle_detect.launch --job lidar3 --user root --setup /home/ubuntu/ros1_ws/devel/setup.bash --symlink --systemd-after hciuart.service --master http://{PC IP}"11311
```
This will create a service in your device that will run everytime you boot up your pc, which is already done in pi.

You can check the info about service in ubuntu with `systemctl`
```
# In this case, the service name is the argument at --job
sudo systemctl status lidar3
sudo systemctl restart lidars
sudo systemctl enable lidar3
sudo systemctl disable lidars
```

You can check logging of service using `journalctl` or `rostopic echo /rosout`. For you to confirm data validity.
``` 
sudo journalctl lidar3
```

For data visualization, remember to change the ROS_HOSTNAME at robot_upstart/templates/job-start.em with your current Pi IP. 

## Roadmap
Currently this app can only accept NEAR (nearest pole) and FAR (farest pole) in discontinuous mode. 

I should able to instruct which pole to shoot, maybe based on sequence, but some pole may be blocked if I am aiming a pole( in case the lidar not 360 degree)

Haven't tested on real robot, just can receive data successfully
