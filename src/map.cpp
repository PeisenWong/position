#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "tf/transform_broadcaster.h"

int main( int argc, char** argv )
{
  ros::init(argc, argv, "newMap");
  ros::NodeHandle n;
  ros::Rate r(1);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 1);

  // Parameters
  double p_x, p_y, p_z, p_yaw;

  n.getParam("/map/X", p_x);
  n.getParam("/map/Y", p_y);
  n.getParam("/map/Z", p_z);
  n.getParam("/map/Yaw", p_yaw);

  while (ros::ok())
  {
    visualization_msgs::Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = ros::Time::now();

    marker.ns = "GameField";
    marker.id = 0;

    marker.type = visualization_msgs::Marker::MESH_RESOURCE;

    marker.pose.position.x = p_x;
    marker.pose.position.y = p_y;
    marker.pose.position.z = p_z;
    marker.pose.orientation = tf::createQuaternionMsgFromYaw(p_yaw);

    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;

    marker.color.r = 0;
    marker.color.g = 0;
    marker.color.b = 0;
    marker.color.a = 0;
    marker.mesh_resource = "package://position/gamefield2023.dae";
    marker.mesh_use_embedded_materials = 1;
    marker.lifetime = ros::Duration();



    while (marker_pub.getNumSubscribers() < 1)
    {
      if (!ros::ok())
      {
        return 0;
      }
      ROS_WARN_ONCE("Please create a subscriber to the marker");
      sleep(1);
    }
    marker_pub.publish(marker);
    
    r.sleep();
  }

  return 0;
}
