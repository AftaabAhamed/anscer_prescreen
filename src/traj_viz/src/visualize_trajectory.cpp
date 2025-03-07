#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "visualization_msgs/Marker.h"
#include "traj_viz/traj_req.h"
#include "traj_viz/viz_req.h"
#include "tf2_ros/transform_listener.h"
#include <ctime>
#include <fstream>
#include <iostream>


visualization_msgs::MarkerArray marker_array;

bool visualiseTrajectory(traj_viz::viz_req::Request &req , traj_viz::viz_req::Response &res)
{   
  std::string filename = req.filename;  
  std::ifstream file(filename+".csv");
  std::string line;

  int id = 0;
  while (std::getline(file, line))
  {
    
    std::stringstream ss(line);
    std::vector<std::string> row;
    std::string data;
    while (std::getline(ss, data, ','))
    {
      row.push_back(data);
    }
    visualization_msgs::Marker marker;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.header.frame_id = "odom";
    
    marker.id = id; id ++;

    marker.header.stamp = ros::Time();
    marker.ns = "trajectory";
    
    
    marker.pose.position.x = std::stof(row[0]);
    marker.pose.position.y = std::stof(row[1]);
    marker.pose.position.z = std::stof(row[2]);
    marker.pose.orientation.x = std::stof(row[3]);
    marker.pose.orientation.y = std::stof(row[4]);
    marker.pose.orientation.z = std::stof(row[5]);
    marker.pose.orientation.w = std::stof(row[6]);
    marker.scale.x = 0.1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; 
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;

    
    marker_array.markers.push_back(marker);

  }
    ros::NodeHandle n;

  return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "vizualise_trajectory_service");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("visualize_trajectory", visualiseTrajectory);
  ros::Publisher marker_pub = n.advertise<visualization_msgs::MarkerArray>("visualization_marker", 10);
  ROS_INFO("Ready to visualize trajectory");
  while(ros::ok())
  {
    marker_pub.publish(marker_array);
    ros::spinOnce();
  }
  return 0;
}
