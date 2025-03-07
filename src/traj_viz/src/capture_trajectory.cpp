#include "ros/ros.h"
#include "std_srvs/Empty.h"
#include "geometry_msgs/PoseStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "traj_viz/traj_req.h"
#include "traj_viz/viz_req.h"
#include "tf2_ros/transform_listener.h"
#include <ctime>
#include <fstream>
#include <iostream>
// #include "std_msgs/String.h"
// Service callback function

// geometry_msgs::PoseStamped pose;


bool captureTrajectory(traj_viz::traj_req::Request &req , traj_viz::traj_req::Response &res)
{
    ROS_INFO("Trajectory capture service called");

    auto filename  = req.filename;
    auto duration = req.duration;
    // auto duration = req.duration;
    // res.str = "saved: " + filename;

    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    geometry_msgs::TransformStamped transformStamped;

    ros::Rate rate(10.0);

    time_t now = time(NULL);

    std::ofstream file;
    file.open(filename+".csv");
    // file << "x,y,z,rx,ry,rz,rw\n";

    while(difftime(time(NULL), now) < duration)
    {   
        ROS_INFO("recording for : %d", (int)time(NULL));
        try {
            transformStamped = tfBuffer.lookupTransform("odom", "base_link", ros::Time(0));
            
            file 
                << transformStamped.transform.translation.x
                << "," << transformStamped.transform.translation.y 
                << "," << transformStamped.transform.translation.z 
                << "," << transformStamped.transform.rotation.x 
                << "," << transformStamped.transform.rotation.y 
                << "," << transformStamped.transform.rotation.z 
                << "," << transformStamped.transform.rotation.w
                << "\n";
            
        } catch (tf2::TransformException &ex) {
            ROS_WARN("%s",ex.what());
            ros::Duration(1.0).sleep();
            continue;
        }
        rate.sleep();
        
        // file << pose.pose.position.x << "," << pose.pose.position.y << "," << pose.pose.position.z << "," << pose.pose.orientation.x << "," << pose.pose.orientation.y << "," << pose.pose.orientation.z << "\n";
    }
    file.close();


    ROS_INFO("recording complete after %d seconds", duration);

    // Add your trajectory capture logic here

    return true;
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "capture_trajectory_service");
    ros::NodeHandle n;


    ros::ServiceServer service = n.advertiseService("capture_trajectory", captureTrajectory);
    ROS_INFO("Ready to capture trajectory.");

    ros::spin();

    return 0;

}

