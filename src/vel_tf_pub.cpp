#include "ros/ros.h"

#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/convert.h>
#include "tf2/transform_datatypes.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float32.h>
#include "kf_odom/helper.h"
#include <math.h>

ros::Subscriber T265_sub;
ros::Publisher vel_pub;


void velCallback(const nav_msgs::Odometry::ConstPtr &msg){
	tf2::Quaternion rot;
	tf2::convert(msg->pose.pose.orientation,rot);
	tf2::Vector3 in_vel;
	tf2::convert(msg->twist.twist.linear,in_vel);
	tf2::Quaternion tf_quat = rot*in_vel*tf2::inverse(rot);
	ROS_INFO_STREAM(tf_quat.getAxis().getX());
	geometry_msgs::Quaternion out_quat= tf2::toMsg(tf_quat);
	geometry_msgs::Vector3 out_vel;
	out_vel.x = out_quat.x;
	out_vel.y = out_quat.y;
	out_vel.z = out_quat.z;
	vel_pub.publish(out_vel);
}

int main(int argc, char **argv){
    ros::init(argc, argv, "vel_transformer");
    ros::NodeHandle n;  
	
    T265_sub = n.subscribe("/T265/odom/sample", 1000, velCallback);
	vel_pub = n.advertise<geometry_msgs::Vector3>("transformVel", 1000);
  
  
    ros::spin();

    return 0;
}