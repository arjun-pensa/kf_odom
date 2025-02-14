#include "ros/ros.h"

#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32.h>
#include "kf_odom/helper.h"
#include <math.h>

class accel_stamped {
 public:
	accel_stamped() {}
	accel_stamped(const Eigen::Vector2d &meas,
		          const ros::Time &stamp,
		          const double &dt) {
		meas_ = meas;
		stamp_ = stamp;
		dt_ = dt;
	}
	Eigen::Vector2d get_meas() {return meas_;}
	ros::Time get_time() {return stamp_;}
	double get_dt() {return dt_;}
 private:
	Eigen::Vector2d meas_;
	ros::Time stamp_;
	double dt_; // Time between the previous measurement and the current
};


class velpub {
public:
	accel_stamped a_meas_;
	ros::Time cur_time_;
	ros::NodeHandle *nh_;
	ros::Subscriber pose_sub_, t265_sub_;
	ros::Publisher vel_pub_, yaw_pub_;
	ros::Time init_time_;
	float velx_, vely_;
	std_msgs::Float32 yaw_;
	// List of past measurements
	std::list<accel_stamped> list_a_meas_;
	double max_list_size_;

	velpub (    ros::NodeHandle *nh,
		    const double &max_list_size) {  // List maximum size (in seconds)
		nh_ = nh;
		max_list_size_ = max_list_size;
		init_time_ = ros::Time(0.0);

		// Start subscribers
		std::string pose_topic, vel_topic, yaw_topic, t265_topic;
		nh_->getParam("pose_topic", pose_topic);
		nh_->getParam("velpub_topic", vel_topic);
		nh_->getParam("T265_topic", t265_topic);
		nh_->getParam("yaw_topic", yaw_topic);
		// ROS_INFO("%s ", pose_topic.c_str());
		t265_sub_ = nh_->subscribe(t265_topic, 1, &velpub::T265Callback, this);
		ROS_INFO("[velpub] T265 topic: %s", t265_sub_.getTopic().c_str());
		pose_sub_ = nh_->subscribe(pose_topic, 1, &velpub::PoseCallback, this);
		ROS_INFO("[velpub] Pose topic: %s", pose_sub_.getTopic().c_str());
		vel_pub_ = nh_->advertise<geometry_msgs::Twist>(vel_topic, 10);
		ROS_INFO("[velpub] Velocity topic: %s", vel_pub_.getTopic().c_str());
		yaw_pub_ = nh_->advertise<std_msgs::Float32>(yaw_topic, 10);
		// ROS_INFO("[velpub] Yaw topic: %s", yaw_pub_.getTopic().c_str());
	}

	void AddMeasToList(const accel_stamped &a_new) {
		list_a_meas_.push_front(a_new);
		ros::Duration dt;
		while(true) {
			dt = list_a_meas_.front().get_time() - list_a_meas_.back().get_time();
			if (dt.toSec() > max_list_size_) {
				list_a_meas_.pop_back();
			} else {
				break;
			}
		}
		Eigen::Vector2d disp = list_a_meas_.front().get_meas() - list_a_meas_.back().get_meas();
		dt = list_a_meas_.front().get_time() - list_a_meas_.back().get_time();
		
		if (!dt.isZero()) {
			velx_ = disp(0)/dt.toSec();
			vely_ = disp(0)/dt.toSec();
		}
		geometry_msgs::Twist vel;
		vel.linear.x = velx_;
		vel.linear.y = vely_;
		vel_pub_.publish(vel);
		 
		// ROS_INFO("Time deviation: %f", dt.toSec());
	}
 
 	void PoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg) {
		
		Eigen::Vector2d pose_meas = Eigen::Vector2d(msg->pose.pose.position.x,msg->pose.pose.position.y);
		
		// Initialization (first time this callback runs)
		if (init_time_.toSec() == 0) {
			init_time_ = msg->header.stamp;
			cur_time_ = init_time_;
			const double dt = 0.0;
			a_meas_ = accel_stamped(pose_meas, cur_time_, dt);
			this->AddMeasToList(a_meas_);
			return;
		}

		const double dt = (msg->header.stamp - cur_time_).toSec();
		if (dt > 0) {
			cur_time_ = msg->header.stamp;
			a_meas_ = accel_stamped(pose_meas, cur_time_, dt);
			// this->PropagationUpdate(dt);
			this->AddMeasToList(a_meas_);
		}
	}

	void T265Callback(const nav_msgs::Odometry::ConstPtr &msg) {
		Eigen::Vector2d vel_meas = Eigen::Vector2d(msg->twist.twist.linear.x, msg->twist.twist.linear.y);	

		yaw_.data = std::remainder(atan2(vel_meas(1),vel_meas(0)) - atan2(vely_,velx_),2*M_PI);
		yaw_pub_.publish(yaw_);
	}

};



int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle n("~");

 	  
  double max_list_size = 0.5;  // Half a second as maximum width in the list

  velpub vel_obj(&n, max_list_size);

   
  
  
  ros::spin();

  return 0;
} 