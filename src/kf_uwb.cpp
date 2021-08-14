// ROS
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>
#include <kf_odom/DeviceRangeMsg.h>
#include <Eigen/Dense>
#include <math.h>
#include "kf_odom/helper.h"

class vel_stamped {
 public:
	vel_stamped() {}
	vel_stamped(const Eigen::Vector3d &meas,
		          const ros::Time &stamp,
		          const double &dt) {
		meas_ = meas;
		stamp_ = stamp;
		dt_ = dt;
	}

	Eigen::Vector3d get_meas() {return meas_;}
	ros::Time get_time() {return stamp_;}
	double get_dt() {return dt_;}
 private:
	Eigen::Vector3d meas_;
	ros::Time stamp_;
	double dt_; // Time between the previous measurement and the current
};

class KF {
 public:
	//Eigen::MatrixXd Hk_range_;
	Eigen::MatrixXd Qk_;
	// Eigen::MatrixXd Rk_;
	Eigen::MatrixXd R_pv_;
	Eigen::MatrixXd R_v_;
	Eigen::MatrixXd R_b_;
	Eigen::VectorXd X_;
	Eigen::MatrixXd Pk1_k1_, Pk_k1_, Pk_k_;
	vel_stamped v_meas_;
	ros::Time cur_time_;
	ros::NodeHandle *nh_;
	ros::Subscriber vel_sub_, range_sub_;
	ros::Time init_time_;

	// List of past measurements
	std::list<vel_stamped> list_v_meas_;
	double max_list_size_;

	KF (    ros::NodeHandle *nh,
		    const Eigen::Vector3d &sigma_nu_a,  // Convariance of accelerometer measurements
			const double &sigma_nu_b,  // Convariance of bias process noise
			const double &sigma_eta_b, // Covariance of bias measurement
			const double &sigma_eta_v, // Convariance of velocity measurement
			const Eigen::VectorXd &X0,  // Initial state
			const Eigen::MatrixXd &P0,  // Initial covariance
			const double &max_list_size) {  // List maximum size (in seconds)
		nh_ = nh;
		// Hk_range_ = Eigen::MatrixXd::Zero(2,8);
		Qk_ = Eigen::MatrixXd::Zero(5,5);
		// Rk_ = Eigen::MatrixXd(8,8);
		R_pv_ = Eigen::MatrixXd::Zero(6,6);
		R_b_ = Eigen::MatrixXd::Zero(2,2);
		Pk1_k1_ = P0;
		Pk_k1_ = P0;
		Pk_k_ = P0;
		X_ = X0;
		max_list_size_ = max_list_size;
		init_time_ = ros::Time(0.0);

		// Start subscribers
		std::string vel_topic, range_topic;
		nh_->getParam("vel_topic", vel_topic);
		//nh_->getParam("imu_topic",  imu_topic);
		nh_->getParam("range_topic", range_topic);
		ROS_INFO("%s %s %s", vel_topic.c_str(), range_topic.c_str());
		vel_sub_  = nh_->subscribe(vel_topic,  1, &KF::VelCallback,  this);
		range_sub_ = nh_->subscribe(range_topic, 1, &KF::RangeCallback, this);
		ROS_INFO("[kf_odom] Vel topic: %s", vel_sub_.getTopic().c_str());
		ROS_INFO("[kf_odom] Range topic: %s", range_sub_.getTopic().c_str());

		// Measurement matrix
		// Hk_range_.block(0,6,2,2) = Eigen::MatrixXd::Identity(2,2);
		// std::cout << "Hk_range_:\n" << Hk_range_ << std::endl << std::endl;

		// Initialized process noise covariances
		Qk_(0,0) = sigma_nu_a(0);
		Qk_(1,1) = sigma_nu_a(1);
		Qk_(2,2) = sigma_nu_a(2);
		Qk_.block(3,3,2,2) = sigma_nu_b*sigma_nu_b*Eigen::MatrixXd::Identity(2,2);
		std::cout << "Qk_:\n" << Qk_ << std::endl << std::endl;

		// Initialize constant covariance matrices
		R_b_ = sigma_eta_b*sigma_eta_b*Eigen::MatrixXd::Identity(2,2);
		R_v_ = sigma_eta_v*sigma_eta_v*Eigen::MatrixXd::Identity(3,3);
		R_pv_.block(3,3,3,3) = R_v_;
		std::cout << "R_b_:\n" << R_b_ << std::endl << std::endl;
		std::cout << "R_v_:\n" << R_v_ << std::endl << std::endl;
	}

	uint32_t pred_range(const Eigen::Vector3d &anchor)
	{
		Eigen::Vector3d dr = anchor - X_;
		return sqrt(pow(dr(0),2)+pow(dr(1),2)+pow(dr(2),2));
	}

	Eigen::MatrixXd GetHk(const Eigen::Vector3d &anchor)
	{
		Eigen::Vector3d dr = anchor - X_;
		uint32_t dist = pred_range(anchor);
		Eigen::MatrixXd H;
		H << dr(0)/dist, dr(1)/dist, dr(2)/dist;

	}

	void AddMeasToList(const vel_stamped &v_new) {
		list_v_meas_.push_front(v_new);
		ros::Duration dt;
		while(true) {
			dt = list_v_meas_.front().get_time() - list_v_meas_.back().get_time();
			if (dt.toSec() > max_list_size_) {
				list_v_meas_.pop_back();
			} else {
				break;
			}
		}
		// ROS_INFO("Time deviation: %f", dt.toSec());
	}


	Eigen::MatrixXd GetFk(const double &dt) {
		Eigen::MatrixXd Fk = Eigen::MatrixXd::Identity(3,3);
		return Fk;
	}

	Eigen::MatrixXd GetBk(const double &dt) {
		Eigen::MatrixXd Bk = Eigen::MatrixXd::Zero(8,3);
		Bk.block(0,0,3,3) = 0.5*dt*dt*Eigen::MatrixXd::Identity(3,3);
		Bk.block(3,0,3,3) = dt*Eigen::MatrixXd::Identity(3,3);
		return Bk;
	}

	Eigen::MatrixXd GetGammak(const double &dt) {
		Eigen::MatrixXd Gammak = Eigen::MatrixXd::Zero(8,5);
		Gammak.block(0,0,3,3) = 0.5*dt*dt*Eigen::MatrixXd::Identity(3,3);
		Gammak.block(3,0,3,3) = dt*Eigen::MatrixXd::Identity(3,3);
		Gammak.block(6,3,2,2) = dt*Eigen::MatrixXd::Identity(2,2);
		return Gammak;
	}

	void PropagationUpdate(const double &dt) {
		Eigen::MatrixXd Fk = this->GetFk(dt);
		Eigen::MatrixXd Bk = this->GetBk(dt);
		Eigen::MatrixXd Gammak = this->GetGammak(dt);
		X_ = Fk*X_ + Bk*v_meas_.get_meas();
		Pk_k1_ = Fk*Pk1_k1_*Fk.transpose() + dt*Gammak*Qk_*Gammak.transpose();
	}


	void RangeMeasurementUpdate(const uint32_t &dist, const Eigen::VectorXd &anchor,
		                       const Eigen::MatrixXd &Rk) {
		// pred_dist = pred_range(anchor);
		Eigen::MatrixXd Hk_range_ = GetHk(anchor);
		const uint32_t y = dist - pred_range(anchor);
		const Eigen::MatrixXd S = Hk_range_*Pk_k1_*Hk_range_.transpose() + Rk;
		const Eigen::MatrixXd K = Pk_k1_*Hk_range_.transpose()*S.inverse();
		X_ = X_ + K*y;
		const Eigen::MatrixXd T = Eigen::MatrixXd::Identity(8,8) - K*Hk_range_;
		Pk_k_ = T*Pk_k1_*T.transpose() + K*Rk*K.transpose();
		Pk1_k1_ = Pk_k_;
	}

	bool CanProcessMeasurement(const ros::Time &stamp) {
		// Do not run before initializing time
		if (init_time_.toSec() == 0) {
			return false;
		}
		// Do not run if stamp is older than intial time
		if ((stamp - init_time_).toSec() < 0) {
			return false;
		}
		return true;
	}

	void RangeCallback(const kf_odom::DeviceRangeMsg::ConstPtr &msg) {
		if (!CanProcessMeasurement(msg->header.stamp)) {
			return;
		}
		Eigen::Vector3d anchor(msg->anchor_x,msg->anchor_y,msg->anchor_z);

		this->RangeMeasurementUpdate(msg->distance, anchor, R_pv_);
	}

	void VelCallback(const nav_msgs::Odometry::ConstPtr &msg) {
		geometry_msgs::Vector3 vel = 
			helper::convert_to_inertial_frame(msg->pose.pose.orientation, 
				                              msg->twist.twist.linear);
			Eigen::Vector3d vel_meas = helper::vector3_to_eigen(vel);
		
		// Initialization (first time this callback runs)
		if (init_time_.toSec() == 0) {
			init_time_ = msg->header.stamp;
			cur_time_ = init_time_;
			const double dt = 0.0;
			v_meas_ = vel_stamped(vel_meas, cur_time_, dt);
			this->AddMeasToList(v_meas_);
			return;
		}

		const double dt = (msg->header.stamp - cur_time_).toSec();
		if (dt > 0) {
			cur_time_ = msg->header.stamp;
			v_meas_ = vel_stamped(vel_meas, cur_time_, dt);
			this->PropagationUpdate(dt);
			this->AddMeasToList(v_meas_);
		}
	}
};

int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle n("~");

  Eigen::Vector3d sigma_nu_a(0.7, 0.7, 1.8); // Accelerometer standard dev (m/s^2)
  double sigma_nu_b = 1.0/20.0;   // Bias_dot = one meter per 20 seconds
  double sigma_eta_v = 0.02;      // Velocity measurements standard deviation
  double sigma_eta_b = 0.1;      // Bias measurements standard deviation
  double large_number = 1000000;
  Eigen::VectorXd X0 = Eigen::VectorXd::Zero(3);
  Eigen::MatrixXd P0 = large_number*Eigen::MatrixXd::Identity(8,8);
  double max_list_size = 1.0;  // One second as maximum width in the list

  KF kf_obj(&n, sigma_nu_a, sigma_nu_b, sigma_eta_b, 
  	        sigma_eta_v, X0, P0, max_list_size);

  ros::spin();

  return 0;
} 