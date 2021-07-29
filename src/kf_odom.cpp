// ROS
#include "ros/ros.h"
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <std_msgs/Float64.h>

#include <Eigen/Dense>

#include "kf_odom/helper.h"

class accel_stamped {
 public:
	accel_stamped() {}
	accel_stamped(const Eigen::Vector3d &meas,
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
	Eigen::MatrixXd Hk_odom_, Hk_bias_;
	Eigen::MatrixXd Qk_;
	// Eigen::MatrixXd Rk_;
	Eigen::MatrixXd R_pv_;
	Eigen::MatrixXd R_v_;
	Eigen::MatrixXd R_b_;
	Eigen::VectorXd X_;
	Eigen::MatrixXd Pk1_k1_, Pk_k1_, Pk_k_;
	accel_stamped a_meas_;
	ros::Time cur_time_;
	ros::NodeHandle *nh_;
	ros::Subscriber odom_sub_, imu_sub_, bias_sub_;
	ros::Time init_time_;

	// List of past measurements
	std::list<accel_stamped> list_a_meas_;
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
		Hk_odom_ = Eigen::MatrixXd::Zero(6,8);
		Hk_bias_ = Eigen::MatrixXd::Zero(2,8);
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
		std::string odom_topic, imu_topic, bias_topic;
		nh_->getParam("odom_topic", odom_topic);
		nh_->getParam("imu_topic",  imu_topic);
		nh_->getParam("bias_topic", bias_topic);
		ROS_INFO("%s %s %s", odom_topic.c_str(), imu_topic.c_str(), bias_topic.c_str());
		odom_sub_ = nh_->subscribe(odom_topic, 1, &KF::OdomCallback, this);
		imu_sub_  = nh_->subscribe(imu_topic,  1, &KF::ImuCallback,  this);
		bias_sub_ = nh_->subscribe(bias_topic, 1, &KF::BiasCallback, this);
		ROS_INFO("[kf_odom] Odom topic: %s", odom_sub_.getTopic().c_str());
		ROS_INFO("[kf_odom] Imu topic: %s",  imu_sub_.getTopic().c_str());
		ROS_INFO("[kf_odom] Bias topic: %s", bias_sub_.getTopic().c_str());

		// Measurement matrix
		Hk_odom_ = Eigen::MatrixXd::Identity(6,8);
		Hk_odom_.block(0,6,2,2) = Eigen::MatrixXd::Identity(2,2);
		Hk_bias_.block(0,6,2,2) = Eigen::MatrixXd::Identity(2,2);
		std::cout << "Hk_odom_:\n" << Hk_odom_ << std::endl << std::endl;
		std::cout << "Hk_bias_:\n" << Hk_bias_ << std::endl << std::endl;

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
		// ROS_INFO("Time deviation: %f", dt.toSec());
	}

	Eigen::MatrixXd GetFk(const double &dt) {
		Eigen::MatrixXd Fk = Eigen::MatrixXd::Identity(8,8);
		Fk.block(0, 3, 3, 3) = dt*Eigen::MatrixXd::Identity(3,3);
		return Fk;
	}

	Eigen::MatrixXd GetInvFk(const double &dt) {
		Eigen::MatrixXd Fk_inv(8,8);
		Fk_inv = Eigen::MatrixXd::Identity(9,9);
		Fk_inv.block(0, 3, 3, 3) = -dt*Eigen::MatrixXd::Identity(3,3);
		return Fk_inv;
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
		X_ = Fk*X_ + Bk*a_meas_.get_meas();
		Pk_k1_ = Fk*Pk1_k1_*Fk.transpose() + dt*Gammak*Qk_*Gammak.transpose();
	}

	Eigen::VectorXd PropagateStateBackwards(
			const double &dt, const Eigen::VectorXd &X,
			const Eigen::Vector3d accel) {
		Eigen::MatrixXd Fk_inv = this->GetInvFk(dt);
		Eigen::MatrixXd Bk = this->GetBk(dt);
		return Fk_inv*(X - Bk*accel);
	}

	Eigen::VectorXd PropagateStateForward(
			const double &dt, const double &X,
			const Eigen::Vector3d accel) {
		Eigen::MatrixXd Fk = this->GetFk(dt);
		Eigen::MatrixXd Bk = this->GetBk(dt);
		return Fk*X + Bk*accel;
	}

	Eigen::VectorXd GetStateInPast(const ros::Time &time) {
		Eigen::VectorXd X_past = X_;
		uint idx = 0;
		std::list<accel_stamped>::iterator it;
		for (it = list_a_meas_.begin(); it != list_a_meas_.end(); ++it) {
			if ((it->get_time().toSec() - it->get_dt()) > time.toSec()) {
				const double dt = it->get_dt();
				X_past = PropagateStateBackwards(dt, X_past, it->get_meas());
			} else {
				const double dt = (it->get_time() - time).toSec();
				X_past = PropagateStateBackwards(dt, X_past, it->get_meas());
				break;
			}
		}

		return X_past;
	}

	void OdomMeasurementUpdate(const Eigen::VectorXd &odom,
		                       const Eigen::MatrixXd &Rk) {
		const Eigen::VectorXd y = odom - Hk_odom_*X_;
		const Eigen::MatrixXd S = Hk_odom_*Pk_k1_*Hk_odom_.transpose() + Rk;
		const Eigen::MatrixXd K = Pk_k1_*Hk_odom_.transpose()*S.inverse();
		X_ = X_ + K*y;
		const Eigen::MatrixXd T = Eigen::MatrixXd::Identity(8,8) - K*Hk_odom_;
		Pk_k_ = T*Pk_k1_*T.transpose() + K*Rk*K.transpose();
		Pk1_k1_ = Pk_k_;
	}

	void BiasMeasurementUpdate(const Eigen::VectorXd &bias,
		                       const Eigen::MatrixXd &Rk) {
		const Eigen::VectorXd y = bias - Hk_bias_*X_;
		const Eigen::MatrixXd S = Hk_bias_*Pk_k1_*Hk_bias_.transpose() + Rk;
		const Eigen::MatrixXd K = Pk_k1_*Hk_bias_.transpose()*S.inverse();
		X_ = X_ + K*y;
		const Eigen::MatrixXd T = Eigen::MatrixXd::Identity(8,8) - K*Hk_bias_;
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

	void OdomCallback(const nav_msgs::Odometry::ConstPtr &msg) {
		if (!CanProcessMeasurement(msg->header.stamp)) {
			return;
		}

		Eigen::Vector3d pos = helper::point_to_eigen(msg->pose.pose.position);
		Eigen::Vector3d vel = helper::vector3_to_eigen(msg->twist.twist.linear);
		Eigen::VectorXd odom;
		odom << pos(0), pos(1), pos(2), vel(0), vel(1), vel(2);
		Eigen::Matrix3d cov_pos;
		cov_pos << msg->pose.covariance[0],  msg->pose.covariance[1],  msg->pose.covariance[2],
		           msg->pose.covariance[6],  msg->pose.covariance[7],  msg->pose.covariance[8],
		           msg->pose.covariance[12], msg->pose.covariance[13], msg->pose.covariance[14];
		R_pv_.block(0,0,3,3) = cov_pos;
		this->OdomMeasurementUpdate(odom, R_pv_);
	}

	void ImuCallback(const sensor_msgs::Imu::ConstPtr &msg) {
		geometry_msgs::Vector3 accel = 
			helper::convert_to_inertial_frame(msg->orientation, 
				                              msg->linear_acceleration);
		
		// Deduct gravity from acceleration measurement
		Eigen::Vector3d accel_meas = helper::vector3_to_eigen(accel) + 
		                             Eigen::Vector3d(0.0, 0.0, -9.81);
		
		// Initialization (first time this callback runs)
		if (init_time_.toSec() == 0) {
			init_time_ = msg->header.stamp;
			cur_time_ = init_time_;
			const double dt = 0.0;
			a_meas_ = accel_stamped(accel_meas, cur_time_, dt);
			this->AddMeasToList(a_meas_);
			return;
		}

		const double dt = (msg->header.stamp - cur_time_).toSec();
		if (dt > 0) {
			cur_time_ = msg->header.stamp;
			a_meas_ = accel_stamped(accel_meas, cur_time_, dt);
			this->PropagationUpdate(dt);
			this->AddMeasToList(a_meas_);
		}
	}

	void BiasCallback(const geometry_msgs::TransformStamped::ConstPtr &msg) {
		ROS_INFO("bias callback!");
		if (!CanProcessMeasurement(msg->header.stamp)) {
			return;
		}

		Eigen::Vector3d bias_meas = helper::vector3_to_eigen(msg->transform.translation);
		this->BiasMeasurementUpdate(bias_meas, R_b_);
	}
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "~");
  ros::NodeHandle n("~");

  Eigen::Vector3d sigma_nu_a(0.7, 0.7, 1.8); // Accelerometer standard dev (m/s^2)
  double sigma_nu_b = 1.0/20.0;   // Bias_dot = one meter per 20 seconds
  double sigma_eta_v = 0.02;      // Velocity measurements standard deviation
  double sigma_eta_b = 0.1;      // Bias measurements standard deviation
  double large_number = 1000000;
  Eigen::VectorXd X0 = Eigen::VectorXd::Zero(8);
  Eigen::MatrixXd P0 = large_number*Eigen::MatrixXd::Identity(8,8);
  double max_list_size = 1.0;  // One second as maximum width in the list

  KF kf_obj(&n, sigma_nu_a, sigma_nu_b, sigma_eta_b, 
  	        sigma_eta_v, X0, P0, max_list_size);

  // std::cout << "parameters:" << std::endl;
  // std::cout << odom_topic << " " << frame_id << " " << " " << child_frame_id << std::endl;

  // ros::Subscriber sub = n.subscribe("/vislam/odometry", 5, odomCallback);
  
  ros::spin();

  return 0;
} 