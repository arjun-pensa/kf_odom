
#include "kf_odom/helper.h"

namespace helper {

	geometry_msgs::Vector3 set_vector3(const double &x, const double &y, const double &z) {
		geometry_msgs::Vector3 v;
		v.x = x; v.y = y; v.z = z;
		return v;
	}

	geometry_msgs::Vector3 eigen_to_vector3(const Eigen::Vector3d& vector3d) {
		geometry_msgs::Vector3 vec3;
		vec3.x = vector3d[0];
		vec3.y = vector3d[1];
		vec3.z = vector3d[2];
		return vec3;
	}

	geometry_msgs::Vector3 convert_to_body_frame(
	const geometry_msgs::Quaternion &quat,
	const geometry_msgs::Vector3 &vec) {
		tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
		tf::Quaternion q_world(vec.x, vec.y, vec.z, 0.0);
		tf::Quaternion q_vec_body = q.inverse()*q_world*q;
		geometry_msgs::Vector3 vec_body = set_vector3(q_vec_body.x(), q_vec_body.y(), q_vec_body.z());
		return vec_body;
	}

	geometry_msgs::Vector3 convert_to_inertial_frame(
	const geometry_msgs::Quaternion &quat,
	const geometry_msgs::Vector3 &vec) {
		tf::Quaternion q(quat.x, quat.y, quat.z, quat.w);
		tf::Quaternion q_body(vec.x, vec.y, vec.z, 0.0);
		tf::Quaternion q_vec_world = q*q_body*q.inverse();
		geometry_msgs::Vector3 vec_body = set_vector3(q_vec_world.x(), q_vec_world.y(), q_vec_world.z());
		return vec_body;
	}

	Eigen::Vector3d point_to_eigen(const geometry_msgs::Point& point) {
		return Eigen::Vector3d(point.x, point.y, point.z);
	}

	Eigen::Vector3d vector3_to_eigen(const geometry_msgs::Vector3& vector3) {
		return Eigen::Vector3d(vector3.x, vector3.y, vector3.z);
	}

}  // namespace helper