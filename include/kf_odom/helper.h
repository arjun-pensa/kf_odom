#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/Quaternion.h"
#include <tf/tf.h>
#include <Eigen/Dense>

namespace helper {

	geometry_msgs::Vector3 set_vector3(const double &x, const double &y, const double &z);

	geometry_msgs::Vector3 eigen_to_vector3(const Eigen::Vector3d& vector3d);

	geometry_msgs::Vector3 convert_to_body_frame(
	const geometry_msgs::Quaternion &quat,
	const geometry_msgs::Vector3 &vec);

	geometry_msgs::Vector3 convert_to_inertial_frame(
	const geometry_msgs::Quaternion &quat,
	const geometry_msgs::Vector3 &vec);

	Eigen::Vector3d point_to_eigen(const geometry_msgs::Point& point);

	Eigen::Vector3d vector3_to_eigen(const geometry_msgs::Vector3& vector3);

}  // namespace helper