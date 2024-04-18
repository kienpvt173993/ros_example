#ifndef BASE_MODEL_GEOMETRY_UTILS__HPP__
#define BASE_MODEL_GEOMETRY_UTILS__HPP__

#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_util/geometry_utils.hpp"

namespace base_robot_model{
namespace utils{

inline float toYaw(const geometry_msgs::msg::Quaternion& q){
    double siny_cosp = 2 * (q.w * q.z + q.x * q.y);
    float cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    float yaw = std::atan2(siny_cosp, cosy_cosp);
    return yaw;
}

inline geometry_msgs::msg::Pose updateNewRobotPose(const geometry_msgs::msg::Pose& last_pose,
    const geometry_msgs::msg::Twist& twist, const float& d_t){
    float x = last_pose.position.x;
    float y = last_pose.position.y;
    float theta = toYaw(last_pose.orientation);
    float delta_yaw = d_t*twist.angular.z;
    float delta_d = d_t*twist.linear.x;
    float d_x = delta_d*cos(delta_yaw);
    float d_y = delta_d*sin(delta_yaw);
    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = x + d_x;
    new_pose.position.y = y + d_y;
    new_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta + delta_yaw);
    return new_pose;
}

}
}

#endif