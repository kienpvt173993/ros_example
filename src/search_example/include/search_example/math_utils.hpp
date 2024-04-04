#ifndef SEARCH_EXAMPLE_MATH_UTILS__HPP__
#define SEARCH_EXAMPLE_MATH_UTILS__HPP__

#include "nav2_util/geometry_utils.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include "tf2/utils.h"

namespace search_example{

/**
 * @brief 
 * 
 * @param pose 
 * @return geometry_msgs::msg::Pose2D 
 */
inline geometry_msgs::msg::Pose2D toPose2D(const geometry_msgs::msg::Pose& pose){
    tf2::Quaternion q(pose.orientation.x,
        pose.orientation.y,
        pose.orientation.z,
        pose.orientation.w
    );
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    geometry_msgs::msg::Pose2D pose_2d;
    pose_2d.theta = yaw;
    pose_2d.x = pose.position.x;
    pose_2d.y = pose.position.y;
    return pose_2d;
}

/**
 * @brief 
 * 
 * @param pose_2d 
 * @return geometry_msgs::msg::Pose 
 */
inline geometry_msgs::msg::Pose fromPose2D(const geometry_msgs::msg::Pose2D& pose_2d){
    geometry_msgs::msg::Pose pose;
    pose.position.x = pose_2d.x;
    pose.position.y = pose_2d.y;
    pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(pose_2d.theta);
    return pose;
}

/**
 * @brief 
 * 
 * @param index 
 * @param width 
 * @return std::pair<uint32_t, uint32_t> 
 */
inline std::pair<uint32_t, uint32_t> indexToPoint(const size_t& index, const uint32_t& width){
    return std::make_pair(
        static_cast<uint32_t>(index)%width,
        static_cast<uint32_t>(index)/width
    );
}

/**
 * @brief 
 * 
 * @param cell 
 * @param pre_pose 
 * @param meta_data 
 * @return geometry_msgs::msg::Pose 
 */
inline geometry_msgs::msg::Pose cellToPose(const std::pair<uint32_t, uint32_t>& cell,
    const geometry_msgs::msg::Pose2D& pre_pose,
    const nav_msgs::msg::MapMetaData& meta_data){
    auto col = cell.first;
    auto row = cell.second;
    geometry_msgs::msg::Pose pose;
    geometry_msgs::msg::Pose2D pose_2d;
    pose_2d.x = col*meta_data.resolution + meta_data.origin.position.x + meta_data.resolution/2.0;
    pose_2d.y = row*meta_data.resolution + meta_data.origin.position.y + meta_data.resolution/2.0;
    pose_2d.theta = atan2(pose_2d.y - pre_pose.y,
        pose_2d.x - pre_pose.x);
    return fromPose2D(pose_2d);
}

/**
 * @brief 
 * 
 * @param pose_2d 
 * @param meta_data 
 * @return std::pair<uint32_t, uint32_t> 
 */
inline std::pair<uint32_t, uint32_t> poseToCell(const geometry_msgs::msg::Pose2D& pose_2d,
    const nav_msgs::msg::MapMetaData& meta_data){
    return std::make_pair(
        static_cast<uint32_t>((pose_2d.x - meta_data.origin.position.x)/meta_data.resolution),
        static_cast<uint32_t>((pose_2d.y - meta_data.origin.position.y)/meta_data.resolution)
    );
}

template <typename T>
inline uint64_t pointIntToKey(const T& row, const T& col){
    return ((static_cast<uint64_t>(row) << 32) + static_cast<uint64_t>(col));
}

}

#endif