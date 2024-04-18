#ifndef BASE_MODEL_ROBOT_MODEL__HPP__
#define BASE_MODEL_ROBOT_MODEL__HPP__

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "rclcpp/rclcpp.hpp"

namespace base_robot_model{
namespace robot{

using pose_callback = std::function<void(const geometry_msgs::msg::PoseStamped&)>;

class RobotModel {
public:

/**
 * @brief Construct a new Robot Model object
 * 
 * @param node 
 */
RobotModel(const rclcpp::Node::WeakPtr& node);

~RobotModel();

/**
 * @brief 
 * 
 * @param x 
 * @param y 
 * @param z 
 */
void init(const float& x, const float& y, const float& theta);

/**
 * @brief 
 * 
 * @param pose 
 */
void init(const geometry_msgs::msg::Pose& pose);

/**
 * @brief 
 * 
 * @param grid 
 */
void loadMap(const nav_msgs::msg::OccupancyGrid::WeakPtr& grid);

/**
 * @brief Set the Speed object
 * 
 * @param speed 
 */
void setSpeed(const geometry_msgs::msg::Twist::SharedPtr& speed);

/**
 * @brief Get the Current Position object
 * 
 * @return geometry_msgs::msg::PoseStamped 
 */
geometry_msgs::msg::PoseStamped getCurrentPosition();

void addCallback(const pose_callback& update_pose);

protected:

geometry_msgs::msg::PoseStamped last_pose_;
geometry_msgs::msg::Twist last_speed_, target_speed_;
float linear_accel_, angular_accel_, limit_linear_, limit_angular_;
nav_msgs::msg::OccupancyGrid::SharedPtr grid_;
rclcpp::Node::SharedPtr node_;
pose_callback update_pose_callback_;
std::mutex mutex_;

/**
 * @brief 
 * 
 * @param new_pose 
 * @return true 
 * @return false 
 */
std::pair<bool, geometry_msgs::msg::Pose> isCollided();

/**
 * @brief Get the Current Speed object
 * 
 * @param d_t 
 */
void getCurrentSpeed(const float& d_t);

/**
 * @brief 
 * 
 */
void declareParam();

/**
 * @brief Get the Param object
 * 
 */
void getParam();

};
} // end robot
} // end base_robot_model

#endif