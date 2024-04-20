#include "base_robot_model/robot_model.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "base_robot_model/utils/geometry_utils.hpp"
#include "search_example/utils/map_utils.hpp"

namespace base_robot_model{
namespace robot{

RobotModel::RobotModel(const rclcpp::Node::WeakPtr& node){
    node_ = node.lock();
    declareParam();
    getParam();
}

RobotModel::~RobotModel(){}

void RobotModel::init(const float& x, const float& y, const float& theta){
    geometry_msgs::msg::Pose new_pose;
    new_pose.position.x = x;
    new_pose.position.y = y;
    new_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(theta);
    init(new_pose);
}

void RobotModel::init(const geometry_msgs::msg::Pose& pose){
    mutex_.lock();
    last_pose_.pose = pose;
    last_speed_.angular.z = 0.0;
    last_speed_.linear.x = 0.0;
    mutex_.unlock();
    getCurrentPosition();
}

void RobotModel::loadMap(const nav_msgs::msg::OccupancyGrid::WeakPtr& grid){
    grid_ = grid.lock();
}

void RobotModel::setSpeed(const geometry_msgs::msg::Twist::SharedPtr& speed){
    getCurrentPosition();
    last_speed_ = *speed;
}

geometry_msgs::msg::PoseStamped RobotModel::getCurrentPosition(){
    const std::lock_guard<std::mutex> lock(mutex_);
    auto [is_collied, new_pose] = isCollided();
    if(is_collied){
        last_speed_.angular.z = 0.0;
        last_speed_.linear.x = 0.0;
    }
    last_pose_.pose = new_pose;
    last_pose_.header.frame_id = "map";
    last_pose_.header.stamp = node_->get_clock()->now();
    update_pose_callback_(last_pose_);
    return last_pose_;
}

void RobotModel::addCallback(const pose_callback& update_pose){
    update_pose_callback_ = update_pose;
}

std::pair<bool, geometry_msgs::msg::Pose> RobotModel::isCollided(){
    auto d_t_nano = node_->get_clock()->now().nanoseconds() - last_pose_.header.stamp.nanosec;
    float d_t = static_cast<float>(d_t_nano)/10e9;
    getCurrentSpeed(d_t);
    auto new_pose = utils::updateNewRobotPose(last_pose_.pose, last_speed_, d_t);
    if(grid_ == nullptr)
        return {false, new_pose};
    auto delta_x = new_pose.position.x - last_pose_.pose.position.x;
    auto delta_y = new_pose.position.y - last_pose_.pose.position.y;
    auto delta_theta = d_t*last_speed_.angular.z;
    auto current_theta = utils::toYaw(last_pose_.pose.orientation);
    auto delta_d = sqrt(delta_x*delta_x + delta_y*delta_y);
    new_pose = last_pose_.pose;
    size_t resolution = static_cast<size_t>(delta_d/grid_->info.resolution);
    for(size_t i = 0; i <= resolution; i++){
        auto x = last_pose_.pose.position.x + delta_x/resolution*i;
        auto y = last_pose_.pose.position.y + delta_y/resolution*i;
        geometry_msgs::msg::Pose2D pose_2d;
        pose_2d.x = x;
        pose_2d.y = y;
        auto index = search_example::utils::poseToIndex(pose_2d, grid_->info);
        if(grid_->data[index] != 0){
            return {true, new_pose};
        }
        new_pose.position.x = x;
        new_pose.position.y = y;
        new_pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(current_theta + delta_theta/resolution*i);
    }
    return {false, new_pose};
}

void RobotModel::getCurrentSpeed(const float& d_t){
    auto clamp = [](const float& target, const float& limit){
        auto value = (abs(target) > limit)? limit : abs(target);
        return (target < 0.f) ? -value : value;
    };
    auto linear_target = clamp(target_speed_.linear.x, limit_linear_);
    auto linear_angular = clamp(target_speed_.angular.z, limit_angular_);
    auto delta_linear = linear_target - last_speed_.linear.x;
    auto delta_angular = linear_angular - last_speed_.angular.z;
    delta_linear = clamp(delta_linear, linear_accel_ * d_t);
    delta_angular = clamp(delta_angular, angular_accel_ * d_t);
    last_speed_.linear.x += delta_linear;
    last_speed_.angular.z += delta_angular;
}

void RobotModel::declareParam(){
    node_->declare_parameter<float>("robot_model.linear_speed_limit", 4.f);
    node_->declare_parameter<float>("robot_model.angular_speed_limit", 4.f);
    node_->declare_parameter<float>("robot_model.linear_accel", 1.f);
    node_->declare_parameter<float>("robot_model.angular_accel", 1.5f);
}

void RobotModel::getParam(){
    limit_linear_ = node_->get_parameter("robot_model.linear_speed_limit").as_double();
    limit_angular_ = node_->get_parameter("robot_model.angular_speed_limit").as_double();
    linear_accel_ = node_->get_parameter("robot_model.linear_accel").as_double();
    angular_accel_ = node_->get_parameter("robot_model.angular_accel").as_double();
}

}
}