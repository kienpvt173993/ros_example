#ifndef SEARCH_EXAMPLE_ALGORITHM_BASE__HPP__
#define SEARCH_EXAMPLE_ALGORITHM_BASE__HPP__

#include "nav_msgs/msg/path.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "search_example/algorithm_base.hpp"
#include "rclcpp/rclcpp.hpp"

namespace search_example{

/**
 * @brief 
 * 
 */
class AlgorithmBase
{
public:

virtual ~AlgorithmBase(){};

/**
 * @brief 
 * 
 * @param parent 
 */
virtual void initialize(const rclcpp::Node::WeakPtr& parent) = 0;

/**
 * @brief Set the Map object
 * 
 * @param grid 
 */
virtual void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid) = 0;

/**
 * @brief Set the Start object
 * 
 * @param start 
 */
virtual void setStart(const geometry_msgs::msg::Pose2D start) = 0;

/**
 * @brief Set the Goal object
 * 
 * @param goal 
 */
virtual void setGoal(const geometry_msgs::msg::Pose2D goal) = 0;

/**
 * @brief Get the Path object
 * 
 * @return nav_msgs::msg::Path* 
 */
virtual nav_msgs::msg::Path* getPath() = 0;

using SharedPtr =  std::shared_ptr<AlgorithmBase>;

}; // end namespace search_example


}

#endif