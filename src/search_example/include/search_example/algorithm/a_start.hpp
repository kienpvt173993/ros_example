#ifndef SEARCH_EXAMPLE_A_START__HPP__
#define SEARCH_EXAMPLE_A_START__HPP__

#include "search_example/algorithm_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <limits>
#include <map>

namespace search_example{
namespace algorithm{

class AStart: public AlgorithmBase{
public:
explicit AStart();

~AStart();
void initialize(const rclcpp::Node::WeakPtr& node) override final;
void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid) override final;
void setStart(const geometry_msgs::msg::Pose2D start) override final;
void setGoal(const geometry_msgs::msg::Pose2D goal) override final;
nav_msgs::msg::Path* getPath() override final;

};

} // end namespace algorithm
} // end namespace search_example

#endif