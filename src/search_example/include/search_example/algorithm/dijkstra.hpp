#ifndef SEARCH_EXAMPLE_DIJKSTRA__HPP__
#define SEARCH_EXAMPLE_DIJKSTRA__HPP__

#include "search_example/algorithm_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include <map>

namespace search_example{

namespace algorithm{

class Dijkstra: public AlgorithmBase
{
public:
explicit Dijkstra();

~Dijkstra();
void initialize(const rclcpp::Node::WeakPtr& node) override final;
void setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid) override final;
void setStart(const geometry_msgs::msg::Pose2D start) override final;
void setGoal(const geometry_msgs::msg::Pose2D goal) override final;
nav_msgs::msg::Path* getPath() override final;

};

} // end namespace algorithm
} // end namespace search_example

#endif