#ifndef SEARCH_EXAMPLE_DIJKSTRA__HPP__
#define SEARCH_EXAMPLE_DIJKSTRA__HPP__

#include "search_example/algorithm_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/map_meta_data.hpp"
#include <map>

namespace search_example{

namespace algorithm{

/**
 * @brief 
 * 
 */
class DijkstraCell{
public:
/**
 * @brief Construct a new Dijkstra Cell object
 * 
 * @param col 
 * @param row 
 */
DijkstraCell(uint32_t col, uint32_t row);

/**
 * @brief Get the Cost object
 * 
 * @return double 
 */
double getCost();

/**
 * @brief Set the Cost object
 * 
 * @param cost 
 */
void setCost(double cost);

/**
 * @brief Get the Point object
 * 
 * @return std::pair<uint32_t, uint32_t> 
 */
std::pair<uint32_t, uint32_t> getPoint();

/**
 * @brief Get the Pre Cell object
 * 
 * @return DijkstraCell* 
 */
DijkstraCell* getPreCell();

/**
 * @brief Set the Pre Cell object
 * 
 * @param prev 
 */
void setPreCell(DijkstraCell* prev);

/**
 * @brief Get the Hash object
 * 
 * @return uint64_t 
 */
uint64_t getHash();

/**
 * @brief Get the Hash Around object
 * 
 * @return std::vector<uint64_t> 
 */
std::vector<uint64_t> getHashAround() const;

using SharedPtr = std::shared_ptr<DijkstraCell>;

protected:

double cost_;
uint32_t col_, row_;
DijkstraCell* prev_cell_;

};

/**
 * @brief 
 * 
 */
class Dijkstra: public AlgorithmBase
{
protected:
rclcpp::Node::SharedPtr node_;
std::map<uint64_t, DijkstraCell::SharedPtr> map_;
geometry_msgs::msg::Pose2D goal_, start_;
nav_msgs::msg::MapMetaData::SharedPtr meta_data_;

/**
 * @brief 
 * 
 */
void resetMap();

/**
 * @brief 
 * 
 * @param cell 
 * @param update_cells 
 */
void updatePointAround(const DijkstraCell::SharedPtr& cell,
    std::vector<uint64_t>& update_cells);

/**
 * @brief 
 * 
 * @param start_key 
 */
void finishTable(const uint64_t& start_key);

/**
 * @brief 
 * 
 * @param goal_key 
 * @return nav_msgs::msg::Path* 
 */
nav_msgs::msg::Path* fromGoalToPath(const uint64_t& goal_key);

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