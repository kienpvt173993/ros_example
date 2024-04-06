#ifndef SEARCH_EXAMPLE_A_START__HPP__
#define SEARCH_EXAMPLE_A_START__HPP__

#include "search_example/algorithm_base.hpp"
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include <limits>
#include <map>

namespace search_example{
namespace algorithm{

enum class CellState {unread, close, open, constant};

struct CellCost{
    double cost = std::numeric_limits<double>::max();
    double g_cost = std::numeric_limits<double>::max();
    CellCost* pre_cell = nullptr;
    CellState state = CellState::unread;
    const size_t index;

    /**
     * @brief 
     * 
     */
    void reset(){
        cost = std::numeric_limits<double>::max();
        g_cost = std::numeric_limits<double>::max();
        pre_cell = nullptr;
        state = CellState::unread;
    }

    CellCost(const size_t& index): index(index){}
    using SharedPtr = std::shared_ptr<CellCost>;
};

class AStart: public AlgorithmBase{
protected:
rclcpp::Node::SharedPtr node_;
geometry_msgs::msg::Pose2D goal_, start_;
std::vector<CellCost::SharedPtr> table_;
nav_msgs::msg::MapMetaData::SharedPtr meta_data_;

/**
 * @brief 
 * 
 */
void resetMap();

/**
 * @brief 
 * 
 * @param index 
 * @param goal_index 
 * @param update_list 
 */
void updatePointAround(const size_t& index, const size_t& goal_index,
    std::vector<size_t>& open_list);
/**
 * @brief 
 * 
 * @param start_index
 */
void finishTable(const size_t& start_index, const size_t& goal_index);

/**
 * @brief 
 * 
 * @param index 
 * @param pre_cell_index 
 * @return double 
 */
double gScore(const size_t& index, const size_t& pre_cell_index);

/**
 * @brief 
 * 
 * @param index 
 * @param goal_index 
 * @return double 
 */
double hScore(const size_t& index, const size_t& goal_index);

/**
 * @brief 
 * 
 * @param index 
 * @param pre_cell_index 
 * @param goal_index 
 * @return double 
 */
double fScore(const size_t& index, const size_t& pre_cell_index, const size_t& goal_index);

/**
 * @brief 
 * 
 * @param goal_index
 * @return nav_msgs::msg::Path* 
 */
nav_msgs::msg::Path* fromGoalToPath(const size_t& goal_index);

/**
 * @brief 
 * 
 * @param index 
 * @return true 
 * @return false 
 */
bool cellAvailable(const size_t& index);

std::vector<size_t> getPointAround(const size_t& index);

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