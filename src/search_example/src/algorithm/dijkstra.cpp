#include "search_example/algorithm/dijkstra.hpp"
#include <limits>
#include "search_example/utils/search_utils.hpp"

namespace search_example{
namespace algorithm{
DijkstraCell::DijkstraCell(uint32_t col, uint32_t row):
    col_(col), row_(row), prev_cell_(nullptr){
    cost_ = std::numeric_limits<double>::max();
}

double DijkstraCell::getCost(){ return cost_; }

void DijkstraCell::setCost(double cost){
    cost_ = cost;
}

std::pair<uint32_t, uint32_t> DijkstraCell::getPoint(){
    return std::make_pair(col_, row_);
}

DijkstraCell* DijkstraCell::getPreCell(){ return prev_cell_; }

void DijkstraCell::setPreCell(DijkstraCell* prev){
    prev_cell_ = prev;
};

uint64_t DijkstraCell::getHash(){
    return utils::pointIntToKey(row_, col_);
}

std::vector<uint64_t> DijkstraCell::getHashAround() const{
    uint64_t row = static_cast<uint64_t>(row_);
    uint64_t col = static_cast<uint64_t>(col_);
    return {
        utils::pointIntToKey(row-1, col-1),
        utils::pointIntToKey(row-1, col),
        utils::pointIntToKey(row-1, col+1),
        utils::pointIntToKey(row, col-1),
        utils::pointIntToKey(row, col+1),
        utils::pointIntToKey(row+1, col-1),
        utils::pointIntToKey(row+1, col),
        utils::pointIntToKey(row+1, col+1),
    };
}

Dijkstra::Dijkstra(){}

Dijkstra::~Dijkstra(){}

void Dijkstra::initialize(const rclcpp::Node::WeakPtr&){
}

void Dijkstra::setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid){
    for(size_t i = 0; i < grid->data.size(); i++){
        if(grid->data[i] == 0){
            auto [col, row] = utils::indexToPointInt(i, grid->info.width);
            uint64_t key = utils::pointIntToKey(row, col);
            DijkstraCell::SharedPtr new_cell = std::make_shared<DijkstraCell>(col, row);
            this->map_.emplace(std::make_pair(key, new_cell));
        }
    }
    meta_data_.reset(&grid->info);
}

void Dijkstra::setStart(const geometry_msgs::msg::Pose2D start){
    start_ = start;
}

void Dijkstra::setGoal(const geometry_msgs::msg::Pose2D goal){
    goal_ = goal;
}

void Dijkstra::resetMap(){
    for(auto& cell: map_){
        cell.second->setCost(std::numeric_limits<double>::max());
        cell.second->setPreCell(nullptr);
    }
}

void Dijkstra::updatePointAround(const DijkstraCell::SharedPtr& cell,
    std::vector<uint64_t>& update_cells){
    auto around_cells = cell->getHashAround();
    for(const auto& cell_key: around_cells){
        auto it = map_.find(cell_key);
        if(it == map_.end()) continue;
        if(it->second->getPreCell() == nullptr)
            update_cells.push_back(it->second->getHash());
        auto cell_point = cell->getPoint();
        auto check_point = it->second->getPoint();
        auto delta_col = static_cast<double>(cell_point.first-check_point.first);
        auto delta_row = static_cast<double>(cell_point.second-check_point.second);
        double cost = sqrt(
            static_cast<double>(delta_col*delta_col)
            + static_cast<double>(delta_row*delta_row)
        )*meta_data_->resolution;
        if(it->second->getCost() > (cell->getCost()+cost)){
            it->second->setCost(cell->getCost()+cost);
            it->second->setPreCell(cell.get());
        }
    }
    std::sort(update_cells.begin(), update_cells.end());
    auto last = std::unique(update_cells.begin(), update_cells.end());
    update_cells.erase(last, update_cells.end());
}

void Dijkstra::finishTable(const uint64_t& start_key){
    // set value of start
    auto start_cell = map_.find(start_key)->second;
    start_cell->setCost(0.0);

    std::vector<uint64_t> update_cells;
    // init algorithm
    updatePointAround(start_cell, update_cells);

    std::vector<uint64_t> new_cells;
    // stop if don't have new points list
    while (update_cells.size() != 0)
    {
        new_cells.clear();
        // update for every new point
        for(const auto& cell_key: update_cells){
            auto it = map_.find(cell_key);
            if (it != map_.end()){
                updatePointAround(it->second, new_cells);
            }
        }
        update_cells = new_cells;
    }
}

nav_msgs::msg::Path* Dijkstra::fromGoalToPath(const uint64_t& goal_key){
    // check goal is haven possible path to goal
    auto cell = map_.find(goal_key)->second.get();
    if(cell->getPreCell() == nullptr){
        return nullptr;
    }
    nav_msgs::msg::Path* path = new nav_msgs::msg::Path();
    auto pre_pose_2d = goal_;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = rclcpp::Clock().now();
    // start pose
    pose_stamped.pose = utils::fromPose2D(goal_);
    path->poses.push_back(pose_stamped);
    // link list to path
    while(cell != nullptr){
        auto pose = utils::pointIntToPose(cell->getPoint(),
            pre_pose_2d,
            *meta_data_);
        pre_pose_2d = utils::toPose2D(pose);
        pose_stamped.pose = pose;
        path->poses.push_back(pose_stamped);
        cell = cell->getPreCell();
    }
    // goal pose
    pose_stamped.pose = utils::fromPose2D(start_);
    path->poses.push_back(pose_stamped);
    path->header.frame_id = "map";
    path->header.stamp = rclcpp::Clock().now();
    return path;
}

nav_msgs::msg::Path* Dijkstra::getPath(){
    // reset all cost and link in map
    this->resetMap();
    // get start hash value
    auto [start_col, start_row] = utils::poseToPointInt(start_, *meta_data_);
    auto start_key = utils::pointIntToKey(start_row, start_col);
    auto [goal_col, goal_row] = utils::poseToPointInt(goal_, *meta_data_);
    auto goal_key = utils::pointIntToKey(goal_row, goal_col);

    // if goal or start pose not in map, return
    if(map_.find(start_key) == map_.end() || map_.find(goal_key) == map_.end())
        return nullptr;
    finishTable(start_key);
    // get goal hash value
    return fromGoalToPath(goal_key);
}

} // end namespace algorithm
} // end namespace search_example

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(search_example::algorithm::Dijkstra, 
    search_example::AlgorithmBase);