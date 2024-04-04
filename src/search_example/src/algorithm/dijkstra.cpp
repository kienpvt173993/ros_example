#include "search_example/algorithm/dijkstra.hpp"
#include <limits>
#include "search_example/math_utils.hpp"

namespace search_example{
namespace algorithm{
DijkstraCell::DijkstraCell(uint32_t col, uint32_t row):
    col_(col), row_(row), prev_node_(nullptr){
    cost_ = std::numeric_limits<double>::max();
}

double DijkstraCell::getCost(){ return cost_; }

void DijkstraCell::setCost(double cost){
    cost_ = cost;
}

std::pair<uint32_t, uint32_t> DijkstraCell::getPoint(){
    return std::make_pair(col_, row_);
}

DijkstraCell* DijkstraCell::getPreNode(){ return prev_node_; }

void DijkstraCell::setPreNode(DijkstraCell* prev){
    prev_node_ = prev;
};

uint64_t DijkstraCell::getHash(){
    return pointIntToKey(row_, col_);
}

std::vector<uint64_t> DijkstraCell::getHashAround() const{
    uint64_t row = static_cast<uint64_t>(row_);
    uint64_t col = static_cast<uint64_t>(col_);
    return {
        pointIntToKey(row-1, col-1),
        pointIntToKey(row-1, col),
        pointIntToKey(row-1, col+1),
        pointIntToKey(row, col-1),
        pointIntToKey(row, col+1),
        pointIntToKey(row+1, col-1),
        pointIntToKey(row+1, col),
        pointIntToKey(row+1, col+1),
    };
}

Dijkstra::Dijkstra(){}

Dijkstra::~Dijkstra(){}

void Dijkstra::initialize(const rclcpp::Node::WeakPtr&){
}

void Dijkstra::setMap(nav_msgs::msg::OccupancyGrid::SharedPtr grid){
    for(size_t i = 0; i < grid->data.size(); i++){
        if(grid->data[i] == 0){
            auto [col, row] = indexToPoint(i, grid->info.width);
            uint64_t key = pointIntToKey(row, col);
            DijkstraCell::SharedPtr new_cell = std::make_shared<DijkstraCell>(col, row);
            this->map_.emplace(std::make_pair(key, new_cell));
        }
    }
    meta_data_ = grid->info;
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
        cell.second->setPreNode(nullptr);
    }
}

void Dijkstra::updatePointAround(const DijkstraCell::SharedPtr& cell,
    std::vector<uint64_t>& update_cells){
    auto around_cells = cell->getHashAround();
    for(const auto& cell_key: around_cells){
        auto it = map_.find(cell_key);
        if(it == map_.end()) continue;
        if(it->second->getPreNode() == nullptr)
            update_cells.push_back(it->second->getHash());
        if(it->second->getCost() > (cell->getCost()+1.0)){
            it->second->setCost(cell->getCost()+1.0);
            it->second->setPreNode(cell.get());
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

    std::vector<uint64_t> update_nodes;
    // init algorithm
    updatePointAround(start_cell, update_nodes);

    std::vector<uint64_t> new_nodes;
    // stop if don't have new points list
    while (update_nodes.size() != 0)
    {
        new_nodes.clear();
        // update for every new point
        for(const auto& node_key: update_nodes){
            auto it = map_.find(node_key);
            if (it != map_.end()){
                updatePointAround(it->second, new_nodes);
            }
        }
        update_nodes = new_nodes;
    }
}

nav_msgs::msg::Path* Dijkstra::fromGoalToPath(const uint64_t& goal_key){
    // check goal is haven possible path to goal
    auto node = map_.find(goal_key)->second.get();
    if(node->getPreNode() == nullptr){
        return nullptr;
    }
    nav_msgs::msg::Path* path = new nav_msgs::msg::Path();
    auto pre_pose_2d = goal_;
    geometry_msgs::msg::PoseStamped pose_stamped;
    pose_stamped.header.frame_id = "map";
    pose_stamped.header.stamp = rclcpp::Clock().now();
    // start pose
    pose_stamped.pose = fromPose2D(goal_);
    path->poses.push_back(pose_stamped);
    // link list to path
    while(node != nullptr){
        auto pose = cellToPose(node->getPoint(),
            pre_pose_2d,
            meta_data_);
        pre_pose_2d = toPose2D(pose);
        pose_stamped.pose = pose;
        path->poses.push_back(pose_stamped);
        node = node->getPreNode();
    }
    // goal pose
    pose_stamped.pose = fromPose2D(start_);
    path->poses.push_back(pose_stamped);
    path->header.frame_id = "map";
    path->header.stamp = rclcpp::Clock().now();
    return path;
}

nav_msgs::msg::Path* Dijkstra::getPath(){
    // reset all cost and link in map
    this->resetMap();
    // get start hash value
    auto [start_col, start_row] = poseToCell(start_, meta_data_);
    auto start_key = pointIntToKey(start_row, start_col);
    auto [goal_col, goal_row] = poseToCell(goal_, meta_data_);
    auto goal_key = pointIntToKey(goal_row, goal_col);

    // if goal or start pose not in map, return
    if(map_.find(start_key) == map_.end() || map_.find(goal_key) == map_.end())
        return nullptr;
    finishTable(start_key);
    // get goal hash value
    return fromGoalToPath(goal_key);
}

}
}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(search_example::algorithm::Dijkstra, 
    search_example::AlgorithmBase);