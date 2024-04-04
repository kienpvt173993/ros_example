#include "search_example/data_interface.hpp"

int main(int argc, char **argv){
    rclcpp::init(argc, argv);
    std::shared_ptr<search_example::DataInterface> node = 
        std::make_shared<search_example::DataInterface>();
    node->start();
    rclcpp::spin(node);
    rclcpp::shutdown();
}