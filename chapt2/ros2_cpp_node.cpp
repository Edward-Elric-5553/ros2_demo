#include "iostream"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[]){
    std::cout << "parameter cout " << argc << std::endl;
    std::cout << "program Name " << argv[0] << std::endl;

    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("cpp_node");

    RCLCPP_INFO(node->get_logger(), "Hello CPP Node");

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}