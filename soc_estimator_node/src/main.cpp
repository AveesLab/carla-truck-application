#include "rclcpp/rclcpp.hpp"
#include "soc_estimator_node.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SOCEstimatorNode>());
    rclcpp::shutdown();
    return 0;
}

