#include "BlimpVisionTuning.hpp"

int main(int argc, char ** argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<BlimpVisionTuning>());
    rclcpp::shutdown();
    return 0;
}

