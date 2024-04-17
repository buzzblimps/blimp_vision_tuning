#include "BlimpVisionTuning.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

BlimpVisionTuning::BlimpVisionTuning() : Node("blimp_vision_tuning_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing Blimp Vision Tuning Node");

    // sub_camera_ = image_transport::create_camera_subscription(this, "image_raw", std::bind(&BlimpVisionTuning::image_callback, this, std::placeholders::_1, std::placeholders::_2), "raw");
    comp_img_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 1, std::bind(&BlimpVisionTuning::compressed_image_callback, this, _1));

    timer_ = this->create_wall_timer(1000ms, std::bind(&BlimpVisionTuning::timer_callback, this));
}

void BlimpVisionTuning::compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr comp_img_msg) {
    // RCLCPP_INFO(this->get_logger(), "Got image");
    // cv::Mat hsv, mask, mask1, mask2, mask_orange, mask_yellow;

    frame_count_++;
    
    cv::Mat sync_frame = cv::imdecode(cv::Mat(comp_img_msg->data), 1);

    // Crop the left and right images
    cv::Rect left_roi(0, 0, sync_frame.cols/2, sync_frame.rows);
    cv::Rect right_roi(sync_frame.cols/2, 0, sync_frame.cols/2, sync_frame.rows);
    cv::Mat left_frame(sync_frame, left_roi);
    cv::Mat right_frame(sync_frame, right_roi);

    cv::imshow("Left", sync_frame);
    cv::waitKey(1);
}

// void BlimpVisionTuning::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg) {

//     std::cout << image_msg->width << "x" << image_msg->height << ", len=" << image_msg->data.size() << std::endl;

//     // for (int i = 0; i < image_msg->data.size(); i++) {
//     //     std::cout << std::to_string(image_msg->data[i]) << " ";
//     // }
//     // std::cout << std::endl;

//     if (image_msg->width == 0 || image_msg->height == 0) {
//         //Empty image - return;
//         return;
//     }

//     // cv::Mat sync_frame = cv_bridge::toCvShare(image_msg, "bgr8")->image;

//     cv::imshow("view", cv_bridge::toCvCopy(image_msg, "bgr8")->image);

//     //Todo: Apply stereo image rectification to both images
//     // cv::Mat left_cal = left_frame;

//     // cv::imshow("Sync", sync_frame);
//     cv::waitKey(1);

//     frame_count_++;
// }

void BlimpVisionTuning::timer_callback() {
    RCLCPP_INFO(this->get_logger(), "%d frames/second", frame_count_);
    frame_count_ = 0;
}