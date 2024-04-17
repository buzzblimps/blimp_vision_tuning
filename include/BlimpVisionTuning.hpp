#ifndef BLIMP_VISION_TUNING_HPP
#define BLIMP_VISION_TUNING_HPP

#include <rclcpp/rclcpp.hpp>

#include <sensor_msgs/msg/image.hpp>

#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>

//OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/ximgproc.hpp>
#include <opencv2/ximgproc/disparity_filter.hpp>

class BlimpVisionTuning : public rclcpp::Node {
private:
    rclcpp::Subscription<sensor_msgs::msg::CompressedImage>::SharedPtr comp_img_subscriber_;
    rclcpp::TimerBase::SharedPtr timer_;

    int frame_count_;

    void timer_callback();

    // image_transport::CameraSubscriber sub_camera_;
    // void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr &image_msg, const sensor_msgs::msg::CameraInfo::ConstSharedPtr &info_msg);

    void compressed_image_callback(const sensor_msgs::msg::CompressedImage::SharedPtr comp_img_msg);

public:
    BlimpVisionTuning();
};

#endif
