#include "BlimpVisionTuning.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;
using namespace cv;

    Mat imgBlur, imgHSV, mask;
    int mode = 5;
    int hmin[5] = {0,0,0,0,0}, smin[5] = {0,0,0,0,0}, vmin[5] = {0,0,0,0,0};
    int hmax[5] = {179,179,179,179,179}, smax[5] = {255,255,255,255,255}, vmax[5] = {255,255,255,255,255};
    string color_name[4] = {"Green", "Purple", "Orange", "Yellow"};
    int thresh[5] = {25,25,25,25,25};
    int kernel_size[5] = {5,5,5,5,5};

    bool mouseClicked = false;
    int imgX = 0;
    int imgY = 0;
    Scalar lower(hmin[mode], smin[mode], vmin[mode]);
	Scalar upper(hmax[mode], smax[mode], vmax[mode]);

	Vec3b pixelValue;
    string trackbarWinName = "Trackbars";

BlimpVisionTuning::BlimpVisionTuning() : Node("blimp_vision_tuning_node") {
    RCLCPP_INFO(this->get_logger(), "Initializing Blimp Vision Tuning Node");

    initColorDetection();

    // sub_camera_ = image_transport::create_camera_subscription(this, "image_raw", std::bind(&BlimpVisionTuning::image_callback, this, std::placeholders::_1, std::placeholders::_2), "raw");
    comp_img_subscriber_ = this->create_subscription<sensor_msgs::msg::CompressedImage>("image_raw/compressed", 1, std::bind(&BlimpVisionTuning::compressed_image_callback, this, _1));


    //computerVision.init();
    timer_ = this->create_wall_timer(1000ms, std::bind(&BlimpVisionTuning::timer_callback, this));
}

void BlimpVisionTuning::CallBackFunc(int event, int x, int y, int flags, void* userdata)
{
     if  (event == EVENT_LBUTTONDOWN)
     {
          cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
          mouseClicked = true;
          imgX = y;
          imgY = x;
     } else if (event == EVENT_RBUTTONDOWN) {
        cout << "Right button is down, exporting" << endl;
        std::ofstream yaml_file("color_detection.yaml");
        if (yaml_file.is_open()) {
            for (int i = 0; i < 4; i++) {
                yaml_file << color_name[i] << "_Min: [" << hmin[i] << "," << smin[i] << "," << vmin[i] << "]" << endl; 
                yaml_file << color_name[i] << "_Max: [" << hmax[i] << "," << smax[i] << "," << vmax[i] << "]" << endl; 
                yaml_file << color_name[i] << "_Thresh: " << thresh[i] << endl;
                yaml_file << color_name[i] << "_Kernel_Size: " << kernel_size[i] << endl;  
            }
        }
     }
//     else if  ( event == EVENT_RBUTTONDOWN )
//     {
//          cout << "Right button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//     }
//     else if  ( event == EVENT_MBUTTONDOWN )
//     {
//          cout << "Middle button of the mouse is clicked - position (" << x << ", " << y << ")" << endl;
//     }
//     else if ( event == EVENT_MOUSEMOVE )
//     {
//          cout << "Mouse move over the window - position (" << x << ", " << y << ")" << endl;
//     }
}

void BlimpVisionTuning::initColorDetection() {
    namedWindow("Image", (640, 480));
    namedWindow("Image Blur", (640, 480));
    namedWindow("Image Mask", (640, 480));
    namedWindow("Trackbars", (640, 200));
    
    setMouseCallback("Image Blur", [](int event, int x, int y, int flags, void* userdata) {
    static_cast<BlimpVisionTuning*>(userdata)->CallBackFunc(event, x, y, flags, userdata);
}, NULL);

    
    createTrackbar("Threshold", trackbarWinName, &thresh[mode], 255, [](int, void*) {
    thresh[mode] = getTrackbarPos("Threshold", trackbarWinName);
});
    createTrackbar("Hue Min",   trackbarWinName, &hmin[mode], 179, [](int, void*) {
    hmin[mode] = getTrackbarPos("Hue Min", trackbarWinName);
});
    createTrackbar("Hue Max",   trackbarWinName, &hmax[mode], 179, [](int, void*) {
    hmax[mode] = getTrackbarPos("Hue Max", trackbarWinName);
});
    createTrackbar("Sat Min",   trackbarWinName, &smin[mode], 255, [](int, void*) {
    smin[mode] = getTrackbarPos("Sat Min", trackbarWinName);
});
    createTrackbar("Sat Max",   trackbarWinName, &smax[mode], 255, [](int, void*) {
    smax[mode] = getTrackbarPos("Sat Max", trackbarWinName);
});
    createTrackbar("Val Min",   trackbarWinName, &vmin[mode], 255, [](int, void*) {
    vmin[mode] = getTrackbarPos("Val Min", trackbarWinName);
});
    createTrackbar("Val Max",   trackbarWinName, &vmax[mode], 255, [](int, void*) {
    vmax[mode] = getTrackbarPos("Val Max", trackbarWinName);
});
    createTrackbar("Kernel Size", trackbarWinName, &kernel_size[mode], 15, [](int, void*) {
    kernel_size[mode] = getTrackbarPos("Kernel Size", trackbarWinName);
});


    lower = Scalar(hmin[mode], smin[mode], vmin[mode]);
	upper = Scalar(hmax[mode], smax[mode], vmax[mode]);
    RCLCPP_INFO(this->get_logger(), "Initializing color detection");
    mode = 0;
    createTrackbar("Color Mode", trackbarWinName, &mode, 3, [](int, void*) {
    // Update trackbar values based on the selected mode
    setTrackbarPos("Threshold", trackbarWinName, thresh[mode]);
    setTrackbarPos("Hue Min", trackbarWinName, hmin[mode]);
    setTrackbarPos("Hue Max", trackbarWinName, hmax[mode]);
    setTrackbarPos("Sat Min", trackbarWinName, smin[mode]);
    setTrackbarPos("Sat Max", trackbarWinName, smax[mode]);
    setTrackbarPos("Val Min", trackbarWinName, vmin[mode]);
    setTrackbarPos("Val Max", trackbarWinName, vmax[mode]);
    setTrackbarPos("Kernel Size", trackbarWinName, kernel_size[mode]);
});
    setTrackbarPos("Threshold", trackbarWinName, thresh[mode]);
    setTrackbarPos("Hue Min", trackbarWinName, hmin[mode]);
    setTrackbarPos("Hue Max", trackbarWinName, hmax[mode]);
    setTrackbarPos("Sat Min", trackbarWinName, smin[mode]);
    setTrackbarPos("Sat Max", trackbarWinName, smax[mode]);
    setTrackbarPos("Val Min", trackbarWinName, vmin[mode]);
    setTrackbarPos("Val Max", trackbarWinName, vmax[mode]);
    setTrackbarPos("Kernel Size", trackbarWinName, kernel_size[mode]);
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

    cv::imshow("Sync", sync_frame);

    int key = cv::waitKey(1);

    // if(key == 27){
    //     // Escape key
    //     rclcpp::shutdown();
    // }
    // RCLCPP_INFO(this->get_logger(), "key: %d", key);

    //computer vision stuff?
    autoState mode = searching;
    goalType goalColor = orange;
    computerVision.update(left_frame, right_frame, mode, goalColor);

    std::vector<std::vector<float> > target;
    target = computerVision.getTargetBalloon();
    RCLCPP_INFO(this->get_logger(), "size: %d", int(target.size()));

    //Open webcam

//	string path = "/home/corelab/opencv-workspace/Resources/fpvBalloon.png";
//	Mat img = imread(path);//



		lower = Scalar(hmin[mode], smin[mode], vmin[mode]);
		upper = Scalar(hmax[mode], smax[mode], vmax[mode]);

//		cout << "Threshold: " << thresh << endl;

		// cap.read(sync_frame);
//		GaussianBlur(img, imgBlur, Size(15,15), 0);
		cvtColor(sync_frame, imgHSV, COLOR_BGR2HSV);

		if (mouseClicked) {
			pixelValue = imgHSV.at<Vec3b>(imgX, imgY);
			cout << "HSV: " << (unsigned int)pixelValue[0] << ", " << (unsigned int)pixelValue[1] << ", " << (unsigned int)pixelValue[2] << endl;

			hmin[mode] = max(0, pixelValue[0] - thresh[mode]);
			hmax[mode] = min(255, pixelValue[0] + thresh[mode]);

			smin[mode] = max(0, pixelValue[1] - thresh[mode]);
			smax[mode] = min(255, pixelValue[1] + thresh[mode]);

			vmin[mode] = max(0, pixelValue[2] - thresh[mode]);
			vmax[mode] = min(255, pixelValue[2] + thresh[mode]);


            setTrackbarPos("Color Mode", trackbarWinName, mode);
            setTrackbarPos("Kernel Size", trackbarWinName, kernel_size[mode]);
			setTrackbarPos("Hue Min", trackbarWinName, hmin[mode]);
			setTrackbarPos("Hue Max", trackbarWinName, hmax[mode]);
			setTrackbarPos("Sat Min", trackbarWinName, smin[mode]);
			setTrackbarPos("Sat Max", trackbarWinName, smax[mode]);
			setTrackbarPos("Val Min", trackbarWinName, vmin[mode]);
			setTrackbarPos("Val Max", trackbarWinName, vmax[mode]);
//			cout << "hmin: " << hmin << ", hmax: " << hmax << endl;
//			cout << "smin: " << smin << ", smax: " << smax << endl;
//			cout << "vmin: " << vmin << ", vmax: " << vmax << endl;

			mouseClicked = false;
		}



		inRange(imgHSV, lower, upper, mask);

//		imshow("Image", img);
//		imshow("Image HSV", imgHSV);
		imshow("Image Blur", sync_frame);

		resizeWindow("Image Blur", 1280, 720);

		imshow("Image Mask", mask);
		resizeWindow("Image Mask", 1280, 720);
        if (kernel_size[mode] > 0) {
            Mat kernel = getStructuringElement(MORPH_ELLIPSE, Size(kernel_size[mode], kernel_size[mode]));
            Mat bMask_cleaned;
            morphologyEx(mask, bMask_cleaned, MORPH_CLOSE, kernel);
            morphologyEx(bMask_cleaned, bMask_cleaned, MORPH_OPEN, kernel);
            imshow("Mask after morphology", bMask_cleaned);
        }
        
		// waitKey(1);
	

}

void BlimpVisionTuning::timer_callback() {

    RCLCPP_INFO(this->get_logger(), "%d frames/second", frame_count_);
    frame_count_ = 0;
}