#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <string>
#include <iostream>
#include <stdio.h>
#include <stdarg.h>       
#include <sstream>

int main(int argc, char * argv[])
{

    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);

    // Setup RealSense Camera
    rs2::colorizer color_map;
    rs2::rates_printer printer;
    rs2::pipeline pipe;

    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    pipe.start(cfg);

    std::map<int, rs2::frame> render_frames;
    std::vector<rs2::frame> new_frames;
    //rs2::rs2_intrinsics intrinsics = pipe.get_intrinsics();
    auto const intrinsics = pipe.get_active_profile().get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
    std::cout << "Distortion Coefficients : [" << intrinsics.coeffs[0] << "," << intrinsics.coeffs[1] << "," <<
                    intrinsics.coeffs[2] << "," << intrinsics.coeffs[3] << "," << intrinsics.coeffs[4] << "]" << std::endl;

    cv::Mat image;
    int count = 0;

    // Node init
    ros::init(argc, argv, "rs_image");
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    sensor_msgs::ImagePtr msg;

    ros::Rate loop_rate(10);

    while (nh.ok()) 
    {
        rs2::frameset fs = pipe.wait_for_frames();;

        rs2::frame color_frame = fs.get_color_frame();
        cv::Mat color(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        
        cv::namedWindow("Display Image", cv::WINDOW_AUTOSIZE );
        cv::imshow("Display Image", color);

        cv::cvtColor(color, image, cv::COLOR_BGR2RGB);
        msg = cv_bridge::CvImage(std_msgs::Header(), "rgb8", image).toImageMsg();
        pub.publish(msg);

        char c = (char)cv::waitKey(25);
        if(c == 27){
            exit(0);
        }

        ros::spinOnce();
        loop_rate.sleep();

    }

    return EXIT_SUCCESS;
}