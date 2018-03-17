#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vision/VisionMessage.h>

int main(int argc, char **argv){

    if (argc < 2){
        ROS_ERROR("Missing Arguments: aborting");
        exit(1);
    }

    cv::VideoCapture raw_input(argv[1]);
    while (!raw_input.isOpened()) { //check if video device has been initialised
        ROS_INFO("cannot open camera");
    }

    raw_input.open(argv[1]);
    ROS_INFO("%d",raw_input.isOpened());
    cv::namedWindow("raw_input");

    ros::init(argc, argv, "vision_node");
    ros::NodeHandle node_handle;

    ros::Publisher publisher = node_handle.advertise<vision::VisionMessage>("vision_topic", 1);
    ros::Rate loop_rate(30);

    while (ros::ok()){
        
        cv::Mat frame;

        raw_input >> frame;
        cv::imshow("raw_input", frame);

        loop_rate.sleep();
    }


    return 0;
}