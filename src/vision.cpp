#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vision/VisionMessage.h>

int main(int argc, char **argv){

    ros::init(argc, argv, "vision_node");
    ros::NodeHandle node_handle;

    ros::Publisher publisher = node_handle.advertise<vision::VisionMessage>("vision_topic", 1);
    ros::Rate loop_rate(30);

    while (ros::ok()){
        ROS_INFO("running");
        loop_rate.sleep();
    }

    return 0;
}