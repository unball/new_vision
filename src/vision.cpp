#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <vision/VisionMessage.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

cv::Mat frame;

void save_frame(const sensor_msgs::ImageConstPtr& msg){
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::imshow("raw_input", cv_ptr->image);

}

int main(int argc, char **argv){

    if (argc < 2){
        ROS_ERROR("Missing Arguments: aborting");
        exit(1);
    }

    ros::init(argc, argv, "vision_node");
    ros::NodeHandle node_handle;
    ros::Subscriber sub = node_handle.subscribe("image_raw", 0, save_frame);
    ros::Publisher publisher = node_handle.advertise<vision::VisionMessage>("vision_topic", 1);
    ros::Rate loop_rate(30);

    while (ros::ok()){
        
        loop_rate.sleep();
    }


    return 0;
}