#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include "opencv2/imgcodecs.hpp"
#include "opencv2/highgui.hpp"
#include "opencv2/imgproc.hpp"
#include <opencv2/ximgproc.hpp>
#include <cv_bridge/cv_bridge.h>



class HelperFLS
{
public:
    HelperFLS() {
        sub = nh.subscribe<sensor_msgs::Image> ("/fls_in", 1, &HelperFLS::callback, this);
        pub = nh.advertise<sensor_msgs::Image>("/fls_out",1);
    }

    ~HelperFLS() {}

    void callback(const sensor_msgs::Image::ConstPtr& msg);

private:
    ros::NodeHandle nh;

    ros::Subscriber sub;
    ros::Publisher pub;
};

void HelperFLS::callback(const sensor_msgs::Image::ConstPtr& msg){
    // ==================== receive image ==================== //
    
    cv_bridge::CvImagePtr cv_ptr;
    try{
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_8UC1); // bgr8, MONO8, TYPE_8UC1
        // cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8); // bgr8, MONO8, TYPE_8UC1
        // printf("raw img size: rows=%d, cols=%d\n", cv_ptr->image.rows, cv_ptr->image.cols);
    }
    catch (cv_bridge::Exception& e){
        ROS_ERROR("cv_bridge exception: %s", e.what());
        std::exit(EXIT_FAILURE);
    }

    // ==================== create the map ==================== //
    // cv::Mat map_bb_x = cv::Mat::zeros(cv::Size(512, 667), CV_32FC1);
    cv::Mat map_bb_x = cv::Mat::zeros(cv::Size(cv_ptr->image.cols, cv_ptr->image.rows), CV_32FC1);
    cv::Mat map_bb_y = cv::Mat::zeros(cv::Size(cv_ptr->image.cols, cv_ptr->image.rows), CV_32FC1);
    for( int i = 0; i < map_bb_x.rows; i++ )
    {
        for( int j = 0; j < map_bb_x.cols; j++ )
        {
            map_bb_x.at<float>(i, j) = (float)(j); 
            map_bb_y.at<float>(i, j) = (float)(map_bb_x.rows - i);
        }
    }  

    // ==================== remap the image ==================== //
    cv::Mat img_remap;
    cv::remap( cv_ptr->image, img_remap, map_bb_x, map_bb_y, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0) );

    // ==================== remap the image ==================== //
    sensor_msgs::ImagePtr msg_out = cv_bridge::CvImage(msg->header, "mono8", img_remap).toImageMsg();
    pub.publish(msg_out);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "Helper_FLS_Node");

  HelperFLS  example;

  ros::spin();
  return 0;
}