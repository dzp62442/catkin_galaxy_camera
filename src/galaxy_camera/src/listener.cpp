#include "ros/ros.h"
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include "opencv2/highgui/highgui.hpp"
#include <cv_bridge/cv_bridge.h>  // opencv的mat和ros的message格式互相转化

cv::Mat imgCallback;

//回调函数定义，参数类型需要与所订阅的消息类型保持一致
void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    try
    {
        cv::imshow("view", cv_bridge::toCvShare(msg, "bgr8")->image);
        // cv::waitKey(0);  // 开启线程时，按住空格可以实施刷新图像显示
        cv::waitKey(10);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
    }
}


int main(int argc, char **argv)
{
    std::cout << "Start Listener!" << std::endl;
    ros::init(argc, argv, "listener_node");  // 初始化ros，命名节点
    ros::NodeHandle nh;
    cv::namedWindow("view");
    cv::startWindowThread();  // 开一个线程实时刷新图片显示
    image_transport::ImageTransport it(nh);
    // 使用image_transport的订阅者，而不是ros的订阅者
    image_transport::Subscriber sub = it.subscribe("/galaxy_camera_node/image_raw", 1, imageCallback);
    // ros::Subscriber sub = nh.subscribe("image_raw", 1, imageCallback);  // 定义一个订阅者
    ros::spin();  // 不返回函数，监听订阅者中的回调队列，并执行回调函数。
    cv::destroyWindow("view");
    return 0;
    
}
