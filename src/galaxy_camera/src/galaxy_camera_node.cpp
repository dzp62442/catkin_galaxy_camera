//
// Created by qiayuan on 6/27/20.
//

#include "galaxy_camera_node.h"

namespace galaxy_camera {

    GalaxyCameraNode::GalaxyCameraNode() : nh_("~") {
        /*
        image_transport发布和订阅独特的ROS Topic为每一个可用的Transport。
        与ROS Publisher不同，它并非注册一个单独的Topic，而是多个Topic，Topic命名遵循一个标准规则。
        在接口中，我们只能指定Base Topic名，其它具体名字由规则指定，例如"/image_raw/compressed"
        */
        image_transport::ImageTransport it(nh_);
        /*
        发布名为"image_raw"的话题，消息数据类型为sensor_msgs/Image
        第二个参数是发布序列的大小，如果发布消息的频率太高，缓冲区中的消息在超过序列大小的时候就会开始丢弃先前发布的消息
        */
        image_pub_ = it.advertiseCamera("image_raw", 1);

        // 加载相机信息
        std::string camera_name = "MER_200_20GC";  // 相机型号
        std::string device_ip = "192.168.29.1";  // 设备IP 
        nh_.param("camera_frame_id", galaxy_camera::GalaxyCamera::image_.header.frame_id, camera_name);
        nh_.param("camera_name", camera_name_, camera_name);
        nh_.param("camera_info_url", camera_info_url_, std::string(""));
        nh_.param("image_width", image_width_, 1280);
        nh_.param("image_height", image_height_, 1024);
        nh_.param("image_offset_x", image_offset_x_, 0);
        nh_.param("image_offset_y", image_offset_y_, 0);
        nh_.param("pixel_format", pixel_format_, std::string("bgr8"));
        nh_.param("device_ip", device_ip_, device_ip);
        ROS_INFO("My IP is  %s", device_ip_.c_str());
        info_.reset(new camera_info_manager::CameraInfoManager(nh_, camera_name_, camera_info_url_));
        // check for default camera info
        if (!info_->isCalibrated()) {
            info_->setCameraName(camera_name_);
            sensor_msgs::CameraInfo camera_info;
            camera_info.header.frame_id = galaxy_camera::GalaxyCamera::image_.header.frame_id;
            camera_info.width = image_width_;
            camera_info.height = image_height_;
            info_->setCameraInfo(camera_info);
        }
        ROS_INFO("Starting '%s' at %dx%d", camera_name_.c_str(),
                 image_width_, image_height_);
        galaxy_camera_ = new GalaxyCamera(
                nh_, image_pub_, info_->getCameraInfo(),
                image_height_, image_width_, image_width_ * 3,
                image_offset_x_, image_offset_y_, pixel_format_,device_ip_);
    }

    GalaxyCameraNode::~GalaxyCameraNode() {
        delete galaxy_camera_;
    }

}


// main函数
int main(int argc, char **argv) {
    ros::init(argc, argv, "galaxy_camera_node");  // 初始化节点，定义节点名称
    ros::NodeHandle nh("~");
    galaxy_camera::GalaxyCameraNode galaxy_camera_node;
    ros::spin();
}