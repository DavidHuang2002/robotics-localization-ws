#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/camera_info.hpp"
#include "std_msgs/msg/bool.hpp"


using std::placeholders::_1;

class camPublisher : public rclcpp::Node {
public:
    camPublisher() : Node("cam_publisher"),
                     baseLinkFlipped(false)
    {
        colorImageSub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 1, std::bind(&camPublisher::colorImageCallBack, this, _1)
        );

        infoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 1, std::bind(&camPublisher::colorInfoCallBack, this, _1)
        );

        depthImageSub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw", 1, std::bind(&camPublisher::depthImageCallBack, this, _1)
        );

        flipBaseLinkSub = this->create_subscription<std_msgs::msg::Bool>(
            "/base_link_flipped", 1, std::bind(&camPublisher::flipBaseLinkCallback, this, _1)
        );

        colorImageSub2 = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera2/color/image_raw", 1, std::bind(&camPublisher::colorImageCallBack2, this, _1)
        );

        infoSub2 = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera2/color/camera_info", 1, std::bind(&camPublisher::colorInfoCallBack2, this, _1)
        );

        depthImageSub2 = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera2/depth/image_rect_raw", 1, std::bind(&camPublisher::depthImageCallBack2, this, _1)
        );



        colorImagePub = this->create_publisher<sensor_msgs::msg::Image>("/cam/color/image", 1);
        infoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/cam/info", 1);
        depthImagePub = this->create_publisher<sensor_msgs::msg::Image>("/cam/depth/image", 1);

        colorImagePub2 = this->create_publisher<sensor_msgs::msg::Image>("/cam/color/image", 1);
        infoPub2 = this->create_publisher<sensor_msgs::msg::CameraInfo>("/cam/info", 1);
        depthImagePub2 = this->create_publisher<sensor_msgs::msg::Image>("/cam/depth/image", 1);
    }
private:

    bool baseLinkFlipped;

    void colorImageCallBack(sensor_msgs::msg::Image::ConstSharedPtr msg){
        // auto image = sensor_msgs::msg::Image();
        // image.data = msg->data;
        // image.header = msg->header;
        // image.encoding = msg->encoding;
        // image.height = msg->height;
        // image.width = msg->width;
        // image.step = msg->step;
        // image.is_bigendian = msg->is_bigendian;

        if(!baseLinkFlipped){
            colorImagePub -> publish(*msg);
        }
    };

    void colorInfoCallBack(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){
        // auto info = sensor_msgs::msg::CameraInfo();
        // info.binning_x = msg->binning_x;
        // info.binning_y = msg->binning_y;
        // info.d = msg->d;
        // info.distortion_model = msg->distortion_model;
        // info.header = msg->header;
        // info.height = msg->height;
        // info.width = msg->width;
        // info.k = msg->k;
        // info.p = msg->p;
        // info.r = msg->r;
        // info.roi = msg->roi;
        
        if(!baseLinkFlipped){
            infoPub -> publish(*msg);
        }
    };

    void depthImageCallBack(sensor_msgs::msg::Image::ConstSharedPtr msg){
        // auto image = sensor_msgs::msg::Image();
        // image.data = msg->data;
        // image.header = msg->header;
        // image.encoding = msg->encoding;
        // image.height = msg->height;
        // image.width = msg->width;
        // image.step = msg->step;
        // image.is_bigendian = msg->is_bigendian;

        if(!baseLinkFlipped){
            depthImagePub -> publish(*msg);
        }
    };

    void colorImageCallBack2(sensor_msgs::msg::Image::ConstSharedPtr msg){
       // auto image = *msg; //sensor_msgs::msg::Image();
        // image.data = msg->data;
        // image.header = msg->header;
        // image.encoding = msg->encoding;
        // image.height = msg->height;
        // image.width = msg->width;
        // image.step = msg->step;
        // image.is_bigendian = msg->is_bigendian;

        if(baseLinkFlipped){
            colorImagePub2 -> publish(*msg);
        }
    };

    void depthImageCallBack2(sensor_msgs::msg::Image::ConstSharedPtr msg){
        //auto image = *msg; //sensor_msgs::msg::Image();
        // image.data = msg->data;
        // image.header = msg->header;
        // image.encoding = msg->encoding;
        // image.height = msg->height;
        // image.width = msg->width;
        // image.step = msg->step;
        // image.is_bigendian = msg->is_bigendian;

        if(baseLinkFlipped){
            colorImagePub2 -> publish(*msg);
        }
    };

    void colorInfoCallBack2(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){
        //auto info = *msg; //sensor_msgs::msg::CameraInfo();
        // info.binning_x = msg->binning_x;
        // info.binning_y = msg->binning_y;
        // info.d = msg->d;
        // info.distortion_model = msg->distortion_model;
        // info.header = msg->header;
        // info.height = msg->height;
        // info.width = msg->width;
        // info.k = msg->k;
        // info.p = msg->p;
        // info.r = msg->r;
        // info.roi = msg->roi;
        
        if(baseLinkFlipped){
            infoPub2 -> publish(*msg);
        }
    };

    void flipBaseLinkCallback(std_msgs::msg::Bool::ConstSharedPtr msg) {
        baseLinkFlipped = msg->data;
    };


    //Subcribers

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorImageSub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flipBaseLinkSub;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorImageSub2;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSub2;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub2;

    //Publishers

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorImagePub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthImagePub;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorImagePub2;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub2;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthImagePub2;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<camPublisher>());

    rclcpp::shutdown();
    return 0;
}