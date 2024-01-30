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

        flipBaseLinkSub = this->create_subscription<std_msgs::msg::Bool>(
            "/base_link_flipped", 20, std::bind(&camPublisher::flipBaseLinkCallback, this, _1)
        );

        colorImageSub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/color/image_raw", 20, std::bind(&camPublisher::colorImageCallBack, this, _1)
        );

        infoSub = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera/color/camera_info", 20, std::bind(&camPublisher::colorInfoCallBack, this, _1)
        );

        depthImageSub = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera/depth/image_rect_raw", 20, std::bind(&camPublisher::depthImageCallBack, this, _1)
        );

        colorImageSub2 = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera2/color/image_raw", 20, std::bind(&camPublisher::colorImageCallBack2, this, _1)
        );

        infoSub2 = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera2/color/camera_info", 20, std::bind(&camPublisher::colorInfoCallBack2, this, _1)
        );

        depthImageSub2 = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera2/depth/image_rect_raw", 20, std::bind(&camPublisher::depthImageCallBack2, this, _1)
        );


        colorImagePub = this->create_publisher<sensor_msgs::msg::Image>("/cam/color/image", 20);
        infoPub = this->create_publisher<sensor_msgs::msg::CameraInfo>("/cam/info", 20);
        depthImagePub = this->create_publisher<sensor_msgs::msg::Image>("/cam/depth/image", 20);
    }
private:

    bool baseLinkFlipped;

    void flipBaseLinkCallback(std_msgs::msg::Bool::ConstSharedPtr msg) {
        baseLinkFlipped = msg->data;
    };

    void colorImageCallBack(sensor_msgs::msg::Image::ConstSharedPtr msg){
        if(!baseLinkFlipped){
            colorImagePub -> publish(*msg);
            // RCLCPP_INFO(rclcpp::get_logger("camPublisher"), "0");
        }
    };

    void colorInfoCallBack(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){
        if(!baseLinkFlipped){
            infoPub -> publish(*msg);
            // RCLCPP_INFO(rclcpp::get_logger("camPublisher"), "0");
        }
    };

    void depthImageCallBack(sensor_msgs::msg::Image::ConstSharedPtr msg){
        if(!baseLinkFlipped){
            depthImagePub -> publish(*msg);
            // RCLCPP_INFO(rclcpp::get_logger("camPublisher"), "0");
        }
    };

    void colorImageCallBack2(sensor_msgs::msg::Image::ConstSharedPtr msg){
        if(baseLinkFlipped){
            colorImagePub -> publish(*msg);
            // RCLCPP_INFO(rclcpp::get_logger("camPublisher"), "1");
        }
    };

    void depthImageCallBack2(sensor_msgs::msg::Image::ConstSharedPtr msg){
        if(baseLinkFlipped){
            depthImagePub -> publish(*msg);
            // RCLCPP_INFO(rclcpp::get_logger("camPublisher"), "1");
        }
    };

    void colorInfoCallBack2(sensor_msgs::msg::CameraInfo::ConstSharedPtr msg){
        if(baseLinkFlipped){
            infoPub -> publish(*msg);
            // RCLCPP_INFO(rclcpp::get_logger("camPublisher"), "1");
        }
    };

    


    //Subcribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flipBaseLinkSub;

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorImageSub;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSub;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub;
   

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr colorImageSub2;
    rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr infoSub2;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr depthImageSub2;

    //Publishers

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr colorImagePub;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr infoPub;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr depthImagePub;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<camPublisher>());

    rclcpp::shutdown();
    return 0;
}