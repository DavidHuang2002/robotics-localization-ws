#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/bool.hpp"
#include <iostream>
#include <vector>
#include <random>
#include <tf2_ros/buffer.h>
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/multi_array_dimension.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include <librealsense2/rs.hpp>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.hpp>
#include "sensor_msgs/msg/camera_info.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <opencv2/core.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/opencv.hpp>
#include <boost/shared_ptr.hpp>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>

using namespace cv;
using std::placeholders::_1;

class CustomNode : public rclcpp::Node
{
    // You can ignore the red squiggles, you can build and run this confirmed
    public:
        CustomNode(): Node("my_node") {
            RCLCPP_INFO(this->get_logger(), "TEST Cpp node");
            
            camera_info_subscription_ = this->create_subscription<sensor_msgs::msg::CameraInfo>(
            "/camera2/color/camera_info", 10, std::bind(&CustomNode::camera_info_callback, this, _1));

            RCLCPP_INFO(this->get_logger(), "/camera2/color/camera_info found");

            lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&CustomNode::lidar_callback, this, _1));
            
            camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "/camera2/color/image_raw", 10, std::bind(&CustomNode::camera_callback, this, _1));

            flipBaseLink = this->create_subscription<std_msgs::msg::Bool>(
            "/base_link_flipped", 1, std::bind(&CustomNode::flipBaseLinkCallback, this, _1)
            );
            
            RCLCPP_INFO(this->get_logger(), "/camera2/color/image_raw found");

            publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "realsense_cv2", 10);

            line_image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "line_image", 10);

            dot_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "dot_image", 10);

            test_board_pixel_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("test_board_pixel",10);

            lidar_line_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
                "lidar_line_image", 10);

            found_board_publisher_ = this->create_publisher<std_msgs::msg::Bool> (
                "/found_board", 10);

            boardIntersection_ = this->create_publisher<std_msgs::msg::Float32MultiArray>("/boardIntersection", 5);

            theta_publisher_ = this->create_publisher<std_msgs::msg::Float32>("/board_theta", 10);


            broadcaster = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            // staticbroadcaster = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
            camera_board_corners = Eigen::MatrixXf::Zero(3,4);
            extrinsic_matrix = Eigen::MatrixXf::Identity(4,4);
            camera_to_lidar_tf = Eigen::MatrixXf::Zero(4,4);
            scan_to_base_link_tf1 = Eigen::MatrixXf::Zero(4,4);
            scan_to_base_link_tf2 = Eigen::MatrixXf::Zero(4,4);
            
            //matrix represents lidar W.R.T camera frame
            camera_to_lidar_tf << 0, -1, 0, 0.05, //y translation
                                  0, 0, -1, 0.0525, //z translation
                                  1, 0, 0, -0.062, //x translation
                                  0, 0, 0, 1;

            // matrix represents scan to base link static transform
            scan_to_base_link_tf2 << 1, 0, 0, -0.038,
                                    0, 1, 0, 0.178,
                                    0, 0, 1, -0.787,
                                    0, 0, 0, 1; //Nav backward

            scan_to_base_link_tf1 << -1, 0, 0, -0.038,
                                    0, -1, 0, 0.178,
                                    0, 0, 1, -0.787,
                                    0, 0, 0, 1; //Nav forward 

            /*r_matrix <<  1.032889E-10,0,0,
                        0,0,0,
                        0,0,0;
            prior << 0,0,0;
            covariance_prior << 1.032889E-10,0,0,
                                0,0,0,
                                0,0,0;
            q_matrix << 0.0002f,0,0,
                        0,0.0002f,0,
                        0,0,0.0002f;*/

            RCLCPP_INFO(this->get_logger(), "Translations posted");
            /*
            Paper Parameters
            0, -1, 0, 0.10795, // 0.01905
            0, 0, -1, -0.047625, //-0.18. about -0.16 / Changed from 0.10795 , 0889
            1, 0, 0, 0.0,
            0, 0, 0, 1;
            */
            intrinsic_matrix = Eigen::MatrixXf::Zero(3,4); // Actually 3,4 but resize so it shouldn't matter
            board_pixel_line = Eigen::MatrixXf::Zero(3,1);
            board_lidar_line = Eigen::MatrixXf::Zero(3,1);
            board_center_pixel_line = Eigen::MatrixXf::Zero(3,1);
            pixel_to_lidar_matrix = Eigen::MatrixXf::Zero(3,4);
            camera_center = Eigen::MatrixXf::Zero(4,1);
            center_pixel = Eigen::MatrixXf::Zero(3,1);
            center_point = Eigen::MatrixXf::Zero(3,1);
            
            // float ta[8] = {1.0,10.0,0.0,0.0,1.0,0.0,0.0,1.0};
            // this->make_transforms(); // 1.61 - 1.71, Left:  1.34 - 1.4
    };  


    private:
        Eigen::MatrixXf camera_board_contour;
        Eigen::MatrixXf camera_blue_contour;
        Eigen::MatrixXf lidar_points; // Nx4
        Eigen::MatrixXf camera_board_corners;
        Eigen::MatrixXf lidar_pixel_points;
        Eigen::MatrixXf pixel_board_points;
        Eigen::MatrixXf lidar_board_points;
        Eigen::MatrixXf extrinsic_matrix;
        Eigen::MatrixXf camera_to_lidar_tf;
        Eigen::MatrixXf intrinsic_matrix; // 3x4
        Eigen::MatrixXf board_pixel_line; // 3x1
        Eigen::MatrixXf board_lidar_line; // 3x1
        Eigen::MatrixXf board_center_pixel_line;
        Eigen::MatrixXf pixel_to_lidar_matrix; // 3x4
        Eigen::MatrixXf camera_center;
        Eigen::MatrixXf center_pixel;
        Eigen::MatrixXf center_point;
        Eigen::MatrixXf filtered_image;
        Eigen::MatrixXf scan_to_base_link_tf1;
        Eigen::MatrixXf scan_to_base_link_tf2;
        Eigen::MatrixXf ekf_values = Eigen::MatrixXf::Zero(1000, 3);
        int count = 0;
        cv::Mat image_msg;
        bool baseLinkFlipped = false;
        float z_theta;
        /*Eigen::Vector3f prior;
        Eigen::Vector3f current_state;
        Eigen::Matrix3f covariance_prior;
        Eigen::Matrix3f current_covariance;
        Eigen::Matrix3f q_matrix;
        Eigen::Matrix3f r_matrix;
        bool got_prior = false;*/
        cv::Mat colored_image;
        std::vector<cv::Point> first_contour;
        std::vector<cv::Point> second_contour;
        std::vector<cv::Point> blue_max_contour;
        Scalar pink_lower = Scalar(100, 120, 120);
        Scalar pink_upper = Scalar(200, 255, 255);
        Scalar blue_lower = Scalar(80, 100, 70); // Scalar(100, 100, 70);
        Scalar blue_upper = Scalar(150, 255, 255); // Scalar(125, 255, 255);
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        



        float z = 0; //height of the LiDAR scan

    //     void make_transforms()
    // {
    //     geometry_msgs::msg::TransformStamped t;

    //     t.header.stamp = this->get_clock()->now();
    //     t.header.frame_id = "world";
    //     t.child_frame_id = '0';

    //     t.transform.translation.x = 0;
    //     t.transform.translation.y = 0;
    //     t.transform.translation.z = 1;
    //     tf2::Quaternion q;
    //     q.setRPY(
    //     0,
    //     0,
    //     0);
    //     t.transform.rotation.x = q.x();
    //     t.transform.rotation.y = q.y();
    //     t.transform.rotation.z = q.z();
    //     t.transform.rotation.w = q.w();

    //     // staticbroadcaster->sendTransform(t);
    // }
        void flipBaseLinkCallback(std_msgs::msg::Bool::ConstSharedPtr msg) {
            // Albert's flip once tf are fixed
            baseLinkFlipped = msg->data;
        }
        void camera_info_callback(sensor_msgs::msg::CameraInfo::SharedPtr msg) {
            // std::cout << "HERE" << std::endl;
            intrinsic_matrix << msg->k[0], msg->k[1], msg->k[2], 0, msg->k[3], msg->k[4], msg->k[5], 0, msg->k[6], msg->k[7], msg->k[8], 0;
            pixel_to_lidar_matrix = (intrinsic_matrix*extrinsic_matrix)*camera_to_lidar_tf;
            
            Eigen::MatrixXf a = pixel_to_lidar_matrix;
            Eigen::MatrixXf camera_to_lidar_rotation = camera_to_lidar_tf;
            a.conservativeResize(3,3);
            Eigen::VectorXf b(3,1);
            b << pixel_to_lidar_matrix.coeff(0,3), pixel_to_lidar_matrix.coeff(1,3), pixel_to_lidar_matrix.coeff(2,3);

            //Camera center is in LiDAR frame
            camera_center = -a.inverse()*b;
            std::cout << "Camera center: " << camera_center << std::endl;
            // RCLCPP_INFO(this->get_logger(), "step 1");
        }

        void lidar_callback(sensor_msgs::msg::LaserScan::SharedPtr msg) {
            if (pixel_to_lidar_matrix.rows() != 0) {
                lidar_points = convert_to_cartesian(msg->ranges, msg->angle_min, msg->angle_max, msg->angle_increment);
                // std::cout << lidar_points << std::endl;
                // std::cout << "Passed Here" << std::endl;
                lidar_pixel_points = convert_pixel_to_lidar();
                // std::cout << lidar_pixel_points << std::endl;
                find_lidar_board_points();
                // std::cout << "Finished find_lidar_board_points" << std::endl;
                // std::cout << "Points:" << lidar_board_points << std::endl;
                if (lidar_board_points.rows() > 2) {
                    board_lidar_line = find_best_fit_line(lidar_board_points);
                    // std::cout << "Finished find_best_fit_line for lidar_board_points" << std::endl;
                    // std::cout << "BLL "<< board_lidar_line << std::endl;
                    board_pixel_line = find_best_fit_line(pixel_board_points);
                    // std::cout << "Finished find_best_fit_line for pixel_board_points" << std::endl;
                    // std::cout << "BPL" << board_pixel_line << std::endl;
                    center_pixel = find_intersection(board_pixel_line, board_center_pixel_line);
                    // std::cout << "Finished find_intersection" << std::endl;
                    // std::cout << "CP: "<< center_pixel << std::endl;
                    // Shows the center pixel as a dot in the Image (This may be hidden due to high cpu requirements of the full algorithm)
                    cv::Mat final_board_image = cv::Mat::zeros(image_msg.size(), CV_8UC1);
                    auto cp = final_board_image;
                    cv::Point central_pixel = Point(center_pixel(0),center_pixel(1));
                    cv::circle(final_board_image, central_pixel, 5, Scalar(255,255,255), -1);
                    cv::cvtColor(final_board_image, cp, CV_GRAY2BGR);
                    // cv::imwrite("drawing.jpg", final_board_image);
                    std_msgs::msg::Header hdr;
                    sensor_msgs::msg::Image::SharedPtr frame;
                    frame = cv_bridge::CvImage(hdr, "bgr8", cp).toImageMsg(); // Changed from mask
                    dot_publisher_->publish(*frame);
                    // Finds the center point
                    center_point = find_center_point();
                    // z_theta = find_theta();
                    // current_state(0) = center_point(0,0);
                    // current_state(1) = center_point(1,0);
                    // current_state(2) = z_theta;

                    // std::cout << "Center Point: " << center_point << std::endl;
                    // if(got_prior) {
                    //     ekf(current_state(0),current_state(1),current_state(2));
                    // }
                    // prior = current_state;
                    // covariance_prior = current_covariance;
                    // find_tf(current_state(0),current_state(1),current_state(2));
                    // std::cout << "Center Point: " << center_point << std::endl;
                    // find_tf();
                    // std::cout << "Center Point: " << center_point << std::endl;
                    find_tf();
                }
            }
        }

        void camera_callback(sensor_msgs::msg::Image::SharedPtr msg) {
            // std::cout << "camera callback" << std::endl;
            if(test_board_pix_point.size() == 2) {
                try {
                    std::cout << "Center pix point: " << test_board_pix_point << std::endl;
                    
                    cv::Mat cv_ptr = cv_bridge::toCvShare(msg, "bgr8")->image;
                    cv::Point center(test_board_pix_point(0), test_board_pix_point(1));
                    int radius = 1;
            
            // Draw a red circle on the image
                    cv::circle(cv_ptr, center, radius, cv::Scalar(0, 0, 255), -1);
                    std_msgs::msg::Header hdr;
                    sensor_msgs::msg::Image::SharedPtr frame;
                    frame = cv_bridge::CvImage(hdr, "bgr8", cv_ptr).toImageMsg(); // Changed from mask
                    test_board_pixel_publisher_->publish(*frame);
                } catch(...) {
                    std::cout << "Cry me a river" << std::endl;
                }
            }
            auto store = find_camera_board_contour(msg);
            camera_board_contour = store.first;
            camera_blue_contour = store.second;
            // camera_board_contour, camera_blue_contour = find_camera_board_contour(msg);
            if(!blue_max_contour.empty()) {
                board_center_pixel_line = find_superior_center_pixel_line(msg);
            }
            std::cout << board_center_pixel_line << std::endl;
        }

        float find_theta() {
            // std::cout << "L1, L0" << board_lidar_line(1) << " " << board_lidar_line(0) << std::endl;
            float z_tan = board_lidar_line(1)/board_lidar_line(0);
            return atan2(z_tan,1);
        }

        // NOTE To myself what I need to be looking at.
        /**
         * this func. does all the calculation to the transform bet. robots pt. to the homing pt.
         * set odem to the same as map
         * find translation and rotation
         * 
        */
        void find_tf() {
            // For RTAPMAP
            geometry_msgs::msg::TransformStamped map_to_odom = geometry_msgs::msg::TransformStamped();
            map_to_odom.header.stamp = this->get_clock()->now();
            map_to_odom.header.frame_id = "map";
            map_to_odom.child_frame_id = "odom";
            map_to_odom.transform.translation.x = 0;
            map_to_odom.transform.translation.y = 0;
            map_to_odom.transform.translation.z = 0.0;
            Eigen::VectorXf qu = euler_to_quaternion(0,0,0);
            // std::cout << "Q TIME" <<  q << std::endl;
            map_to_odom.transform.rotation.x = qu[0];
            map_to_odom.transform.rotation.y = qu[1];
            map_to_odom.transform.rotation.z = qu[2];
            map_to_odom.transform.rotation.w = qu[3];
            
            // the stuff below just repeat that block
            float z_theta = find_theta();
            
            std::cout << "Z theta: " << z_theta << std::endl;
            Eigen::Matrix4f scan_to_map_tf;
            scan_to_map_tf << cos(z_theta),-sin(z_theta),0,center_point(0,0),
                             sin(z_theta),cos(z_theta),0,center_point(1,0),
                             0,0,1,0,
                             0,0,0,1;
            Eigen::MatrixXf map_to_scan_tf = scan_to_map_tf.inverse();
            //For cart setup, scan->base link is identity, so map->scan = odom->base_link

            geometry_msgs::msg::TransformStamped odom_to_scan = geometry_msgs::msg::TransformStamped();
            odom_to_scan.header.stamp = this->get_clock()->now();
            odom_to_scan.header.frame_id = "odom";
            odom_to_scan.child_frame_id = "scan"; // Cart only
            odom_to_scan.transform.translation.x = map_to_scan_tf(0,3);
            odom_to_scan.transform.translation.y = map_to_scan_tf(1,3);
            odom_to_scan.transform.translation.z = 0.0;

            Eigen::Matrix3f odom_to_scan_rotation;
            odom_to_scan_rotation << map_to_scan_tf(0,0),map_to_scan_tf(0,1),map_to_scan_tf(0,2),
                                          map_to_scan_tf(1,0),map_to_scan_tf(1,1),map_to_scan_tf(1,2),
                                          map_to_scan_tf(2,0),map_to_scan_tf(2,1),map_to_scan_tf(2,2);

            Eigen::Quaternionf odom_scan_q(odom_to_scan_rotation);
            odom_to_scan.transform.rotation.w = odom_scan_q.w();
            odom_to_scan.transform.rotation.x = odom_scan_q.x();
            odom_to_scan.transform.rotation.y = odom_scan_q.y();
            odom_to_scan.transform.rotation.z = odom_scan_q.z();
            
            geometry_msgs::msg::TransformStamped scan_to_base_link = geometry_msgs::msg::TransformStamped();
            scan_to_base_link.header.stamp = this->get_clock()->now();
            scan_to_base_link.header.frame_id = "scan";
            scan_to_base_link.child_frame_id = "base_link";

            // baseLinkStuff dont need to worry about
            // if(!baseLinkFlipped) {
            //     scan_to_base_link.transform.translation.x = scan_to_base_link_tf1(0,3);
            //     scan_to_base_link.transform.translation.y = scan_to_base_link_tf1(1,3);
            //     scan_to_base_link.transform.translation.z = scan_to_base_link_tf1(2,3);

            //     Eigen::Matrix3f scan_to_base_link_rotation;
            //     scan_to_base_link_rotation <<   scan_to_base_link_tf1(0,0),scan_to_base_link_tf1(0,1),scan_to_base_link_tf1(0,2),
            //                                     scan_to_base_link_tf1(1,0),scan_to_base_link_tf1(1,1),scan_to_base_link_tf1(1,2),
            //                                     scan_to_base_link_tf1(2,0),scan_to_base_link_tf1(2,1),scan_to_base_link_tf1(2,2);

            //     Eigen::Quaternionf scan_base_link_q(scan_to_base_link_rotation);
            //     scan_to_base_link.transform.rotation.w = scan_base_link_q.w();
            //     scan_to_base_link.transform.rotation.x = scan_base_link_q.x();
            //     scan_to_base_link.transform.rotation.y = scan_base_link_q.y();
            //     scan_to_base_link.transform.rotation.z = scan_base_link_q.z();
            // }
            // else {
            //     scan_to_base_link.transform.translation.x = scan_to_base_link_tf2(0,3);
            //     scan_to_base_link.transform.translation.y = scan_to_base_link_tf2(1,3);
            //     scan_to_base_link.transform.translation.z = scan_to_base_link_tf2(2,3);

            //     Eigen::Matrix3f scan_to_base_link_rotation_flipped;
            //     scan_to_base_link_rotation_flipped <<   scan_to_base_link_tf2(0,0),scan_to_base_link_tf2(0,1),scan_to_base_link_tf2(0,2),
            //                                             scan_to_base_link_tf2(1,0),scan_to_base_link_tf2(1,1),scan_to_base_link_tf2(1,2),
            //                                             scan_to_base_link_tf2(2,0),scan_to_base_link_tf2(2,1),scan_to_base_link_tf2(2,2);

                

            //     Eigen::Quaternionf scan_base_link_q_flipped(scan_to_base_link_rotation_flipped);
            //     std::cout << "Scan_to_Base_Flipped: " << scan_base_link_q_flipped.w() << scan_base_link_q_flipped.x() << scan_base_link_q_flipped.y() << scan_base_link_q_flipped.z() << std::endl;

            //     scan_to_base_link.transform.rotation.w = scan_base_link_q_flipped.w();
            //     scan_to_base_link.transform.rotation.x = scan_base_link_q_flipped.x();
            //     scan_to_base_link.transform.rotation.y = scan_base_link_q_flipped.y();
            //     scan_to_base_link.transform.rotation.z = scan_base_link_q_flipped.z();
            // }
            
            // broadcaster->sendTransform(map_to_odom);
            // broadcaster->sendTransform(odom_to_scan);
            // broadcaster->sendTransform(scan_to_base_link);
            // found_board_publisher_->publish(true); 
            // auto myFloatMsg = std_msgs::msg::Float32();
            // myFloatMsg.data = z_theta;
            // theta_publisher_->publish(myFloatMsg);
        }

        Eigen::VectorXf euler_to_quaternion(float roll, float pitch, float yaw) {
            Eigen::Quaternionf q;
            q = Eigen::AngleAxisf(roll, Eigen::Vector3f::UnitX()) 
                * Eigen::AngleAxisf(pitch, Eigen::Vector3f::UnitY()) 
                * Eigen::AngleAxisf(yaw, Eigen::Vector3f::UnitZ());
            return q.coeffs();
        }       

        Eigen::MatrixXf find_center_point() {
            //a is 3x4 matrix that tranforms LiDAR point into pixel space
            Eigen::MatrixXf a = pixel_to_lidar_matrix;

            //b is the translation from the physical camera to LiDAR
            Eigen::MatrixXf b = camera_center;
            b.conservativeResize(3,1);

            //Only need rotation matrix because of https://math.stackexchange.com/questions/2237994/back-projecting-pixel-to-3d-rays-in-world-coordinates-using-pseudoinverse-method
            a.conservativeResize(3,3);
            a = a.inverse();

            //Camera ray in LiDAR frame
            Eigen::MatrixXf center_pixel_product = a * center_pixel;

            //Coordinate ratio is the vector from the LiDAR to the center pixel (P - C)
            Eigen::MatrixXf coordinate_ratio = center_pixel_product - b;
            //std::cout << "Center Pixel Product: " << center_pixel_product << std::endl;
            
            Eigen::Matrix3f board_intersection_a;
            board_intersection_a << board_lidar_line(0),board_lidar_line(1),0,1,0,-center_pixel_product(0),0,1,-center_pixel_product(1);
            Eigen::Vector3f board_intersection_b;
            board_intersection_b << board_lidar_line(2),camera_center(0),camera_center(1);
            Eigen::MatrixXf board_intersection = board_intersection_a.inverse() * board_intersection_b;
            std::cout << "Board intersection: " << board_intersection << std::endl;
            
            // if(count < 1000) {
            //     ekf_values.row(count) << board_intersection.coeff(0,0), board_intersection.coeff(1, 0), cvFastArctan(board_intersection.coeff(0, 1), board_intersection.coeff(1, 0));
            //     count++;
            //     std::cout << "Count: " << count << std::endl;
            // }
            // if(count >= 1000) {
            //     auto ekf_col = ekf_values;
            //     double ekf_x[1000];
            //     double ekf_y[1000];
            //     double ekf_theta[1000];
            //     for(int i = 0; i < ekf_values.rows(); i++) {
            //         ekf_x[i] = ekf_values.coeff(i, 0);
            //         ekf_y[i] = ekf_values.coeff(i, 1);
            //         ekf_theta[i] = cvFastArctan(ekf_values.coeff(i, 1), ekf_values.coeff(i, 0));
            //     }
            //     std::cout << "Complete" << std::endl;
            //     std::cout << "Covariance X: " << calculateCovariance(ekf_x, ekf_values) << "Covariance Y: " << calculateCovariance(ekf_y, ekf_values)
            //     << "Covariance Theta" << calculateCovariance(ekf_theta, ekf_values) << std::endl;
            // }

            auto boardIntersectionFloatArray = std_msgs::msg::Float32MultiArray();
            boardIntersectionFloatArray.data = {board_intersection.coeff(0,0), board_intersection.coeff(1, 0), board_intersection.coeff(2, 0)};
            boardIntersection_->publish(boardIntersectionFloatArray); // It looks like the data flips so I'm manually flipping it in the array
            //std::cout << "A inverse b: " << a * b << std::endl;
            //float alpha = (board_lidar_line(0) * coordinate_ratio(0) + board_lidar_line(1) * coordinate_ratio(1)) / board_lidar_line(2); //python code has [0] at very end
            //return coordinate_ratio / alpha;
            
            return board_intersection;
        }

        /*double calculateMean(double column[1000]) {
            double sum = 0;
            for(int i = 0; i < 1000; i++) {
                sum += column[i];
            }
            return sum / 1000;
        }

        std::vector<double> calculateDeviations(double column[1000], double mean) {
            std::vector<double> deviations(1000);
            for (size_t i = 0; i < 1000; ++i) {
                deviations[i] = column[i] - mean;
            }
            return deviations;
        }

        double calculateCovariance(double columnX[1000], const Eigen::MatrixXf& matrix) {
            size_t columnCount = matrix.cols();
            size_t rowCount = matrix.rows();
            
            std::vector<double> deviationsX = calculateDeviations(columnX, calculateMean(columnX));
            double covariance = 0.0;

            for (size_t j = 0; j < columnCount; ++j) {
                double sum = 0.0;
                for (size_t i = 0; i < rowCount; ++i) {
                    sum += deviationsX[i] * (matrix.coeff(i, j) - matrix.row(i).mean());
                }
                covariance += sum / (rowCount - 1);  // Divide by (n-1) for sample covariance
            }
            return covariance;
        }*/

        Eigen::MatrixXf find_intersection(Eigen::MatrixXf A, Eigen::MatrixXf B) {
            float d = (A(0) * B(1)) - (A(1) * B(0));
            // std::cout << A(0) << " " << B(1) << " " << A(1) << " " << B(0) << std::endl;
            float dx = (A(2) * B(1)) - (A(1) * B(2));
            float dy = (A(0) * B(2)) - (A(2) * B(0));
            Eigen::MatrixXf tmp(3,1);
            tmp << dx/d, dy/d, 1;
            // std::cout << d << " " << dx << " " << dy << std::endl;
            return tmp;
        }

        Eigen::MatrixXf find_superior_center_pixel_line(sensor_msgs::msg::Image::SharedPtr msg) {
            // cv::Mat cvMat;
            // cv::Mat cvMatFix;
            // cv::eigen2cv(cam            // cv::Mat pink_mask;
            // cv::Mat blue_mask;
            // cv::Mat complete_mask;
            // cv::Mat blue_area;
            // cv::Mat board_area;era_board_contour, cvMat);
            // std::cout << cvMat << std::endl;
            // cvMat.convertTo(cvMatFix, CV_32F, 1.0f / 255.0f);
            Rect rect = cv::boundingRect(blue_max_contour);
            Point topPoint, bottomPoint;
            topPoint.x = (int)(rect.x + rect.width / 2);
            topPoint.y = (int)(rect.y);
            bottomPoint.x = (int)(rect.x + rect.width / 2);
            bottomPoint.y = (int)(rect.y + rect.height);
            float a, b, d, slope;
            a = 1;
            b = 0;
            d = rect.x + rect.width / 2;

            if (bottomPoint.x != topPoint.x) {
                slope = (bottomPoint.y - topPoint.y) / (bottomPoint.x - topPoint.x);
                a = slope;
                b = -1;
                d = slope * topPoint.x - topPoint.y;
            }
            auto cv_ptr = cv_bridge::toCvShare(msg, "bgr8")->image;
            cv::line(cv_ptr, topPoint, bottomPoint, (0,0,255),9);
            std_msgs::msg::Header hdr;
            sensor_msgs::msg::Image::SharedPtr frame;
            frame = cv_bridge::CvImage(hdr, "bgr8", cv_ptr).toImageMsg();
            line_image_publisher_->publish(*frame);

            Eigen::MatrixXf tmp(3,1);
            tmp << a, b, d;
            // std::cout << "HUH?" << tmp << std::endl;
            return tmp;
        }
        // Difference reference - Here
        std::pair<Eigen::MatrixXf, Eigen::MatrixXf> find_camera_board_contour(sensor_msgs::msg::Image::SharedPtr msg) {
            //RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg->data.c_str());
            cv::Mat cv_img;
            cv::Mat blur;
            cv::Mat color;
            cv::Mat mask;
            cv::Mat eroded;
            cv::Mat dilated;
            cv::Mat con_dilated;
            cv::Mat pink_mask;
            cv::Mat blue_mask;
            cv::Mat complete_mask;
            cv::Mat blue_area;
            cv::Mat board_area;
            Eigen::MatrixXf tmp2 = Eigen::MatrixXf::Zero(1,1);
            
            try
            {                
                
                cv::Mat cv_ptr = cv_bridge::toCvShare(msg, "bgr8")->image;

                // // Extract the image properties
                // int width = msg->width;
                // int height = msg->height;
                // int encoding = CV_8UC3;

                // // Create a cv::Mat object pointing to the image data
                // cv::Mat cv_image(height, width, encoding, const_cast<unsigned char*>(msg->data.data()), msg->step);

                colored_image = cv_ptr;
                // cv::GaussianBlur(cv_ptr, blur, cv::Size(5, 5), 0);
                cv::cvtColor(cv_ptr, color, COLOR_BGR2HSV);
                cv::Mat save = color;
                // Pink Mask
                cv::inRange(color, pink_lower, pink_upper, pink_mask);
                cv::cvtColor(pink_mask, pink_mask, CV_GRAY2BGR);
                // RCLCPP_INFO(this->get_logger(), "step 2");
                // cv::imwrite("pink_mask.jpg", pink_mask);
                cv::erode(pink_mask, pink_mask, cv::Mat(), cv::Point(-1,-1), 1);
                // cv::dilate(pink_mask, pink_mask, cv::Mat());
                // cv::cvtColor(pink_mask, pink_mask, COLOR_BGR2GRAY);
                // RCLCPP_INFO(this->get_logger(), "step 3");
                
                // Blue Mask
                cv::inRange(color, blue_lower, blue_upper, blue_mask);
                cv::cvtColor(blue_mask, blue_mask, CV_GRAY2BGR);
                // auto mask = blue_mask;
                cv::erode(blue_mask, blue_mask, cv::Mat(), cv::Point(-1,-1), 1);
                // cv::dilate(blue_mask, blue_mask, cv::Mat());

                // cv::cvtColor(pink_mask, pink_mask, COLOR_BGR2GRAY);
                // cv::cvtColor(blue_mask, blue_mask, COLOR_BGR2GRAY);
                // Combine 
                // std::cout << "HERE" << std::endl;
                add(pink_mask, blue_mask, complete_mask);
                auto mimic = complete_mask;
                //complete_mask = pink_mask + blue_mask;
                // cv::cvtColor(pink_mask, pink_mask, COLOR_BGR2GRAY);
                cv::cvtColor(blue_mask, blue_mask, COLOR_BGR2GRAY);
                cv::cvtColor(complete_mask, complete_mask, COLOR_BGR2GRAY);
                std::vector<std::vector<cv::Point>> contours;
                std::vector<std::vector<cv::Point>> blue_contours;
                // Old board Stuff
                // cv::inRange(color, cv::Scalar(40, 100, 100), cv::Scalar(80, 255, 255), mask);
                // cv::cvtColor(mask, mask, CV_GRAY2BGR);
                // cv::erode(mask, eroded, cv::Mat(), cv::Point(-1, -1), 2);
                // cv::dilate(mask, dilated, cv::Mat());
                // std::vector<std::vector<cv::Point>> contours;
                // cv::cvtColor(dilated, con_dilated, COLOR_BGR2GRAY);
                // Old board stuff
                // std::cout << "HERE" << std::endl;
                // cv::findContours(con_dilated,contours,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
                cv::findContours(complete_mask,contours,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
                cv::findContours(blue_mask,blue_contours,cv::RETR_TREE,cv::CHAIN_APPROX_SIMPLE);
                // std::cout << complete_mask.size() << std::endl;]
                
                
                
                double max_area = 0;
                double blue_max_area = 0;
                
                int first_contour_index = -1;
                int second_contour_index = -1;
                int max_blue_contour_index = -1;
                double current_area = 0;
                
                // auto copy = contours;
                for(int i = 0;i < contours.size();i++) {
                    // Board Contour
                    current_area = cv::contourArea(contours.at(i));
                    if(current_area > max_area) {
                        max_area = current_area; // Because this is max, we only get half the board for a final product
                        first_contour_index = i;
                    }
                }
                max_area = 0;
                for(int i = 0;i < contours.size();i++) {
                    // Board Contour 2
                    current_area = cv::contourArea(contours.at(i));
                    if(current_area > max_area && i != first_contour_index) {
                        max_area = current_area; // Because this is max, we get the other half the board for a final product
                        second_contour_index = i;
                    }
                }
                
                // std::cout << "HERE" << std::endl;
                for(int i = 0; i < blue_contours.size(); i++) {
                    // Blue Mask Contour
                    double blue_area = cv::contourArea(blue_contours.at(i));
                    if(blue_area > blue_max_area) {
                        blue_max_area = blue_area;
                        max_blue_contour_index = i;
                    }
                }
                // std::cout << "HERE" << std::endl;
                // std::cout << "HERE" << std::endl;
                // first_contour = contours[first_contour_index];
                if(max_blue_contour_index != -1) {
                    blue_max_contour = blue_contours[max_blue_contour_index];
                }
                // std::cout << "HERE" << std::endl;
                image_msg = complete_mask;
                cv::Mat final_board_image = cv::Mat::zeros( complete_mask.size(), CV_8UC1);
                cv::Mat final_blue_image = cv::Mat::zeros( blue_mask.size(), CV_8UC1);
                // std::cout << "HERE" << std::endl;
                // cv::Mat final_image = cv::Mat::zeros( con_dilated.size(), CV_8UC1);
                cv::drawContours(final_board_image,contours,first_contour_index,cv::Scalar(255,0,0),-1); // Changed Thickness to -1 and offset point after tuning, , LINE_8, noArray(), INT_MAX, Point(130,0)
                if(second_contour_index != -1) {
                    cv::drawContours(final_board_image,contours,second_contour_index,cv::Scalar(255,0,0),-1);
                }
                cv::drawContours(final_board_image, blue_contours, max_blue_contour_index, cv::Scalar(255,0,0), -1);
                cv::drawContours(final_blue_image,blue_contours,max_blue_contour_index,cv::Scalar(255,0,0),-1); 
                
                // cv::imwrite("drawing1.jpg", final_blue_image);
                cv::Mat cp = final_board_image;
                cv::cvtColor(final_board_image, cp, CV_GRAY2BGR);
                // Sweet point is between 130-150
                // std::cout << "HERE" << std::endl;
                std_msgs::msg::Header hdr;
                sensor_msgs::msg::Image::SharedPtr frame;
                frame = cv_bridge::CvImage(hdr, "bgr8", cp).toImageMsg(); // Changed from mask
                // std::cout << final_image.size() << " M: "<< mask.size() << std::endl;
                // pub.publish(msg);
                // rclcpp::spin_some(node);
                publisher_->publish(*frame);
                // publisher_->publish(*(cv_ptr->toImageMsg()));
                Eigen::MatrixXf board;
                Eigen::MatrixXf blue_board;
                cv::cv2eigen(final_board_image, board);
                cv::cv2eigen(final_blue_image, blue_board); 


                return std::make_pair(board, blue_board);
            }
            catch (cv_bridge::Exception& e)
            {
                // CV_ERROR("cv_bridge exception: %s", e.what());
                // It's Me....
            }

            return std::make_pair(tmp2, tmp2);
        }

        Eigen::MatrixXf convert_to_cartesian(std::vector<float> ranges, float angle_min, float angle_max, float angle_increment) {
            Eigen::MatrixXf cartesian_list(ranges.size(), 4); // This works now (But there may be a better way to fix)
            Eigen::VectorXf vector_input(4);


            float infStuff = ranges[50];
            // std::cout << infStuff << std::endl;          
            int totalPoints = 0;
            // loop through
            float theta;
            float r;
            float x;
            int count = 0;   
            float y;
            // std::cout << "Lidar Scan Size: " << ranges.size() << std::endl;
            for (int i = 0; i < ranges.size(); ++i) {
                theta = angle_min + angle_increment * i;
                r = ranges[i];
                // std::cout << i << "." << r << std::endl;
                if (r != infStuff) {
                    totalPoints += 1;
                    //  convert
                }
                x = r * cos(theta);
                y = r * sin(theta);
                vector_input << x, y, z, 1;
                cartesian_list.row(i) = vector_input;
                // std::cout << "Ran: " << totalPoints << std::endl;
            }
            //   std::cout << "Array: " << cartesian_list << std::endl;
            // std::cout << "Total Board Points: " << totalPoints << std::endl;
            // Eigen::MatrixXf tmp =cartesian_list;
            // return tmp = cartesian_list;
            return cartesian_list;
        }

        Eigen::MatrixXf convert_pixel_to_lidar() {
            Eigen::MatrixXf pixel_points(lidar_points.rows(),2);
            Eigen::VectorXf lidar_point; // 4x1
            Eigen::VectorXf pix_point(2);
            Eigen::MatrixXf lidar_point_final;
            Eigen::MatrixXf lidar_point_pixel_to_lidar;
            Eigen::MatrixXf lidar_point_extrinsic;
            for (int i = 0; i < lidar_points.rows(); ++i) { 
                lidar_point = lidar_points.row(i);
                //  std::cout << "Lidar_Point: " << lidar_point << std::endl;
                // std::cout << "Pixel to Lidar: " << pixel_to_lidar_matrix << std::endl;
                lidar_point_final = (pixel_to_lidar_matrix * lidar_point);
                if(lidar_point_final(2) < 0.001) {
                    lidar_point_final(2) = 1;
                }
                pix_point << lidar_point_final(0) / lidar_point_final(2), lidar_point_final(1) / lidar_point_final(2); //CHECK: What does (0) grab for a 
                if(lidar_point(1) < 0.01 && lidar_point(1) > -0.01) {
                    std::cout << "Lidar point: " << lidar_point << std::endl;
                    // std::cout << "Pixel point: " << pix_point << std::endl;
                    test_board_pix_point = pix_point;
                }
                pixel_points.row(i) = pix_point;
                // std::cout << "Lidar point Final: " << lidar_point_final << std::endl;
                // std::cout << "Math should check out here" << std::endl;
            }
            // std::cout << "Pixel Points:" << pixel_points << std::endl;
            // std::cout << "Finished convert_pixel_to_lidar" << std::endl;
            return pixel_points;
        }


        void find_lidar_board_points() {
            Eigen::MatrixXf lidar_board_points_local(1,4);
            Eigen::MatrixXf lidar_board_pixels(1,2);
            Eigen::MatrixXf lidar_point;
            int max_width = 640; // 640 with d435
            int j = 1;
            // std::cout << "Me" << std::endl;
            for (int i = 0; i < lidar_pixel_points.rows(); ++i) {
                
                lidar_point = lidar_pixel_points.row(i);
                // Talk to Kush about this closing due to image size, may be incorrect
                if(!isnan(lidar_point(0)) && !isnan(lidar_point(1)) && lidar_point(0) < max_width && lidar_point(0) > 0) {
                    int x_local = int(lidar_point(0));
                    int y_local = int(lidar_point(1));
                    // std::cout << "Number " << i << ": Lidar Point:  " << lidar_points.row(i) << std::endl;
                    // std::cout << "Number " << i << ": Pixel Point:  " << lidar_pixel_points.row(i) << std::endl;
                    //  std::cout << "Me" << std::endl;
                    // std::cout << "Cam Contour: " << camera_board_contour << std::endl;
                    if (camera_board_contour.coeff(240,x_local) > 0) { // It is y,x
                        // std::cout << "Number: " << i << ": " << lidar_points.row(i) << std::endl;
                        // std::cout << "Number: " << i << std::endl;
                        lidar_board_pixels.row(j-1) = lidar_point;
                        // std::cout << lidar_points.row(i) << std::endl;
                        lidar_board_points_local.row(j-1) = lidar_points.row(i);
                        lidar_board_pixels.conservativeResize(j+1,2);
                        lidar_board_points_local.conservativeResize(j+1,4);
                        // std::cout << "Finished.." << std::endl;
                        j++;
                    }
                }
            }
            // std::cout << lidar_board_points_local << std::endl;
            // std::cout << lidar_board_pixels << std::endl;
            // std::cout << "Cam Contour: " << camera_board_contour << std::endl;
            // std::cout << "Done" << std::endl;
            if (j != 1) {
                lidar_board_pixels.conservativeResize(j-1,2);
                lidar_board_points_local.conservativeResize(j-1,4);
                Eigen::MatrixXf points = remove_outliers(lidar_board_points_local, lidar_board_pixels, 2.5);
                lidar_board_points = points;
                pixel_board_points = lidar_board_pixels;
                // pixel_board_points = remove_outliers(lidar_board_pixels, 5, 5);
                if(lidar_board_points.size() == 0) {
                    lidar_board_points = lidar_board_points_local;
                    pixel_board_points = lidar_board_pixels;
                }                    
                // pixel_board_points = lidar_board_pixels;
                // lidar_board_points = remove_first_row(lidar_board_points);
                // std::cout << pixel_board_points << std::endl;
            }

            cv::Mat cp = cv::Mat::zeros(colored_image.size(), CV_8UC1);    
            for(int i = 0; i < pixel_board_points.rows(); i++) {
                // std::cout << "Pixel Points: " << pixel_board_points.row(i) <<  std::endl;
                cv::Point lidar_pixel = Point((int)pixel_board_points.coeff(i,0), 240); // I forced it to 240 since the pixel are at a constant y in pixel space
                // and that constant y may not always contain the board
                cv::circle(colored_image, lidar_pixel, 2, Scalar(255,255,255), -1);
            }
        
            std_msgs::msg::Header hdr;
            sensor_msgs::msg::Image::SharedPtr frame;
            frame = cv_bridge::CvImage(hdr, "bgr8", colored_image).toImageMsg();
            lidar_line_publisher_->publish(*frame);
        }
        // Eigen::MatrixXf remove_first_row(Eigen::MatrixXf& mat) {
        //     int n_rows = mat.rows();
        //     int n_cols = mat.cols();
        //     Eigen::MatrixXf result(n_rows-1, n_cols);
        //     result << mat.block(1, 0, n_rows-1, n_cols);
        //     return result;
        // }

        Eigen::MatrixXf find_best_fit_line(Eigen::MatrixXf points) {
            Eigen::MatrixXf A_list(points.rows(), 2);
            Eigen::MatrixXf b_list(points.rows(), 1);
            Eigen::MatrixXf tmp(3, 1);

            for (int i = 0; i < points.rows(); ++i) {
                // tmp = points.row(i);
                A_list.row(i) << points.row(i)(0), points.row(i)(1); // tmp(0),tmp(1);
                b_list.row(i) << 1.0;
            }
            // std::cout << "Points: " << points << std::endl;
            // std::cout << "A" << A_list << std::endl;
            Eigen::MatrixXf A_arr_trans = A_list.transpose();
            Eigen::MatrixXf x_arr = (A_arr_trans * A_list).inverse() * A_arr_trans * b_list;
            
            float a = x_arr.coeff(0,0); // 0,1
            float b = x_arr.coeff(1,0);
            float d = 1.0;
            tmp << a,b,d;
            // std::cout << "tmp" << tmp << std::endl;
            return tmp;
        }     

        // double distance(Point p1, Point p2) {
        //     double dx = p1.x - p2.x;
        //     double dy = p1.y - p2.y;
        //     return std::sqrt(dx*dx + dy*dy);
        // }
        Eigen::MatrixXf remove_outliers(Eigen::MatrixXf& points, Eigen::MatrixXf& pixel_points, double threshold) {
            int num_points = 0;
            Eigen::MatrixXf actual_points(points.rows(), 4);
            for(int i = 0; i < points.rows(); i++) {
                if(points.coeff(i,0) < 9) {
                    actual_points.row(num_points) = points.row(i);
                    num_points++;
                }
            }
            if(num_points == 0) {
                return points;
            }
            actual_points.conservativeResize(num_points, 4);
            
            // int num_points = points.rows();
            double mean_x = actual_points.col(0).mean();
            double std_dev_x = std::sqrt((actual_points.col(0).array() - mean_x).square().sum() / num_points);

            // create a mask to mark outliers
            Eigen::VectorXd mask(num_points);
            for (int i = 0; i < num_points; i++) {
                if (std::abs(actual_points(i, 0) - mean_x) > 0.5) { // Changed threshold to 0.5
                    mask(i) = 0.0;
                } else {
                    mask(i) = 1.0;
                }
            }
            // std::cout << "Points:" << actual_points << std::endl;
            // std::cout << "Mean: " << mean_x << std::endl;
            // std::cout << "standard deviation: " << std_dev_x <<std::endl;
            // std::cout << "threshold: " << threshold * std_dev_x << std::endl;
            // use the mask to filter out outliers
            int num_inliers = mask.sum();
            Eigen::MatrixXf filtered_points(num_inliers, 4);
            Eigen::MatrixXf filtered_pixels(num_inliers, 2);
            int filtered_index = 0;
            for (int i = 0; i < num_points; i++) {
                if (mask(i) > 0.0) {
                    filtered_points.row(filtered_index) = actual_points.row(i);
                    // filtered_pixels.row(filtered_index) = pixel_points.row(i); // Not used anymore, will remove
                    filtered_index++;
                }
            }

            std::cout << "Filtered Points: " << filtered_points << std::endl;

            return filtered_points;
        }

        /*void ekf(float x_measurement,float y_measurement,float theta_measurement) {
            std::cout << "IN EKF" << std::endl;
            std::cout << "State prediction: " << prior(0) << " " << prior(1) << " " << prior(2) << std::endl;
            Eigen::Matrix3f covariance_prediction = covariance_prior + q_matrix;
            std::cout << "Covariance prediction: " << covariance_prediction << std::endl;
            Eigen::Matrix3f kalman_gain = covariance_prediction * (covariance_prediction + r_matrix).inverse();
            std::cout << "Pre inverse: " << (covariance_prediction + r_matrix) << std::endl;
            std::cout << "The inverse: " << (covariance_prediction + r_matrix).inverse() << std::endl;
            std::cout << "Kalman gain: " << kalman_gain << std::endl;
            Eigen::Vector3f measurement_vector;
            measurement_vector << x_measurement, y_measurement, theta_measurement;
            std::cout << "Measurement vector: " << measurement_vector(0) << " " << measurement_vector(1) << " " << measurement_vector(2) << std::endl;
            current_state = prior + kalman_gain * (measurement_vector - prior);
            std::cout << "New state: " << current_state << std::endl;
            current_covariance = (Eigen::Matrix3f::Identity(3,3) - kalman_gain) * covariance_prediction;
            std::cout << "New covariance: " << current_covariance << std::endl;
        }*/
        // Eigen::MatrixXf convert_transform_to_matrix(geometry_msgs::msg::TransformStamped transform) {
            
        //     Eigen::MatrixXf rotation_matrix = convert_quaternion_to_rotation_matrix(transform.transform.rotation); // Rotation Matrix 
            
        //     Eigen::MatrixXf full_matrix = rotation_matrix;   
        //     full_matrix.conservativeResize(3,4);
        //     full_matrix(0,3) = transform.transform.translation.x;
        //     full_matrix(1,3) = transform.transform.translation.y;
        //     full_matrix(2,3) = transform.transform.translation.z;
        //     // std::cout << "Here?" << std::endl;
        //     full_matrix.conservativeResize(4,4);
        //     full_matrix(3,0) = 0;
        //     full_matrix(3,1) = 0;
        //     full_matrix(3,2) = 0;
        //     full_matrix(3,3) = 1;
            
        //     return full_matrix;
        // }

        // Eigen::Matrix3f convert_quaternion_to_rotation_matrix(geometry_msgs::msg::Quaternion matrix) {
        //     auto q0 = matrix.x;
        //     auto q1 = matrix.y;
        //     auto q2 = matrix.z;
        //     auto q3 = matrix.w;

        //     // First row of the rotation matrix
        //     auto r00 = 2 * (q0 * q0 + q1 * q1) - 1;
        //     auto r01 = 2 * (q1 * q2 - q0 * q3);
        //     auto r02 = 2 * (q1 * q3 + q0 * q2);
            
        //     // Second row of the rotation matrix
        //     auto r10 = 2 * (q1 * q2 + q0 * q3);
        //     auto r11 = 2 * (q0 * q0 + q2 * q2) - 1;
        //     auto r12 = 2 * (q2 * q3 - q0 * q1);
            
        //     // Third row of the rotation matrix
        //     auto r20 = 2 * (q1 * q3 - q0 * q2);
        //     auto r21 = 2 * (q2 * q3 + q0 * q1);
        //     auto r22 = 2 * (q0 * q0 + q3 * q3) - 1;

        //     Eigen::Matrix3f rotation;
        //     rotation << r00, r01, r02,
        //                 r10, r11, r12,
        //                 r20, r21, r22;
        //     return rotation;
        // }

        Eigen::Quaternionf rotation_matrix_to_quaternion(Eigen::Matrix3f rotation_matrix) {
            Eigen::Quaternionf quaternion(rotation_matrix);
            return quaternion;
        }

        rclcpp::Subscription<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;
        rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr flipBaseLink;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr line_image_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr dot_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr lidar_line_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr test_board_pixel_publisher_;
        Eigen::VectorXf test_board_pix_point;
        rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr found_board_publisher_;
        rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr boardIntersection_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> broadcaster;

        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr theta_publisher_;
        // std::shared_ptr<tf2_ros::StaticTransformBroadcaster> staticbroadcaster;
        // rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("image_publisher");
        
};
int main(int argc, char **argv)
{
    // You can ignore the red squiggles, you can build and run this

    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}