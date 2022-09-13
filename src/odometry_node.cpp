#include <string>
#include <iostream>
#include <memory>

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <image_transport/image_transport.h>
#include <std_msgs/Float64.h>

#include <cv_bridge/cv_bridge.h>

#include "odometry_lib/utils.h"

#include "visual_odometer.h"

using namespace pylon_odometry;

Algorithm get_algorithm_type(const std::string &str_algorithm) {
    if (str_algorithm == "FAST") {
        return Algorithm::FAST;
    } else if (str_algorithm == "SURF") {
        return Algorithm::SURF;
    } else if (str_algorithm == "BRISK") {
        return Algorithm::BRISK;
    } else {
        throw std::invalid_argument("incorrect name for algorithm");
    }
}

class OdometryNode {
public:
    OdometryNode() {
        ros::NodeHandle nd;
        ros::NodeHandle pnd("~");

        std::string path_to_config;
        std::string path_to_left_calibrate_images;
        std::string path_to_right_calibrate_images;
        std::string algorithm;

        get_node_parameter_(pnd, "path_to_config", path_to_config);
        get_node_parameter_(pnd, "path_to_left_calibrate_images", path_to_left_calibrate_images);
        get_node_parameter_(pnd, "path_to_right_calibrate_images", path_to_right_calibrate_images);
        get_node_parameter_(pnd, "algorithm", algorithm);
        get_node_parameter_(pnd, "service_regim", service_regim_);

        VisualOdometer::OdometerPatams odometer_params;
        odometer_params.path_to_config = path_to_config;
        odometer_params.algorithm = get_algorithm_type(algorithm);

        odometer_ = std::make_unique<VisualOdometer>(odometer_params);

        left_cam_subscriber_ = nd.subscribe("/basler_odometry/left", 1, &OdometryNode::left_cam_callback_, this);
        right_cam_subscriber_ = nd.subscribe("/basler_odometry/right", 1, &OdometryNode::right_cam_callback_, this);

        speed_publisher_ = nd.advertise<std_msgs::Float64>("/basler_odometry/speed", 1);

        if (service_regim_) {
            service_image_publisher_ = nd.advertise<sensor_msgs::Image>("/basler_odometry/service_image", 1);
        }
    }

    void spin() {
        auto speed = odometer_->processing_velocity();
        if (speed.has_value()) {
            std_msgs::Float64 speed_message;
            speed_message.data = speed.value();
            speed_publisher_.publish(speed_message);
        }
        if (service_regim_) {
            auto service_image = odometer_->get_service_image();
            if (!service_image.empty()) {
                sensor_msgs::Image service_image_message;
                cv_bridge::CvImage(std_msgs::Header(),
                                   sensor_msgs::image_encodings::BGR8,
                                   service_image).toImageMsg(service_image_message);
                service_image_publisher_.publish(service_image_message);
            }
        }
    };

private:
    std::unique_ptr<VisualOdometer> odometer_;
    ros::Subscriber left_cam_subscriber_;
    ros::Subscriber right_cam_subscriber_;
    ros::Publisher speed_publisher_;
    ros::Publisher service_image_publisher_;
    bool service_regim_;

    template<typename T>
    void get_node_parameter_(ros::NodeHandle nd, std::string const &name_parameter, T &parameter) {
        if (!nd.getParam(name_parameter, parameter)) {
            std::string error_msg(
                    "Speed Validator Node: parameter: " + name_parameter + " is not found on rosparam server");
            throw std::invalid_argument(error_msg);
        }
    }

    void left_cam_callback_(const sensor_msgs::Image &left_image) {
        odometer_->set_left_image(cv_bridge::toCvCopy(left_image, sensor_msgs::image_encodings::BGR8)->image);
    }

    void right_cam_callback_(const sensor_msgs::Image &right_image) {
        odometer_->set_right_image(cv_bridge::toCvCopy(right_image, sensor_msgs::image_encodings::BGR8)->image);
    }
};

int main(int argc, char **argv) {
    using namespace std::string_literals;

    ros::init(argc, argv, "visual_odometry");

    OdometryNode node;
    ros::Rate rate(10);
    while (ros::ok()) {
        ros::spinOnce();
        node.spin();
        rate.sleep();
    }
}
