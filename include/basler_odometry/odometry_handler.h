#pragma once

#include <utility>

#include <opencv2/opencv.hpp>

#include "odometry_lib/utils.h"

namespace pylon_odometry {
    class ObometryHangler {
    public:
        struct OdometryImages {
            cv::Mat prev_left;
            cv::Mat prev_right;
            cv::Mat current_left;
            cv::Mat current_right;
        };

        struct HandlerParams {
            std::string path_to_config;
            Algorithm algorithm;
        };

        explicit ObometryHangler(const HandlerParams& params);
        ~ObometryHangler()=default;

        std::pair<double, cv::Mat> processing_image(const OdometryImages& images);
    private:
        HandlerParams params_;
        cv::Mat frame_pose_ = cv::Mat::eye(4, 4, CV_64F);
        cv::Mat prev_pose_ = cv::Mat::zeros(3, 1, CV_64F);
        clock_t prev_time_ = 0;
        cv::Mat projMatrl_;
        cv::Mat projMatrr_;
        cv::Mat map11_;
        cv::Mat map12_;
        cv::Mat map21_;
        cv::Mat map22_;
        cv::Mat trajectory_ = cv::Mat::zeros(600, 1200, CV_8UC3);
    };
} // end pylon_odometry