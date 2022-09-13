#pragma once

#include <string>
#include <queue>
#include <optional>

#include "opencv2/opencv.hpp"

#include "odometry_lib/utils.h"

#include "odometry_handler.h"
#include "calibration_handler.h"

namespace pylon_odometry {
    class VisualOdometer {
    public:
        struct OdometerPatams {
            std::string path_to_config;
            std::string path_to_left_calibrate_images;
            std::string path_to_right_calibrate_images;
            Algorithm algorithm;
        };

        explicit VisualOdometer(const OdometerPatams& params);
        void set_left_image(cv::Mat left_image);
        void set_right_image(cv::Mat left_image);

        std::optional<double> processing_velocity();
        cv::Mat get_service_image() {return service_image_;}

    private:
        std::queue<cv::Mat> right_cam_;
        std::queue<cv::Mat> left_cam_;
        OdometerPatams params_;
        cv::Mat service_image_;
        double velocity_ = 0;
        std::unique_ptr<ObometryHangler> odometry_handler_;
        std::unique_ptr<Calibrator> calibration_handler_;
    };
} // end pylon_odometry

