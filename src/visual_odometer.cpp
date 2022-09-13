#include "visual_odometer.h"
#include "odometry_handler.h"
#include "calibration_handler.h"

#include <boost/filesystem.hpp>

using namespace pylon_odometry;

std::optional<double> VisualOdometer::processing_velocity() {
    if (right_cam_.size() < 2 || left_cam_.size() < 2) {
        return false;
    }

    ObometryHangler::OdometryImages images;
    images.prev_left = left_cam_.front();
    images.current_left = left_cam_.back();
    images.prev_right = right_cam_.front();
    images.current_right = right_cam_.back();

    auto [speed, service_image] = odometry_handler_->processing_image(images);

    service_image_ = service_image;
    return speed;
}

VisualOdometer::VisualOdometer(const VisualOdometer::OdometerPatams &params) : params_(params) {
    ObometryHangler::HandlerParams odometry_handler_params;
    odometry_handler_params.path_to_config = params_.path_to_config;
    odometry_handler_params.algorithm = params.algorithm;
    odometry_handler_ = std::make_unique<ObometryHangler>(odometry_handler_params);

    Calibrator::CalibratorParams calibration_params;
    calibration_params.path_to_config = params.path_to_config;
    calibration_params.path_to_left_calibrate_images = params.path_to_left_calibrate_images;
    calibration_params.path_to_right_calibrate_images = params.path_to_right_calibrate_images;
    calibration_handler_ = std::make_unique<Calibrator>(calibration_params);

    if(!boost::filesystem::exists(params_.path_to_config)) {
        calibration_handler_->calibrate();
    }
}

void VisualOdometer::set_left_image(cv::Mat left_image) {
    while (left_cam_.size() > 2) {
        left_cam_.pop();
    }
    left_cam_.push(left_image);
}

void VisualOdometer::set_right_image(cv::Mat left_image) {
    while (right_cam_.size() > 2) {
        right_cam_.pop();
    }
    right_cam_.push(left_image);
}
