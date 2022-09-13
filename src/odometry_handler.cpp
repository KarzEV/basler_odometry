#include "odometry_handler.h"

#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <algorithm>
#include <vector>
#include <ctime>

#include "odometry_lib/feature.h"
#include "odometry_lib/utils.h"
#include "odometry_lib/visualOdometry.h"

using namespace std;
using namespace pylon_odometry;

ObometryHangler::ObometryHangler(const HandlerParams &params) : params_(params) {
    cv::FileStorage fSettings(params_.path_to_config, cv::FileStorage::READ);

    cv::Mat camera_matrix_right, dist_coefs_right, camera_matrix_left, dist_coefs_left, R1, R2, P1, P2;
    cv::Size image_size;

    fSettings["Camera.M1"] >> camera_matrix_left;
    fSettings["Camera.M2"] >> camera_matrix_right;

    fSettings["Camera.D1"] >> dist_coefs_left;
    fSettings["Camera.D2"] >> dist_coefs_right;

    fSettings["Camera.R1"] >> R1;
    fSettings["Camera.R2"] >> R2;

    fSettings["Camera.P1"] >> P1;
    fSettings["Camera.P2"] >> P1;

    fSettings["Image.size"] >> image_size;

    fSettings["Camera.P1"] >> projMatrl_;
    fSettings["Camera.P2"] >> projMatrr_;

    cout << "P_left: " << endl << projMatrl_ << endl;
    cout << "P_right: " << endl << projMatrr_ << endl;

    cv::initUndistortRectifyMap(camera_matrix_left, dist_coefs_left, R1, P1, image_size, CV_16SC2, map11_, map12_);
    cv::initUndistortRectifyMap(camera_matrix_right, dist_coefs_right, R2, P2, image_size, CV_16SC2, map21_, map22_);
}

std::pair<double, cv::Mat> ObometryHangler::processing_image(const ObometryHangler::OdometryImages &images) {
    ObometryHangler::OdometryImages rimages;

    cv::remap( images.prev_left, rimages.prev_left, map11_, map12_, cv::INTER_LINEAR);
    cv::remap( images.current_left, rimages.current_left, map11_, map12_, cv::INTER_LINEAR);
    cv::remap( images.prev_right, rimages.prev_right, map21_, map22_, cv::INTER_LINEAR);
    cv::remap( images.current_right, rimages.current_right, map21_, map22_, cv::INTER_LINEAR);

    cv::Mat rotation = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat translation = cv::Mat::zeros(3, 1, CV_64F);

    std::cout << "frame_pose " << frame_pose_ << std::endl;
    cv::Mat trajectory = cv::Mat::zeros(600, 1200, CV_8UC3);
    FeatureSet currentVOFeatures;
    cv::Mat points4D, points3D;
    int init_frame_id = 0;

    std::vector<FeaturePoint> oldFeaturePointsLeft;
    std::vector<FeaturePoint> currentFeaturePointsLeft;

    std::vector<cv::Point2f> oldPointsLeft_t0 = currentVOFeatures.points;

    std::vector<cv::Point2f> pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1;
    matchingFeatures(rimages.prev_left, rimages.prev_right,
                     rimages.prev_left, rimages.current_right,
                     currentVOFeatures,
                     pointsLeft_t0,
                     pointsRight_t0,
                     pointsLeft_t1,
                     pointsRight_t1, params_.algorithm);

    std::vector<cv::Point2f> newPoints;
    std::vector<bool> valid;

    cv::Mat points3D_t0, points4D_t0;
    cv::triangulatePoints(projMatrl_, projMatrr_, pointsLeft_t0, pointsRight_t0, points4D_t0);
    cv::convertPointsFromHomogeneous(points4D_t0.t(), points3D_t0);

    trackingFrame2Frame(projMatrl_, projMatrr_, pointsLeft_t0, pointsLeft_t1, points3D_t0, rotation, translation,
                        false);

    cv::Vec3f rotation_euler = rotationMatrixToEulerAngles(rotation);

    cv::Mat rigid_body_transformation;

    if (abs(rotation_euler[1]) < 0.1 && abs(rotation_euler[0]) < 0.1 && abs(rotation_euler[2]) < 0.1) {
        integrateOdometryStereo(rigid_body_transformation, frame_pose_, rotation, translation);
    } else {
        std::cout << "Too large rotation" << std::endl;
    }
    clock_t current_time = clock();

    cv::Mat xyz = frame_pose_.col(3).clone();
    double distance = sqrt(pow((xyz.at<double>(0)) - (prev_pose_.at<double>(0)), 2)
                           + pow((xyz.at<double>(1)) - (prev_pose_.at<double>(1)), 2)
                           + pow((xyz.at<double>(2)) - (prev_pose_.at<double>(2)), 2));
    double velocity = distance / (current_time - prev_time_);
    prev_pose_ = xyz;
    prev_time_ = current_time;

    display(trajectory_, xyz);
    return {velocity, cv::Mat()};
}