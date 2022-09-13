#include "odometry_lib/visualOdometry.h"

using namespace cv;
namespace pylon_odometry {
    void checkValidMatch(std::vector<cv::Point2f> &points,
                         std::vector<cv::Point2f> &points_return, std::vector<bool> &status,
                         int threshold) {
        int offset;
        for (int i = 0; i < points.size(); i++) {
            offset = std::max(std::abs(points[i].x - points_return[i].x), std::abs(points[i].y - points_return[i].y));

            if (offset > threshold) {
                status.push_back(false);
            } else {
                status.push_back(true);
            }
        }
    }

    void removeInvalidPoints(std::vector<cv::Point2f> &points, const std::vector<bool> &status) {
        int index = 0;
        for (int i = 0; i < status.size(); i++) {
            if (status[i] == false) {
                points.erase(points.begin() + index);
            } else {
                index++;
            }
        }
    }


    void matchingFeatures(cv::Mat &imageLeft_t0, cv::Mat &imageRight_t0,
                          cv::Mat &imageLeft_t1, cv::Mat &imageRight_t1,
                          FeatureSet &currentVOFeatures,
                          std::vector<cv::Point2f> &pointsLeft_t0,
                          std::vector<cv::Point2f> &pointsRight_t0,
                          std::vector<cv::Point2f> &pointsLeft_t1,
                          std::vector<cv::Point2f> &pointsRight_t1, Algorithm algorithm) {
        std::vector<cv::Point2f> pointsLeftReturn_t0;


        if (currentVOFeatures.size() < 2000) {
            appendNewFeatures(imageLeft_t0, currentVOFeatures, algorithm);
        }

        int bucket_size = imageLeft_t0.rows / 10;
        int features_per_bucket = 1;
        bucketingFeatures(imageLeft_t0, currentVOFeatures, bucket_size, features_per_bucket);

        pointsLeft_t0 = currentVOFeatures.points;

        circularMatching(imageLeft_t0, imageRight_t0, imageLeft_t1, imageRight_t1,
                         pointsLeft_t0, pointsRight_t0, pointsLeft_t1, pointsRight_t1, pointsLeftReturn_t0,
                         currentVOFeatures);
        std::vector<bool> status;
        checkValidMatch(pointsLeft_t0, pointsLeftReturn_t0, status, 0);

        removeInvalidPoints(pointsLeft_t0, status);
        removeInvalidPoints(pointsLeft_t1, status);
        removeInvalidPoints(pointsRight_t0, status);
        removeInvalidPoints(pointsRight_t1, status);

        currentVOFeatures.points = pointsLeft_t1;

    }


    void trackingFrame2Frame(cv::Mat &projMatrl, cv::Mat &projMatrr,
                             std::vector<cv::Point2f> &pointsLeft_t0,
                             std::vector<cv::Point2f> &pointsLeft_t1,
                             cv::Mat &points3D_t0,
                             cv::Mat &rotation,
                             cv::Mat &translation,
                             bool mono_rotation) {

        double focal = projMatrl.at<float>(0, 0);
        cv::Point2d principle_point(projMatrl.at<float>(0, 2), projMatrl.at<float>(1, 2));

        cv::Mat E, mask;
        cv::Mat translation_mono = cv::Mat::zeros(3, 1, CV_64F);
        if (mono_rotation) {
            E = cv::findEssentialMat(pointsLeft_t0, pointsLeft_t1, focal, principle_point, cv::RANSAC, 0.999, 1.0,
                                     mask);
            cv::recoverPose(E, pointsLeft_t0, pointsLeft_t1, rotation, translation_mono, focal, principle_point, mask);
        }
        cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);
        cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);
        cv::Mat intrinsic_matrix = (cv::Mat_<float>(3, 3) << projMatrl.at<float>(0, 0), projMatrl.at<float>(0,
                                                                                                            1), projMatrl.at<float>(
                0, 2),
                projMatrl.at<float>(1, 0), projMatrl.at<float>(1, 1), projMatrl.at<float>(1, 2),
                projMatrl.at<float>(2, 0), projMatrl.at<float>(2, 1), projMatrl.at<float>(2, 2));

        int iterationsCount = 500;
        float reprojectionError = .5;
        float confidence = 0.999;
        bool useExtrinsicGuess = true;
        int flags = cv::SOLVEPNP_ITERATIVE;

        cv::Mat inliers;
        cv::solvePnPRansac(points3D_t0, pointsLeft_t1, intrinsic_matrix, distCoeffs, rvec, translation,
                           useExtrinsicGuess, iterationsCount, reprojectionError, confidence,
                           inliers, flags);

        if (!mono_rotation) {
            cv::Rodrigues(rvec, rotation);
        }

        std::cout << "[trackingFrame2Frame] inliers size: " << inliers.size() << std::endl;
    }
}
