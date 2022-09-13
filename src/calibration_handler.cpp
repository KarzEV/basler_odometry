#include "calibration_handler.h"

#include <exception>

namespace pylon_odometry {

    bool Calibrator::calibrate() {
        std::vector<cv::Mat> left_calibration_array, right_calibration_array;
        std::vector<cv::Point3f> object_corners;
        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> img_points_left, img_points_right;
        cv::Mat camera_matrix_right, dist_coefs_right, camera_matrix_left, dist_coefs_left, R, T, E, F;

        std::ifstream left_file, right_file;
        left_file.open(params_.path_to_left_calibrate_images);
        left_file.open(params_.path_to_right_calibrate_images);

        create_array_of_images_(left_file, left_calibration_array);
        create_array_of_images_(right_file, right_calibration_array);

        create_known_chessboadr_posiyion_(object_corners);

        get_chessboard_corners_(left_calibration_array, img_points_left,
                                object_points, object_corners);
        get_chessboard_corners_(right_calibration_array, img_points_right);

        auto imageSize = left_calibration_array.front().size();

        cv::stereoCalibrate(object_points, img_points_left, img_points_right,
                            camera_matrix_left, dist_coefs_left,
                            camera_matrix_right, dist_coefs_right, imageSize, R, T, E, F,
                            cv::CALIB_RATIONAL_MODEL | cv::CALIB_FIX_PRINCIPAL_POINT);

        cv::FileStorage config_fs(params_.path_to_config, cv::FileStorage::WRITE | cv::FileStorage::FORMAT_YAML);

        cv::Mat R1, R2, P1, P2;

        stereoRectify(
                camera_matrix_left, dist_coefs_left, camera_matrix_right, dist_coefs_right,
                imageSize,
                R, T, R1, R2, P1, P2,
                cv::noArray(), 0
        );

        config_fs << "Camera.M1" << camera_matrix_left;
        config_fs << "Camera.M2" << camera_matrix_right;

        config_fs << "Camera.D1" << dist_coefs_left;
        config_fs << "Camera.D2" << dist_coefs_right;

        config_fs << "Camera.R1" << R1;
        config_fs << "Camera.R2" << R2;

        config_fs << "Camera.P1" << P1;
        config_fs << "Camera.P2" << P1;

        config_fs << "Image.size" << imageSize;
    }

    void Calibrator::create_array_of_images_(std::ifstream &file,
                                             std::vector<cv::Mat> &img_array) {
        if (file.is_open()) {
            std::string file_name;
            while (file >> file_name) {
                img_array.push_back(cv::imread(file_name, CV_LOAD_IMAGE_UNCHANGED));
            }
            file.close();
        } else {
            throw std::invalid_argument("can not open file");
        }
    }

    void Calibrator::create_known_chessboadr_posiyion_(std::vector<cv::Point3f> &object_corners) {
        for (int i = 0; i < params_.board_sz.width; i++) {
            for (int j = 0; j < params_.board_sz.height; j++) {
                object_corners.push_back(
                        cv::Point3f(i * params_.calib_squar_edge_lenght, j * params_.calib_squar_edge_lenght, 0.0f));
            }
        }
    }

    size_t Calibrator::get_chessboard_corners_(std::vector<cv::Mat> img_arr,
                                               std::vector<std::vector<cv::Point2f>> &found_coreners,
                                               std::vector<std::vector<cv::Point3f>> &obj_points,
                                               std::vector<cv::Point3f> obj_corners) {
        size_t success_cases = img_arr.size() * 0.5;
        while (found_coreners.size() != success_cases) {
            for (size_t i = 0; i != success_cases; i++) {
                std::vector<cv::Point2f> bufffer;
                bool result = cv::findChessboardCorners(img_arr[i], params_.board_sz,
                                                        bufffer);
                if (result) {
                    found_coreners.push_back(bufffer);
                    obj_points.push_back(obj_corners);
                    cv::drawChessboardCorners(img_arr[i], params_.board_sz, bufffer,
                                              result);
                    cv::imshow("CHESSBOARD", img_arr[i]);
                    cv::waitKey(50);
                } else {
                    throw std::invalid_argument("can not find chessboard");
                }
            }
            return found_coreners.size();
        }
    }

    size_t Calibrator::get_chessboard_corners_(std::vector<cv::Mat> img_arr,
                                               std::vector<std::vector<cv::Point2f>> &found_coreners) {
        size_t success_cases = img_arr.size() * 0.5;

        while (found_coreners.size() != success_cases) {
            for (size_t i = 0; i != success_cases; i++) {
                std::vector<cv::Point2f> bufffer;
                bool result = cv::findChessboardCorners(img_arr[i], params_.board_sz,
                                                        bufffer);
                if (result) {
                    found_coreners.push_back(bufffer);
                    cv::drawChessboardCorners(img_arr[i], params_.board_sz, bufffer,
                                              result);
                    cv::imshow("CHESSBOARD", img_arr[i]);
                    cv::waitKey(50);
                } else {
                    throw std::invalid_argument("can not find chessboard");
                }
            }
            return found_coreners.size();
        }
    }
} // end pylon_odometry
