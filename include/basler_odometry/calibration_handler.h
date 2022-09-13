#pragma once

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

namespace pylon_odometry {
    class Calibrator {
    public:
        struct CalibratorParams {
            std::string path_to_config;
            std::string path_to_left_calibrate_images;
            std::string path_to_right_calibrate_images;
            cv::Size board_sz = {8, 6};
            float calib_squar_edge_lenght = 0.026f;
        };

        explicit Calibrator(const CalibratorParams &params) : params_(params) {}

        bool calibrate();

    private:
        CalibratorParams params_;

        void create_array_of_images_(std::ifstream &file,
                                    std::vector<cv::Mat> &img_array);
        void create_known_chessboadr_posiyion_(std::vector<cv::Point3f> &object_corners);
        size_t get_chessboard_corners_(std::vector<cv::Mat> img_arr, std::vector<std::vector<cv::Point2f>> &found_coreners,
                                      std::vector<std::vector<cv::Point3f>> &obj_points,
                                      std::vector<cv::Point3f> obj_corners);
        size_t get_chessboard_corners_(std::vector<cv::Mat> img_arr, std::vector<std::vector<cv::Point2f>> &found_coreners);
    };
} // end pylon_odometry

