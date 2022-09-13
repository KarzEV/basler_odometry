#pragma once

#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <cctype>
#include <algorithm>
#include <iterator>
#include <vector>
#include <ctime>
#include <sstream>
#include <fstream>
#include <string>


#include "odometry_lib/feature.h"

namespace pylon_odometry {
    enum class Algorithm {
        FAST = 1,
        SURF = 2,
        BRISK = 3,
    };

    void integrateOdometryStereo(cv::Mat &rigid_body_transformation, cv::Mat &frame_pose,
                                 const cv::Mat &rotation,
                                 const cv::Mat &translation_stereo);

    bool isRotationMatrix(cv::Mat &R);

    cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R);
    void display(int frame_id, cv::Mat& trajectory, cv::Mat& pose);
} // end pylon_odometry
