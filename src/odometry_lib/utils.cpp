#include "odometry_lib/utils.h"

namespace pylon_odometry {

    void integrateOdometryStereo(cv::Mat &rigid_body_transformation, cv::Mat &frame_pose,
                                 const cv::Mat &rotation, const cv::Mat &translation_stereo) {

        cv::Mat addup = (cv::Mat_<double>(1, 4) << 0, 0, 0, 1);

        cv::hconcat(rotation, translation_stereo, rigid_body_transformation);
        cv::vconcat(rigid_body_transformation, addup, rigid_body_transformation);

        double scale = sqrt((translation_stereo.at<double>(0)) * (translation_stereo.at<double>(0))
                            + (translation_stereo.at<double>(1)) * (translation_stereo.at<double>(1))
                            + (translation_stereo.at<double>(2)) * (translation_stereo.at<double>(2)));

        std::cout << "scale: " << scale << std::endl;

        rigid_body_transformation = rigid_body_transformation.inv();
        if (scale > 0.05 && scale < 10) {
            frame_pose = frame_pose * rigid_body_transformation;
        } else {
            std::cout << "[WARNING] scale below 0.1, or incorrect translation" << std::endl;
        }
    }

    bool isRotationMatrix(cv::Mat &R) {
        cv::Mat Rt;
        transpose(R, Rt);
        cv::Mat shouldBeIdentity = Rt * R;
        cv::Mat I = cv::Mat::eye(3, 3, shouldBeIdentity.type());

        return norm(I, shouldBeIdentity) < 1e-6;
    }

    cv::Vec3f rotationMatrixToEulerAngles(cv::Mat &R) {

        assert(isRotationMatrix(R));

        float sy = sqrt(R.at<double>(0, 0) * R.at<double>(0, 0) + R.at<double>(1, 0) * R.at<double>(1, 0));

        bool singular = sy < 1e-6; // If

        float x, y, z;
        if (!singular) {
            x = atan2(R.at<double>(2, 1), R.at<double>(2, 2));
            y = atan2(-R.at<double>(2, 0), sy);
            z = atan2(R.at<double>(1, 0), R.at<double>(0, 0));
        } else {
            x = atan2(-R.at<double>(1, 2), R.at<double>(1, 1));
            y = atan2(-R.at<double>(2, 0), sy);
            z = 0;
        }
        return cv::Vec3f(x, y, z);
    }

    void display(int frame_id, cv::Mat& trajectory, cv::Mat& pose)
    {
        // draw estimated trajectory
        int x = int(pose.at<double>(0)) + 300;
        int y = int(pose.at<double>(2)) + 100;
        circle(trajectory, cv::Point(x, y) ,1, CV_RGB(255,0,0), 2);
    }
}