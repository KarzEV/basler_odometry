#pragma once

#include <string>
#include <memory>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

#include <pylon/PylonIncludes.h>
#include <pylon/TlFactory.h>
#include <pylon/BaslerUniversalInstantCameraArray.h>

namespace pylon_odometry {

    class CImageEventPrinter : public Pylon::CImageEventHandler {
    public:
        CImageEventPrinter(const ros::Publisher &camera_publisher);

        CImageEventPrinter(const CImageEventPrinter &other) = delete;

    public:
        void OnImageGrabbed(Pylon::CInstantCamera &camera,
                            const Pylon::CGrabResultPtr &ptrGrabResult) override;

    private:
        Pylon::CImageFormatConverter formatConverter;
        ros::Publisher camera_publisher_;
    };

    class WrapperStereoPair {
    public:
        WrapperStereoPair(ros::NodeHandle stereo_handle, std::string path_to_conf) :
                ctlFactory_(Pylon::CTlFactory::GetInstance()),
                stereo_handle_(stereo_handle),
                device_key_(std::rand()),
                group_key_(0x112233) {
        }


        WrapperStereoPair(const WrapperStereoPair &other) = delete;

    public:
        void config_and_start();

        void config_stereo_pair();

        void start_stereo_pair();

    private:
        Pylon::CTlFactory &ctlFactory_;
        std::unique_ptr<Pylon::CBaslerUniversalInstantCameraArray> camera_array_;

        ros::NodeHandle stereo_handle_;

        Pylon::String_t subnet_;
        const uint32_t device_key_;
        const uint32_t group_key_;
    };
} // end pylon_odometry
