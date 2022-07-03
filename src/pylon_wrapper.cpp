#include "pylon_wrapper.h"

#include "stdlib.h"

#include <stdexcept>
#include <chrono>
#include <thread>
#include <vector>
#include <iostream>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/core.hpp>

#include <pylon/Info.h>
#include <pylon/gige/GigETransportLayer.h>
#include <pylon/gige/ActionTriggerConfiguration.h>
#include <pylon/gige/BaslerGigEDeviceInfo.h>

#include "basler_odometry/Basler.h"

using namespace Pylon;
using namespace cv_bridge;
using namespace cv;
using namespace pylon_odometry;

static const uint c_maxCamerasToUse = 2;
static const uint t_durationMilliseconds = 100;

CImageEventPrinter::CImageEventPrinter(const ros::Publisher &camera_publisher) : camera_publisher_(camera_publisher) {
    formatConverter.OutputPixelFormat = PixelType_BGR8packed;
}

void CImageEventPrinter::OnImageGrabbed(CInstantCamera &camera, const CGrabResultPtr &ptrGrabResult) {
    CPylonImage pylonImage;
    formatConverter.Convert(pylonImage, ptrGrabResult);

    CvImage cvImage(std_msgs::Header(),
                    sensor_msgs::image_encodings::RGB8,
                    Mat(ptrGrabResult->GetHeight(), ptrGrabResult->GetWidth(), CV_8UC3,
                        (uint8_t *) pylonImage.GetBuffer()));

//    basler_odometry::Basler new_data;
//    new_data.header.stamp = ros::Time::now();
//    cvImage.toImageMsg(new_data.image_basler);
    sensor_msgs::Image new_data;
    cvImage.toImageMsg(new_data);
    camera_publisher_.publish(new_data);
}

void WrapperStereoPair::config_and_start() {
    config_stereo_pair();
    start_stereo_pair();
}

void WrapperStereoPair::config_stereo_pair() {
    using namespace std::string_literals;
    std::vector<std::string> name_topics;
    name_topics.push_back("/basler_odometry/left"s);
    name_topics.push_back("/basler_odometry/right"s);

    IGigETransportLayer *pTL = dynamic_cast<IGigETransportLayer *>(ctlFactory_.CreateTl(BaslerGigEDeviceClass));
    if (pTL == NULL) {
        throw std::runtime_error("No GigE transport layer available.");
    }

    DeviceInfoList_t allDeviceInfos;
    while (pTL->EnumerateDevices(allDeviceInfos) < c_maxCamerasToUse) {
        std::this_thread::sleep_for(std::chrono::milliseconds(t_durationMilliseconds));
        std::cout << "sleep" << std::endl;
        std::cout << pTL->EnumerateDevices(allDeviceInfos) << std::endl;
    }

    DeviceInfoList_t usableDeviceInfos;
    usableDeviceInfos.push_back(allDeviceInfos[0]);
    subnet_ = allDeviceInfos[0].GetSubnetAddress();
    for (size_t i = 1; i < allDeviceInfos.size() && usableDeviceInfos.size() < c_maxCamerasToUse; ++i) {
        if (subnet_ == allDeviceInfos[i].GetSubnetAddress()) {
            usableDeviceInfos.push_back(allDeviceInfos[i]);
        }
    }

    camera_array_ = std::make_unique<CBaslerUniversalInstantCameraArray>(usableDeviceInfos.size());

    for (size_t i = 0; i < camera_array_->GetSize(); ++i) {
        (*camera_array_)[i].Attach(ctlFactory_.CreateDevice(usableDeviceInfos[i]));
        (*camera_array_)[i].RegisterConfiguration( new CSoftwareTriggerConfiguration, RegistrationMode_ReplaceAll, Cleanup_Delete );
//        (*camera_array_)[i].RegisterConfiguration(
//                new CActionTriggerConfiguration(device_key_, group_key_, AllGroupMask),
//                RegistrationMode_Append, Cleanup_Delete);
//        (*camera_array_)[i].RegisterImageEventHandler(
//                new CImageEventPrinter(stereo_handle_.advertise<basler_odometry::Basler>(name_topics[i], 1)),
//                RegistrationMode_Append, Cleanup_Delete);
        (*camera_array_)[i].RegisterImageEventHandler(
                new CImageEventPrinter(stereo_handle_.advertise<sensor_msgs::Image>(name_topics[i], 2)),
                RegistrationMode_Append, Cleanup_Delete);

        (*camera_array_)[i].GrabCameraEvents = true;
    }
}

void WrapperStereoPair::start_stereo_pair() {
    camera_array_->Open();
    camera_array_->StartGrabbing(GrabStrategy_OneByOne, GrabLoop_ProvidedByInstantCamera);
    std::cout << "start" << std::endl;
//    IGigETransportLayer *pTL = dynamic_cast<IGigETransportLayer *>(ctlFactory_.CreateTl(BaslerGigEDeviceClass));
//    if (pTL == NULL) {
//        throw std::runtime_error("No GigE transport layer available.");
//    }
    ros::Rate rate(10);
    while (camera_array_->IsGrabbing()) {
        if ((*camera_array_)[1].WaitForFrameTriggerReady(100,  TimeoutHandling_ThrowException)) {
//            pTL->IssueActionCommand(device_key_, group_key_, AllGroupMask, subnet_);
            (*camera_array_)[1].ExecuteSoftwareTrigger();
            (*camera_array_)[0].ExecuteSoftwareTrigger();
            ros::spinOnce();
            rate.sleep();
        }
    }

    camera_array_->StopGrabbing();
    camera_array_->Close();
}
