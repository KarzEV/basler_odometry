#include <string>
#include <iostream>

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include "pylon_wrapper.h"

int main(int argc, char **argv) {
    using namespace std::string_literals;

    int exitCode = 0;

    ros::init(argc, argv, "basler_pair");
    Pylon::PylonInitialize();

    ros::NodeHandle nd;

    pylon_odometry::WrapperStereoPair wrapper_stereo_pair(nd,
                                                          "../../src/basler_odometry/config/config_addres.json");
    wrapper_stereo_pair.config_and_start();
}

