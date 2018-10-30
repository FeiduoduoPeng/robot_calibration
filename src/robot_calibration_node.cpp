#include "../include/robot_calibration/robot_calibration.h"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "robot_calibration_node");

    robot_calibration::RobotCalibrater robot_calibration;

    ros::spin();
    return 0;
}
