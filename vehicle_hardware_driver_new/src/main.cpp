#include <stdlib.h>
#include <stdio.h>
#include "vehicle_hardware_driver.h"

/***************************************
*Function: main
***************************************/
int main(int argc, char **argv)
{
    ros::init(argc, argv, "autobots_driver");
    ros::Time::init();

    HardwareNS::VehicleHardwareDriver chassis;
    
    chassis.start();

    ROS_INFO("game over!");
    // ros::spin();
    
    return 0;
}