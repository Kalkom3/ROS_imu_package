
/* bno055_i2c_node.cpp
 * Author: Dheera Venkatraman <dheera@dheera.net>
 *
 * Instantiates a BNO055I2C Activity class, as well as
 * a Watchdog that causes this node to die if things aren't
 * working.
 */

#include <imu_bno055/bno055_i2c_activity.h>
#include "watchdog/watchdog.h"
#include <csignal>


int main(int argc, char *argv[]) {
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv("~");

    imu_bno055::BNO055I2CActivity activity(nh,nh_priv);
    watchdog::Watchdog watchdog;

    ros::init(argc, argv, "bno055_node");


    watchdog.start(5000);

    int param_rate;
    nh_priv.param("rate", param_rate, (int)100);
    const char* file_path;
    if(argc == 2)
    {
        file_path=argv[1];
    }
    else if(argc == 1)
    {
        ROS_INFO("No path parameter, Starting whitout calibration");
    }

    if(!activity.calibrate(file_path))
    {
        ROS_INFO("CALIBRATION ERROR");
    }
    else
    {
        ROS_INFO("Calibration loaded!");
    }
    
    ros::Rate rate(param_rate);
    while(ros::ok()) {
        rate.sleep();
        if(activity.spinOnce()) {
            watchdog.refresh();
        }
    }

    activity.stop();
    watchdog.stop();

    return 0;
}
