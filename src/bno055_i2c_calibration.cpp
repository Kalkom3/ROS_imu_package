#include <imu_bno055/bno055_i2c_activity.h>
//#include "watchdog/watchdog.h"
//#include <csignal>
typedef unsigned char BYTE;

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "imu_calibration_node");
    const char* file_path;
    
    if(argc == 2)
    {
        file_path=argv[1];
    }
    else if(argc == 1)
    {
        ROS_INFO("No path parameter, loading default path!");
        std::string calib_file_path = __FILE__;
        std::string calib_localization = calib_file_path.substr(0, calib_file_path.rfind("/src"));
        calib_localization+="/config.NDOF_calibration.bin";
        file_path=calib_file_path.c_str();
    }
    else
    {
        ROS_ERROR("To many parameters(2)");
        return -1;
    }

    ros::NodeHandle nh;
    ros::NodeHandle nh_pv("~");
    

    imu_bno055::BNO055I2CActivity activity(nh,nh_pv);


    if(!activity.start()) {
        ROS_ERROR("Failed to start activity");

        ros::shutdown();
        return -2;
    }

    if(!activity.calibrate(file_path))
    {
        ROS_ERROR("CALIBRATION ERROR");
        return -3;
    }



    activity.stop();

    return 0;
}


