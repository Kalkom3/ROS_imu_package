#ifndef _bno055_i2c_activity_dot_h
#define _bno055_i2c_activity_dot_h

#include <ros/ros.h>
#include <cstdlib>
#include <cerrno>
#include <cstring>
#include <sys/ioctl.h>
#include <fcntl.h>
#include <fstream>
#include <string>

#include <sensor_msgs/Imu.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/Temperature.h>
#include <std_srvs/Trigger.h>
#include <std_msgs/UInt8.h>
#include <diagnostic_msgs/DiagnosticStatus.h>
#include <diagnostic_msgs/DiagnosticArray.h>
#include <diagnostic_msgs/KeyValue.h>

#include <linux/i2c-dev.h>
#include <smbus_functions.h>

#define BNO055_ID 0xA0

#define BNO055_ADDRESS_A 0x28 // default
#define BNO055_ADDRESS_B 0x29

#define BNO055_CHIP_ID_ADDR 0x00
#define BNO055_ACCEL_REV_ID_ADDR 0x01
#define BNO055_MAG_REV_ID_ADDR 0x02
#define BNO055_GYRO_REV_ID_ADDR 0x03
#define BNO055_SW_REV_ID_LSB_ADDR 0x04
#define BNO055_SW_REV_ID_MSB_ADDR 0x05
#define BNO055_BL_REV_ID_ADDR 0x06
#define BNO055_PAGE_ID_ADDR 0x07

#define BNO055_ACCEL_DATA_X_LSB_ADDR 0x08
#define BNO055_ACCEL_DATA_X_MSB_ADDR 0x09
#define BNO055_ACCEL_DATA_Y_LSB_ADDR 0x0A
#define BNO055_ACCEL_DATA_Y_MSB_ADDR 0x0B
#define BNO055_ACCEL_DATA_Z_LSB_ADDR 0x0C
#define BNO055_ACCEL_DATA_Z_MSB_ADDR 0x0D

#define BNO055_MAG_DATA_X_LSB_ADDR 0x0E
#define BNO055_MAG_DATA_X_MSB_ADDR 0x0F
#define BNO055_MAG_DATA_Y_LSB_ADDR 0x10
#define BNO055_MAG_DATA_Y_MSB_ADDR 0x11
#define BNO055_MAG_DATA_Z_LSB_ADDR 0x12
#define BNO055_MAG_DATA_Z_MSB_ADDR 0x13

#define BNO055_GYRO_DATA_X_LSB_ADDR 0x14
#define BNO055_GYRO_DATA_X_MSB_ADDR 0x15
#define BNO055_GYRO_DATA_Y_LSB_ADDR 0x16
#define BNO055_GYRO_DATA_Y_MSB_ADDR 0x17
#define BNO055_GYRO_DATA_Z_LSB_ADDR 0x18
#define BNO055_GYRO_DATA_Z_MSB_ADDR 0x19

#define BNO055_EULER_H_LSB_ADDR 0x1A
#define BNO055_EULER_H_MSB_ADDR 0x1B
#define BNO055_EULER_R_LSB_ADDR 0x1C
#define BNO055_EULER_R_MSB_ADDR 0x1D
#define BNO055_EULER_P_LSB_ADDR 0x1E
#define BNO055_EULER_P_MSB_ADDR 0x1F

#define BNO055_QUATERNION_DATA_W_LSB_ADDR 0x20
#define BNO055_QUATERNION_DATA_W_MSB_ADDR 0x21
#define BNO055_QUATERNION_DATA_X_LSB_ADDR 0x22
#define BNO055_QUATERNION_DATA_X_MSB_ADDR 0x23
#define BNO055_QUATERNION_DATA_Y_LSB_ADDR 0x24
#define BNO055_QUATERNION_DATA_Y_MSB_ADDR 0x25
#define BNO055_QUATERNION_DATA_Z_LSB_ADDR 0x26
#define BNO055_QUATERNION_DATA_Z_MSB_ADDR 0x27

#define BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR 0x28
#define BNO055_LINEAR_ACCEL_DATA_X_MSB_ADDR 0x29
#define BNO055_LINEAR_ACCEL_DATA_Y_LSB_ADDR 0x2A
#define BNO055_LINEAR_ACCEL_DATA_Y_MSB_ADDR 0x2B
#define BNO055_LINEAR_ACCEL_DATA_Z_LSB_ADDR 0x2C
#define BNO055_LINEAR_ACCEL_DATA_Z_MSB_ADDR 0x2D

#define BNO055_GRAVITY_DATA_X_LSB_ADDR 0x2E
#define BNO055_GRAVITY_DATA_X_MSB_ADDR 0x2F
#define BNO055_GRAVITY_DATA_Y_LSB_ADDR 0x30
#define BNO055_GRAVITY_DATA_Y_MSB_ADDR 0x31
#define BNO055_GRAVITY_DATA_Z_LSB_ADDR 0x32
#define BNO055_GRAVITY_DATA_Z_MSB_ADDR 0x33

#define BNO055_TEMP_ADDR 0x34

#define BNO055_CALIB_STAT_ADDR 0x35
#define BNO055_SELFTEST_RESULT_ADDR 0x36
#define BNO055_INTR_STAT_ADDR 0x37

#define BNO055_SYS_CLK_STAT_ADDR 0x38
#define BNO055_SYS_STAT_ADDR 0x39
#define BNO055_SYS_ERR_ADDR 0x3A

#define BNO055_UNIT_SEL_ADDR 0x3B
#define BNO055_DATA_SELECT_ADDR 0x3C

#define BNO055_OPR_MODE_ADDR 0x3D
#define BNO055_PWR_MODE_ADDR 0x3E

#define BNO055_SYS_TRIGGER_ADDR 0x3F
#define BNO055_TEMP_SOURCE_ADDR 0x40

#define BNO055_AXIS_MAP_CONFIG_ADDR 0x41
#define BNO055_AXIS_MAP_SIGN_ADDR 0x42

#define BNO055_SIC_MATRIX_0_LSB_ADDR 0x43
#define BNO055_SIC_MATRIX_0_MSB_ADDR 0x44
#define BNO055_SIC_MATRIX_1_LSB_ADDR 0x45
#define BNO055_SIC_MATRIX_1_MSB_ADDR 0x46
#define BNO055_SIC_MATRIX_2_LSB_ADDR 0x47
#define BNO055_SIC_MATRIX_2_MSB_ADDR 0x48
#define BNO055_SIC_MATRIX_3_LSB_ADDR 0x49
#define BNO055_SIC_MATRIX_3_MSB_ADDR 0x4A
#define BNO055_SIC_MATRIX_4_LSB_ADDR 0x4B
#define BNO055_SIC_MATRIX_4_MSB_ADDR 0x4C
#define BNO055_SIC_MATRIX_5_LSB_ADDR 0x4D
#define BNO055_SIC_MATRIX_5_MSB_ADDR 0x4E
#define BNO055_SIC_MATRIX_6_LSB_ADDR 0x4F
#define BNO055_SIC_MATRIX_6_MSB_ADDR 0x50
#define BNO055_SIC_MATRIX_7_LSB_ADDR 0x51
#define BNO055_SIC_MATRIX_7_MSB_ADDR 0x52
#define BNO055_SIC_MATRIX_8_LSB_ADDR 0x53
#define BNO055_SIC_MATRIX_8_MSB_ADDR 0x54

#define BNO055_ACCEL_OFFSET_X_LSB_ADDR 0x55
#define BNO055_ACCEL_OFFSET_X_MSB_ADDR 0x56
#define BNO055_ACCEL_OFFSET_Y_LSB_ADDR 0x57
#define BNO055_ACCEL_OFFSET_Y_MSB_ADDR 0x58
#define BNO055_ACCEL_OFFSET_Z_LSB_ADDR 0x59
#define BNO055_ACCEL_OFFSET_Z_MSB_ADDR 0x5A

#define BNO055_MAG_OFFSET_X_LSB_ADDR 0x5B
#define BNO055_MAG_OFFSET_X_MSB_ADDR 0x5C
#define BNO055_MAG_OFFSET_Y_LSB_ADDR 0x5D
#define BNO055_MAG_OFFSET_Y_MSB_ADDR 0x5E
#define BNO055_MAG_OFFSET_Z_LSB_ADDR 0x5F
#define BNO055_MAG_OFFSET_Z_MSB_ADDR 0x60

#define BNO055_GYRO_OFFSET_X_LSB_ADDR 0x61
#define BNO055_GYRO_OFFSET_X_MSB_ADDR 0x62
#define BNO055_GYRO_OFFSET_Y_LSB_ADDR 0x63
#define BNO055_GYRO_OFFSET_Y_MSB_ADDR 0x64
#define BNO055_GYRO_OFFSET_Z_LSB_ADDR 0x65
#define BNO055_GYRO_OFFSET_Z_MSB_ADDR 0x66

#define BNO055_ACCEL_RADIUS_LSB_ADDR 0x67
#define BNO055_ACCEL_RADIUS_MSB_ADDR 0x68
#define BNO055_MAG_RADIUS_LSB_ADDR 0x69
#define BNO055_MAG_RADIUS_MSB_ADDR 0x6A

#define BNO055_POWER_MODE_NORMAL 0x00
#define BNO055_POWER_MODE_LOWPOWER 0x01
#define BNO055_POWER_MODE_SUSPEND 0x02

#define BNO055_OPERATION_MODE_CONFIG 0x00
#define BNO055_OPERATION_MODE_ACCONLY 0x01
#define BNO055_OPERATION_MODE_MAGONLY 0x02
#define BNO055_OPERATION_MODE_GYRONLY 0x03
#define BNO055_OPERATION_MODE_ACCMAG 0x04
#define BNO055_OPERATION_MODE_ACCGYRO 0x05
#define BNO055_OPERATION_MODE_MAGGYRO 0x06
#define BNO055_OPERATION_MODE_AMG 0x07
#define BNO055_OPERATION_MODE_IMUPLUS 0x08
#define BNO055_OPERATION_MODE_COMPASS 0x09
#define BNO055_OPERATION_MODE_M4G 0x0A
#define BNO055_OPERATION_MODE_NDOF_FMC_OFF 0x0B
#define BNO055_OPERATION_MODE_NDOF 0x0C

#define BNO055_REMAP_CONFIG_P0 0x21
#define BNO055_REMAP_CONFIG_P1 0x24 // default
#define BNO055_REMAP_CONFIG_P2 0x24
#define BNO055_REMAP_CONFIG_P3 0x21
#define BNO055_REMAP_CONFIG_P4 0x24
#define BNO055_REMAP_CONFIG_P5 0x21
#define BNO055_REMAP_CONFIG_P6 0x21
#define BNO055_REMAP_CONFIG_P7 0x24

#define BNO055_REMAP_SIGN_P0 0x04
#define BNO055_REMAP_SIGN_P1 0x00 // default
#define BNO055_REMAP_SIGN_P2 0x06
#define BNO055_REMAP_SIGN_P3 0x02
#define BNO055_REMAP_SIGN_P4 0x03
#define BNO055_REMAP_SIGN_P5 0x01
#define BNO055_REMAP_SIGN_P6 0x07
#define BNO055_REMAP_SIGN_P7 0x05

#define DIAG_CALIB_STAT 0
#define DIAG_SELFTEST_RESULT 1
#define DIAG_INTR_STAT 2
#define DIAG_SYS_CLK_STAT 3
#define DIAG_SYS_STAT 4
#define DIAG_SYS_ERR 5


namespace imu_bno055 {


typedef struct {
  int16_t raw_linear_acceleration_x;
  int16_t raw_linear_acceleration_y;
  int16_t raw_linear_acceleration_z;
  int16_t raw_magnetic_field_x;
  int16_t raw_magnetic_field_y;
  int16_t raw_magnetic_field_z;
  int16_t raw_angular_velocity_x;
  int16_t raw_angular_velocity_y;
  int16_t raw_angular_velocity_z;
  int16_t fused_heading;
  int16_t fused_roll;
  int16_t fused_pitch;
  int16_t fused_orientation_w;
  int16_t fused_orientation_x;
  int16_t fused_orientation_y;
  int16_t fused_orientation_z;
  int16_t fused_linear_acceleration_x;
  int16_t fused_linear_acceleration_y;
  int16_t fused_linear_acceleration_z;
  int16_t gravity_vector_x;
  int16_t gravity_vector_y;
  int16_t gravity_vector_z;
  int8_t temperature;
  uint8_t calibration_status;
  uint8_t self_test_result;
  uint8_t interrupt_status;
  uint8_t system_clock_status;
  uint8_t system_status;
  uint8_t system_error_code;
} IMURecord;

class BNO055I2CActivity {
  public:
    BNO055I2CActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv);

    bool start();
    bool stop();
    bool spinOnce();

    bool onServiceReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool onServiceCalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res);
    bool calibrate(const char* file_path);
  private:
    bool reset();
    
    void init_calibration();
    void calib_info();
    bool load_calibration(const char* file_path);
    bool new_calibration();

    // class variables
    uint32_t seq = 0;
    int file;
    int calibration_full_counter = 0;
    uint16_t calibration_status[4];
    bool imu_calibrated=false;
    diagnostic_msgs::DiagnosticStatus current_status;

    // ROS parameters
    std::string param_frame_id;
    std::string param_device;
    int param_address;

    // ROS node handles
    ros::NodeHandle nh;
    ros::NodeHandle nh_priv;

    // ROS publishers
    ros::Publisher pub_data;
    ros::Publisher pub_raw;
    ros::Publisher pub_mag;
    ros::Publisher pub_temp;
    ros::Publisher pub_status;

    // ROS subscribers
    ros::ServiceServer service_calibrate;
    ros::ServiceServer service_reset;
};

}

#endif // _bno055_i2c_activity_dot_h
