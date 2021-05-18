
#include "imu_bno055/bno055_i2c_activity.h"
typedef unsigned char BYTE;
// read calibration from file
std::vector<BYTE> readFile(const char* filename)
{
    // open the file:
    std::streampos fileSize;
    std::ifstream file(filename, std::ios::binary);
   // std::printf("opening calibration file...");
    if (!file.is_open())
    {
        throw 1;
    }

   
    // get its size:
    file.seekg(0, std::ios::end);
    fileSize = file.tellg();
    file.seekg(0, std::ios::beg);

    // read the data:
    std::vector<BYTE> fileData(fileSize);
    file.read((char*)&fileData[0], fileSize);
    for(auto it = fileData.begin();it!=fileData.end();it++)
    //std::printf("%d\n",*it);
    return fileData;
}
// write calibration to file
void writeFile(std::vector<BYTE> data, const char* filename)
{
    std::ofstream file(filename, std::ios::binary);
    file.write((char *)&data[0],data.size());
    file.close();
}

namespace imu_bno055 {

    // ******** constructors ******** //

    BNO055I2CActivity::BNO055I2CActivity(ros::NodeHandle &_nh, ros::NodeHandle &_nh_priv) :
    nh(_nh), nh_priv(_nh_priv) {
        ROS_INFO("initializing");
        nh_priv.param("device", param_device, (std::string)"/dev/i2c-1");
        nh_priv.param("address", param_address, (int)BNO055_ADDRESS_A);
        nh_priv.param("frame_id", param_frame_id, (std::string)"imu");

        current_status.level = 0;
        current_status.name = "BNO055 IMU";
        current_status.hardware_id = "bno055_i2c";

        diagnostic_msgs::KeyValue calib_stat;
        calib_stat.key = "Calibration status";
        calib_stat.value = "";
        current_status.values.push_back(calib_stat);

        diagnostic_msgs::KeyValue selftest_result;
        selftest_result.key = "Self-test (clibration) result";
        selftest_result.value = "";
        current_status.values.push_back(selftest_result);

        diagnostic_msgs::KeyValue intr_stat;
        intr_stat.key = "Interrupt status";
        intr_stat.value = "";
        current_status.values.push_back(intr_stat);

        diagnostic_msgs::KeyValue sys_clk_stat;
        sys_clk_stat.key = "System clock status";
        sys_clk_stat.value = "";
        current_status.values.push_back(sys_clk_stat);

        diagnostic_msgs::KeyValue sys_stat;
        sys_stat.key = "System status";
        sys_stat.value = "";
        current_status.values.push_back(sys_stat);

        diagnostic_msgs::KeyValue sys_err;
        sys_err.key = "System error";
        sys_err.value = "";
        current_status.values.push_back(sys_err);
    }

    // ******** private methods ******** //

    bool BNO055I2CActivity::reset() {
        int i = 0;

        _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
        ros::Duration(0.025).sleep();

        // reset
        _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0x20);
        ros::Duration(0.025).sleep();

        // wait for chip to come back online
        while(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
            ros::Duration(0.010).sleep();
            if(i++ > 500) {
                ROS_ERROR_STREAM("chip did not come back online in 5 seconds after reset");
                return false;
            }
        }
        ros::Duration(0.100).sleep();

        // normal power mode
        _i2c_smbus_write_byte_data(file, BNO055_PWR_MODE_ADDR, BNO055_POWER_MODE_NORMAL);
        ros::Duration(0.010).sleep();

        _i2c_smbus_write_byte_data(file, BNO055_PAGE_ID_ADDR, 0);
        _i2c_smbus_write_byte_data(file, BNO055_SYS_TRIGGER_ADDR, 0);
        ros::Duration(0.025).sleep();

        _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
        ros::Duration(0.025).sleep();

        return true;
    }



    // ******** public methods ******** //

    bool BNO055I2CActivity::start() {
        ROS_INFO("starting");

        if(!pub_data) pub_data = nh.advertise<sensor_msgs::Imu>("data", 1);
        if(!pub_raw) pub_raw = nh.advertise<sensor_msgs::Imu>("raw", 1);
        if(!pub_mag) pub_mag = nh.advertise<sensor_msgs::MagneticField>("mag", 1);
        if(!pub_temp) pub_temp = nh.advertise<sensor_msgs::Temperature>("temp", 1);
        if(!pub_status) pub_status = nh.advertise<diagnostic_msgs::DiagnosticStatus>("status", 1);

        if(!service_calibrate) service_calibrate = nh.advertiseService(
            "calibrate",
            &BNO055I2CActivity::onServiceCalibrate,
            this
        );

        if(!service_reset) service_reset = nh.advertiseService(
            "reset",
            &BNO055I2CActivity::onServiceReset,
            this
        );

        file = open(param_device.c_str(), O_RDWR);
        if(ioctl(file, I2C_SLAVE, param_address) < 0) {
            ROS_ERROR("i2c device open failed");
            return false;
        }

        if(_i2c_smbus_read_byte_data(file, BNO055_CHIP_ID_ADDR) != BNO055_ID) {
            ROS_ERROR("incorrect chip ID");
            return false;
        }

        ROS_INFO_STREAM("rev ids:"
        << " accel:" << _i2c_smbus_read_byte_data(file, BNO055_ACCEL_REV_ID_ADDR)
        << " mag:" << _i2c_smbus_read_byte_data(file, BNO055_MAG_REV_ID_ADDR)
        << " gyro:" << _i2c_smbus_read_byte_data(file, BNO055_GYRO_REV_ID_ADDR)
        << " sw:" << _i2c_smbus_read_word_data(file, BNO055_SW_REV_ID_LSB_ADDR)
        << " bl:" << _i2c_smbus_read_byte_data(file, BNO055_BL_REV_ID_ADDR));

        if(!reset()) {
            ROS_ERROR("chip reset and setup failed");
            return false;
        }

        return true;
    }

    bool BNO055I2CActivity::spinOnce() {
        ros::spinOnce();

        ros::Time time = ros::Time::now();

        IMURecord record;

        unsigned char c = 0;

        seq++;

        // can only read a length of 0x20 at a time, so do it in 2 reads
        // BNO055_LINEAR_ACCEL_DATA_X_LSB_ADDR is the start of the data block that aligns with the IMURecord struct
        if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR, 0x20, (uint8_t*)&record) != 0x20) {
            return false;
        }
        if(_i2c_smbus_read_i2c_block_data(file, BNO055_ACCEL_DATA_X_LSB_ADDR + 0x20, 0x13, (uint8_t*)&record + 0x20) != 0x13) {
            return false;
        }

        sensor_msgs::Imu msg_raw;
        msg_raw.header.stamp = time;
        msg_raw.header.frame_id = param_frame_id;
        msg_raw.header.seq = seq;
        msg_raw.linear_acceleration.x = (double)record.raw_linear_acceleration_x / 100.0;
        msg_raw.linear_acceleration.y = (double)record.raw_linear_acceleration_y / 100.0;
        msg_raw.linear_acceleration.z = (double)record.raw_linear_acceleration_z / 100.0;
        msg_raw.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
        msg_raw.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
        msg_raw.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

        sensor_msgs::MagneticField msg_mag;
        msg_mag.header.stamp = time;
        msg_mag.header.frame_id = param_frame_id;
        msg_mag.header.seq = seq;
        msg_mag.magnetic_field.x = (double)record.raw_magnetic_field_x / 16.0;
        msg_mag.magnetic_field.y = (double)record.raw_magnetic_field_y / 16.0;
        msg_mag.magnetic_field.z = (double)record.raw_magnetic_field_z / 16.0;

        sensor_msgs::Imu msg_data;
        msg_data.header.stamp = time;
        msg_data.header.frame_id = param_frame_id;
        msg_data.header.seq = seq;

        double fused_orientation_norm = std::pow(
        std::pow(record.fused_orientation_w, 2) +
        std::pow(record.fused_orientation_x, 2) +
        std::pow(record.fused_orientation_y, 2) +
        std::pow(record.fused_orientation_z, 2), 0.5);

        msg_data.orientation.w = (double)record.fused_orientation_w / fused_orientation_norm;
        msg_data.orientation.x = (double)record.fused_orientation_x / fused_orientation_norm;
        msg_data.orientation.y = (double)record.fused_orientation_y / fused_orientation_norm;
        msg_data.orientation.z = (double)record.fused_orientation_z / fused_orientation_norm;
        msg_data.linear_acceleration.x = (double)record.fused_linear_acceleration_x / 100.0;
        msg_data.linear_acceleration.y = (double)record.fused_linear_acceleration_y / 100.0;
        msg_data.linear_acceleration.z = (double)record.fused_linear_acceleration_z / 100.0;
        msg_data.angular_velocity.x = (double)record.raw_angular_velocity_x / 900.0;
        msg_data.angular_velocity.y = (double)record.raw_angular_velocity_y / 900.0;
        msg_data.angular_velocity.z = (double)record.raw_angular_velocity_z / 900.0;

        sensor_msgs::Temperature msg_temp;
        msg_temp.header.stamp = time;
        msg_temp.header.frame_id = param_frame_id;
        msg_temp.header.seq = seq;
        msg_temp.temperature = (double)record.temperature;

        pub_data.publish(msg_data);
        pub_raw.publish(msg_raw);
        pub_mag.publish(msg_mag);
        pub_temp.publish(msg_temp);

        
        if(seq % 50 == 0) {

            current_status.values[DIAG_CALIB_STAT].value = std::to_string(record.calibration_status);
            current_status.values[DIAG_SELFTEST_RESULT].value = std::to_string(record.self_test_result);
            current_status.values[DIAG_INTR_STAT].value = std::to_string(record.interrupt_status);
            current_status.values[DIAG_SYS_CLK_STAT].value = std::to_string(record.system_clock_status);
            current_status.values[DIAG_SYS_STAT].value = std::to_string(record.system_status);
            current_status.values[DIAG_SYS_ERR].value = std::to_string(record.system_error_code);
            pub_status.publish(current_status);
        }

        return true;    
    }

    bool BNO055I2CActivity::stop() {
        ROS_INFO("stopping");

        if(pub_data) pub_data.shutdown();
        if(pub_raw) pub_raw.shutdown();
        if(pub_mag) pub_mag.shutdown();
        if(pub_temp) pub_temp.shutdown();
        if(pub_status) pub_status.shutdown();

        if(service_calibrate) service_calibrate.shutdown();
        if(service_reset) service_reset.shutdown();

        return true;
    }

    bool BNO055I2CActivity::onServiceReset(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
        if(!reset()) {
            ROS_ERROR("chip reset and setup failed");
            return false;
        }
        return true;
    }

    bool BNO055I2CActivity::onServiceCalibrate(std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res) {
            
        return true;
    }

    //----------calibrate----------------
    bool BNO055I2CActivity::calibrate(const char* file_path) {
        _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
        ros::Duration(0.025).sleep();
        //calib_info();
        init_calibration();
        if(!load_calibration(file_path))
        {
            ROS_INFO("No calibration file in %s\n",file_path);
            std::vector<BYTE>calib_value;
            new_calibration();
            for(int i=0;i<22;i++)
            {
                calib_value.push_back(_i2c_smbus_read_byte_data(file,BNO055_ACCEL_OFFSET_X_LSB_ADDR+i));
            }
            
            std::string calib_file_path = __FILE__;
            std::string calib_localization = calib_file_path.substr(0, calib_file_path.rfind("/src"));
            calib_localization+="/config.NDOF_calibration.bin";
            writeFile(calib_value,calib_localization.c_str() );
        }

        calib_info();
        _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
        ros::Duration(0.025).sleep();
        return true;
    }

    void BNO055I2CActivity::init_calibration()
    {
    ROS_INFO("The IMU will be calibrated to work in NODF mode");
    }

    void BNO055I2CActivity::calib_info()
    {
        
        uint16_t response[6];
        response[0] = _i2c_smbus_read_byte_data(file,BNO055_ACCEL_OFFSET_X_LSB_ADDR);
        response[1] = _i2c_smbus_read_byte_data(file,BNO055_ACCEL_OFFSET_X_MSB_ADDR);
        response[2] = _i2c_smbus_read_byte_data(file,BNO055_ACCEL_OFFSET_Y_LSB_ADDR);
        response[3] = _i2c_smbus_read_byte_data(file,BNO055_ACCEL_OFFSET_Y_MSB_ADDR);
        response[4] = _i2c_smbus_read_byte_data(file,BNO055_ACCEL_OFFSET_Z_LSB_ADDR);
        response[5] = _i2c_smbus_read_byte_data(file,BNO055_ACCEL_OFFSET_Z_MSB_ADDR);
        if(imu_calibrated)
        {
            ROS_INFO("IMU is calibrated\n");
            
        }
        else
        {
            ROS_INFO("IMU is not calibrated\n");
        }
        ROS_INFO("CALIBRATION DATA X: %d, %d Y: %d, %d Z: %d, %d \n",response[0],response[1],response[2],response[3],response[4],response[5]);
        
    }

    bool BNO055I2CActivity::load_calibration(const char* file_path)
    {
        imu_calibrated=false;
        std::vector<BYTE>calib_vec;
        try
        {
            calib_vec = readFile(file_path);
        }
        catch (int e)
        {
            
            return imu_calibrated;
        }
        
        
        short i=0;
        for(auto it=begin(calib_vec);it != end(calib_vec); it++)
        {
            _i2c_smbus_write_byte_data(file,BNO055_ACCEL_OFFSET_X_LSB_ADDR+i,*it);
            i++;
        }
        imu_calibrated=true;
        ROS_INFO("Calibration loaded sucessfully!\n");
        return imu_calibrated;    

    }

    bool BNO055I2CActivity::new_calibration()
    {
        _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_NDOF);
        ros::Duration(0.025).sleep();
        uint16_t status;
        ros::Rate r(2);
        while(!imu_calibrated && ros::ok())
        {
            status=_i2c_smbus_read_byte_data(file,BNO055_CALIB_STAT_ADDR);
            calibration_status[0]=(status>>6)& 0b11; //system
            calibration_status[1]=(status>>4)& 0b11; //gyroscope  
            calibration_status[2]=(status>>2)& 0b11; //accelmeter
            calibration_status[3]=(status>>0)& 0b11; //magnetometer
            ROS_INFO("[ALL: %d]",status);
            ROS_INFO(" [System: %d]" , calibration_status[0]);
            ROS_INFO(" [Gyroscope: %d] " , calibration_status[1]) ;
            ROS_INFO(" [Accelerometer: %d]" , calibration_status[2]);
            ROS_INFO(" [Magnetometer: %d]\n" ,calibration_status[3]);
            r.sleep();
            if(calibration_status[0]==3 && calibration_status[1]==3 && calibration_status[2]==3 && calibration_status[3]==3)
            {
                imu_calibrated=true;
            }           
        }
        _i2c_smbus_write_byte_data(file, BNO055_OPR_MODE_ADDR, BNO055_OPERATION_MODE_CONFIG);
        ros::Duration(0.025).sleep();
            return imu_calibrated;

    }
}
