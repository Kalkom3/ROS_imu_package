ROS_IMU_BNO055

node bno055_i2c_node

	Function:
		publish data from accelometer,  gyroscope and geomagnetic sensor from IMU 

	Parameters:
		device - path to i2c device. Default: /dev/i2c-1
		address - adress of imu. Default: 0x28

	Published topics:
		/data - fused data from IMU
		/raw - data from accelometer
		/mag - data from geomagnetic sensor
		/temp - dane z termometer
		/status - system data

	Serviecs:
		/reset - reset the IMU

node bno055_i2c_calibration:

	Function:
		load calibration from calibration file and uploud to IMU.
		when no parameters are passed starting new calibration.

	Parameters:
		path to calibration file
		
	Published topics:

	Serviecs:
