#include <mainpp.h>
#include <ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/Imu.h>

#include "MPU6050.h"

ros::NodeHandle nh;

sensor_msgs::Imu imu_msg;
ros::Publisher imu_pub("imu", &imu_msg);

MPU6050_t mpu6050;

static void ros_setup(void);
static void ros_publish_imu(void);

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		nh.getHardware()->flush();
	}
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	if (huart->Instance == USART2)
	{
		nh.getHardware()->reset_rbuf();
	}
}

void setup(void)
{
	ros_setup();
	MPU6050_init();
	MPU6050_calib(&mpu6050);
	HAL_Delay(500);
}

void loop(void)
{
	if (!nh.connected())
	{
		nh.negotiateTopics();
	}
	MPU6050_read_all(&mpu6050);
	ros_publish_imu();
	nh.spinOnce();
	HAL_Delay(1000);
}

static void ros_setup(void)
{
	nh.initNode();
	nh.loginfo("Init node successfully!");

	nh.advertise(imu_pub);              /*!< Register the publisher to "imu" topic */
}

static void ros_publish_imu(void)
{
	imu_msg.angular_velocity.x = mpu6050.gyro_X;
	imu_msg.angular_velocity.y = mpu6050.gyro_Y;
	imu_msg.angular_velocity.z = mpu6050.gyro_Z;

	imu_msg.linear_acceleration.x = mpu6050.accel_X;
	imu_msg.linear_acceleration.y = mpu6050.accel_Y;
	imu_msg.linear_acceleration.z = mpu6050.accel_Z;

	ros::Time stamp_now = nh.now();
	imu_msg.header.stamp = stamp_now;
	imu_pub.publish(&imu_msg);
}
