#!/usr/bin/env python
import rospy
from sensor_msgs.msg import Imu
import smbus2 as smbus
from time import sleep

# Some MPU6050 Registers and their Address
PWR_MGMT_1   = 0x6B
SMPLRT_DIV   = 0x19
CONFIG       = 0x1A
GYRO_CONFIG  = 0x1B
INT_ENABLE   = 0x38
ACCEL_XOUT_H = 0x3B
ACCEL_YOUT_H = 0x3D
ACCEL_ZOUT_H = 0x3F
GYRO_XOUT_H  = 0x43
GYRO_YOUT_H  = 0x45
GYRO_ZOUT_H  = 0x47

def MPU_Init():
    bus.write_byte_data(Device_Address, SMPLRT_DIV, 7)
    bus.write_byte_data(Device_Address, PWR_MGMT_1, 1)
    bus.write_byte_data(Device_Address, CONFIG, 0)
    bus.write_byte_data(Device_Address, GYRO_CONFIG, 24)
    bus.write_byte_data(Device_Address, INT_ENABLE, 1)

def read_raw_data(addr):
    high = bus.read_byte_data(Device_Address, addr)
    low = bus.read_byte_data(Device_Address, addr + 1)
    value = ((high << 8) | low)
    if value > 32768:
        value = value - 65536
    return value

def main():
    global bus, Device_Address
    bus = smbus.SMBus(1)
    Device_Address = 0x68

    MPU_Init()
    rospy.init_node('imu_publisher')
    imu_pub = rospy.Publisher('/imu/data', Imu, queue_size=10)
    rate = rospy.Rate(10)  # 10 Hz

    imu_msg = Imu()

    while not rospy.is_shutdown():
        acc_x = read_raw_data(ACCEL_XOUT_H)
        acc_y = read_raw_data(ACCEL_YOUT_H)
        acc_z = read_raw_data(ACCEL_ZOUT_H)
        gyro_x = read_raw_data(GYRO_XOUT_H)
        gyro_y = read_raw_data(GYRO_YOUT_H)
        gyro_z = read_raw_data(GYRO_ZOUT_H)

        imu_msg.header.stamp = rospy.Time.now()
        imu_msg.header.frame_id = "imu_link"

        imu_msg.linear_acceleration.x = acc_x / 16384.0
        imu_msg.linear_acceleration.y = acc_y / 16384.0
        imu_msg.linear_acceleration.z = acc_z / 16384.0

        imu_msg.angular_velocity.x = gyro_x / 131.0
        imu_msg.angular_velocity.y = gyro_y / 131.0
        imu_msg.angular_velocity.z = gyro_z / 131.0

        imu_pub.publish(imu_msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

