#!/usr/bin/env python3
import time
from smbus2 import SMBus
import math
from std_msgs.msg import Float64
import rospy

# gyroZ = 0x47
# gyroY = 0x45
# gyroX = 0x43
# accZ = 0x3F
# accY = 0x3D
# accX = 0x3B



#------------------- Class ---------------------



# Class Mpu : for every Mpu axis
class MpuAxis:

	# constant attributes of MpuAxis for I2C
	power_mgmt_1 = 0x6b
	power_mgmt_2 = 0x6c
	gyro_config = 0x1b
	FS_SEL3 = 16.4
	address = 0x68
	# I2C Parameters
	bus = SMBus(1)
	bus.write_byte_data(address, power_mgmt_1, 0)
	bus.write_byte_data(address, gyro_config, 0x18)


	# constructor of Class MpuAxis
	def __init__(self, axis):

		# Get the I2C Adress of Axis
		self.axisAdress = 0x47

		# Publisher and Rate
		self.stringPubName = 'mpu_' + axis
		self.pub = rospy.Publisher(self.stringPubName, Float64, queue_size = 10)
		self.rate = rospy.Rate(10) # 10Hz


	# Method : read byte of I2C bus
	def read_byte(self, reg):

		return MpuAxis.bus.read_byte_data(MpuAxis.address, reg)


	# Method : read word of I2C bus
	def read_word(self, reg):

		h = MpuAxis.bus.read_byte_data(MpuAxis.address, reg)
		l = MpuAxis.bus.read_byte_data(MpuAxis.address, reg+1)
		value = (h << 8) + l
		return value


	# Method : read filtered word of I2C bus
	def read_word_2c(self, reg):

		val = MpuAxis.read_word(self,reg)
		if val >= 0x8000:
			return -((65535 - val) + 1)
		else:
			return val

	# Method : loop for Publishing of axis Speed
	def run(self):

		while not rospy.is_shutdown():
			self.axisSpeed = self.read_word_2c(self.axisAdress) / MpuAxis.FS_SEL3
			self.pub.publish(self.axisSpeed)
			rospy.loginfo(self.axisSpeed)
			self.rate.sleep()


#------------------- Main ---------------------


if __name__ == '__main__':
	try:
		print("Mpu Node")

		# initialization Node
		rospy.init_node('MpuNode', anonymous=True)

		# creating MpuAxis Object for Z Axis speed
		gyroZ = MpuAxis('gyroZ')
		gyroZ.run()

	except rospy.ROSInterruptException:
		pass

