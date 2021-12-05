#! /usr/bin/env python
# coding: utf-8
"""
This is Ros node for reading data from encoders via serial port
"""
import time
import math
import serial
#import rclpy
#from geometry_msgs.msg import Twist
#from rclpy.qos import qos_profile_sensor_data,qos_profile_system_default
#from rclpy.qos import QoSReliabilityPolicy

#import EngineVel.msg


#from std_msgs.msg import Float64

#Velocity=Twist()
vel1=float()
vel2=float()

msg=str()
i=0
platform_width=float()


encoder_vel="/jetson/cmd_vel"
"""
def clb(data):
	global Velocity
	Velocity=data

def velocity_on_wheels_calculation():
	global vel1,vel2,Velocity
	x=Velocity.linear.x
	fi_vel=Velocity.angular.z
	vel1=Velocity.linear.x
	vel2=Velocity.angular.z
	vel1 = float('{:.2f}'.format(vel1))
	vel2 = float('{:.2f}'.format(vel2))
"""

def main(args=None):
	global i,odometry_vel,vel1,vel2,vel3,vel4,Velocity
	"""
	rclpy.init(args=args)
	node=rclpy.create_node("serial_pwm")
	node.create_subscription(Twist,encoder_vel,clb,qos_profile=qos_profile_system_default)
	"""

	ser = serial.Serial(
		port='/dev/ttyACM0',
		baudrate = 115200,
		bytesize = serial.EIGHTBITS,
		parity = serial.PARITY_NONE,
		stopbits = serial.STOPBITS_ONE,
		xonxoff = False,
		rtscts = False,
		dsrdtr = False,
		timeout = 0.1)

	try:
		ser.isOpen()
	except:
		print("serial not open")
		exit(0)
	while True:
		#rclpy.spin_once(node)
		#velocity_on_wheels_calculation()
		msg="<"+"1"+","+str(2.0)+","+str(3.0)+","+str(4.0)+","+str(5.0)+">"
		print("MSG ",msg)

		ser.write(msg.encode())

if __name__ == "__main__":
	main()

