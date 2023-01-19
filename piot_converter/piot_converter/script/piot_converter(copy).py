#! /usr/bin/env python
import rclpy
import threading
import math
import numpy as np
from math import sin, cos
from rclpy.qos import QoSProfile
from piot_can_msgs.msg import CtrlCmd, CtrlFb
from geometry_msgs.msg import Point, Pose, Quaternion, Twist, Vector3, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
from tf2_ros import TransformBroadcaster


def quaternion_from_euler(ai, aj, ak):
    ai /= 2.0
    aj /= 2.0
    ak /= 2.0
    ci = math.cos(ai)
    si = math.sin(ai)
    cj = math.cos(aj)
    sj = math.sin(aj)
    ck = math.cos(ak)
    sk = math.sin(ak)
    cc = ci*ck
    cs = ci*sk
    sc = si*ck
    ss = si*sk

    q = np.empty((4, ))
    q[0] = cj*sc - sj*cs
    q[1] = cj*ss + sj*cc
    q[2] = cj*cs - sj*sc
    q[3] = cj*cc + sj*ss

    return q

def cmd_vel_callback(msg):
	global ctrl_cmd_gear
	global ctrl_cmd_linear
	global ctrl_cmd_angular
	global ctrl_cmd_slipangle

	if mode_flag == True:
		ctrl_cmd_gear = 6 # Gear 4T4D
		ctrl_cmd_linear = msg.linear.x
		ctrl_cmd_angular = msg.angular.z * 57.2958
		ctrl_cmd_slipangle = 0.0

	else:
		ctrl_cmd_gear = 7 # Gear Parallel Moving
		ctrl_cmd_linear = (msg.linear.x**2 + msg.linear.y**2)**0.5
		ctrl_cmd_angular = 0.0
		ctrl_cmd_slipangle = math.atan2(msg.linear.y,msg.linear.x) * 57.2958
		if abs(ctrl_cmd_slipangle) > 90:
			ctrl_cmd_linear = ctrl_cmd_linear * -1
			if ctrl_cmd_slipangle > 0:
				ctrl_cmd_slipangle = ctrl_cmd_slipangle - 180
			else:
				ctrl_cmd_slipangle = ctrl_cmd_slipangle + 180

def mode_callback(msg):
	global mode_flag
	mode_flag = msg.data

def ctrl_fb_callback(msg):
	ctrl_fb_gear = msg.ctrl_fb_gear
	ctrl_fb_linear = msg.ctrl_fb_linear
	ctrl_fb_angular = msg.ctrl_fb_angular
	ctrl_fb_slipangle = msg.ctrl_fb_slipangle

	global v_x
	global v_y
	global v_th

	if ctrl_fb_gear == 6 : # Gear 4T4D
		v_x = ctrl_fb_linear
		v_y = 0.0
		v_th = ctrl_fb_angular * 0.0174533

	elif ctrl_fb_gear == 7 : # Gear Parallel Moving
		v_x = ctrl_fb_linear * cos(ctrl_fb_slipangle * 0.0174533)
		v_y = ctrl_fb_linear * sin(ctrl_fb_slipangle * 0.0174533)
		v_th = 0.0


def main():
	rclpy.init()
	
	qos = QoSProfile(depth=10)
	node = rclpy.create_node('piot_converter_node')
	ctrl_cmd_pub = node.create_publisher(CtrlCmd, 'ctrl_cmd', qos)
	cmd_vel_sub = node.create_subscription(Twist, 'cmd_vel', cmd_vel_callback, qos)
	rate = node.create_rate(100)
	ctrl_cmd = CtrlCmd()
	ctrl_cmd_gear = 1 # Gear Parking
	ctrl_cmd_linear = 0.0
	ctrl_cmd_angular = 0.0
	ctrl_cmd_slipangle = 0.0
	
	mode_sub = node.create_subscription(Bool, 'mode', mode_callback, qos)
	mode = Bool()
	mode_flag = True
	
	odom_pub = node.create_publisher(Odometry, 'odom', qos)
	ctrl_fb_sub = node.create_subscription(CtrlFb, 'ctrl_fb', ctrl_fb_callback, qos)
	odom_broadcaster = TransformBroadcaster(node)
	odom_stamp = TransformStamped()
	x = 0.0
	y = 0.0
	th = 0.0
	v_x = 0.0
	v_y = 0.0
	v_th = 0.0

	current_time = node.get_clock().now().to_msg()
	last_time = node.get_clock().now().to_msg()
	
	# Spin in a seperate thread
	thread = threading.Thread(target=rclpy.spin, args=(node, ), daemon=True)
	thread.start()

	try:
		while (1):
			ctrl_cmd.ctrl_cmd_gear = ctrl_cmd_gear
			ctrl_cmd.ctrl_cmd_linear = ctrl_cmd_linear
			ctrl_cmd.ctrl_cmd_angular = ctrl_cmd_angular
			ctrl_cmd.ctrl_cmd_slipangle = ctrl_cmd_slipangle
			ctrl_cmd_pub.publish(ctrl_cmd)

			current_time = node.get_clock().now().to_msg()
			dt = (current_time.sec - last_time.sec) + (current_time.nanosec/1e+9 - last_time.nanosec/1e+9)
			delta_x = (v_x * cos(th) - v_y * sin(th)) * dt
			delta_y = (v_x * sin(th) + v_y * cos(th)) * dt
			delta_th = v_th * dt

			x += delta_x
			y += delta_y
			th += delta_th
			
			q = quaternion_from_euler(0,0,th)
			odom = Odometry()
			
			odom_stamp.header.stamp = current_time
			odom_stamp.header.frame_id = 'odom'
			odom_stamp.child_frame_id = 'base_link'
			
			odom_stamp.transform.translation.x = x
			odom_stamp.transform.translation.y = y
			odom_stamp.transform.translation.z = 0.0
			
			odom_stamp.transform.rotation.x = q[0]
			odom_stamp.transform.rotation.y = q[1]
			odom_stamp.transform.rotation.z = q[2]
			odom_stamp.transform.rotation.w = q[3]
			
			odom_broadcaster.sendTransform(odom_stamp)
			
			
			odom.header.stamp = current_time
			odom.header.frame_id = 'odom'
			odom.child_frame_id = 'base_link'
			
			odom.pose.pose.position.x = x
			odom.pose.pose.position.x = x
			odom.pose.pose.position.x = x
			
			odom.pose.pose.orientation.x = q[0]
			odom.pose.pose.orientation.y = q[1]
			odom.pose.pose.orientation.z = q[2]
			odom.pose.pose.orientation.w = q[3]
			
			odom.twist.twist.linear.x = v_x
			odom.twist.twist.linear.y = v_y
			odom.twist.twist.angular.z = v_th
			odom_pub.publish(odom)

			last_time = current_time
			
			rate.sleep()

	except KeyboardInterrupt:
		pass
#	rclpy.shutdown()
	thread.join()
			
if __name__ == '__main__':
	main()
