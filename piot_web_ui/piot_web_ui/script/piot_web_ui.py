#! /usr/bin/env python3

import rclpy
import numpy as np
import cv2
import argparse
from rclpy.node import Node
from rclpy.qos import QoSProfile
from flask import Flask, Response, render_template, request
from geometry_msgs.msg import Pose, Quaternion
from nav_msgs.srv import GetMap
from nav_msgs.msg import OccupancyGrid

Image = np.zeros((320,240,3),dtype='uint8')

app = Flask(__name__)

@app.route("/", methods=['GET','POST'])
def index():
	return render_template('index.html')
 
@app.route("/video_feed")
def video_feed():
	return Response(send_video(),mimetype = "multipart/x-mixed-replace; boundary=frame")

def send_video():
	global Image
	flag, encodedImage = cv2.imencode(".jpg", Image)
	encodedImage = encodedImage.tobytes()
#	if not flag:
#		continue
 
	yield(b'--frame\r\n' b'Content-Type: image/jpeg\r\n\r\n' + encodedImage+ b'\r\n')

class PiotWebUI(Node):

	def __init__(self):
		super().__init__('piot_web_ui_node')
		qos = QoSProfile(depth=10)
		self.mab_sub = self.create_subscription(OccupancyGrid, 'map', self.map_callback, qos)
		self.pose_sub = self.create_subscription(Pose, 'pose', self.pose_callback, qos)   
		
	def	map_callback(self,msg):
		size_x = msg.info.width
		size_y = msg.info.height
		
		if size_x < 3 or size_y < 3:
			self.get_logger().info('Map size is only x: {0},  y: {1} . Not running map to image conversion'.format(size_x,size_y))
			return
   
	def pose_callback(self,msg):
		pass   
			
			
def main(args=None):  
	app.run(host="172.16.128.143", port=5000, debug=True, threaded=True, use_reloader=False)
  
	rclpy.init(args=args)
	piot_web_ui = PiotWebUI()
	rclpy.spin(piot_web_ui)
	piot_web_ui.destroy_node()
	rclpy.shutdown()


if __name__ == '__main__':
	main()

