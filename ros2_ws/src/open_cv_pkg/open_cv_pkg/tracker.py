#!/usr/bin/env python3

# import encodings	# delete this line and test
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2 as cv
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class TrackerNode(Node):
	def __init__(self):
		super().__init__("tracker")

		self.depth_sub_ = self.create_subscription(Image, '/depth/image_rect_raw', self.callback_depth, 10)
		self.tracker_sub_ = self.create_subscription(Image, '/color/image_raw', self.callback_tracker, 10)
		self.target_pub_ = self.create_publisher(Point, '/target_on_image', 10)
		self.output_color_pub_ = self.create_publisher(Image, '/output/color', 10)
		self.output_black_pub_ = self.create_publisher(Image, '/output/black', 10)
		self.output_depth_pub_ = self.create_publisher(Image, '/output/depth', 10)
		self.finished_pub_ = self.create_publisher(Bool, '/manip_finished', 10)

		self.debug_flag_ = True 		# If true, node prints important debug data
		self.debug_flag_loop_ = False	# If true, node prints objects detecion adata in loop
		self.H_ = 700.0 				# [mm] Hardcoded height of Realsense above the ground
		self.object_height_ = 0.0 		# Global variable for saving previous value
		self.br = CvBridge()
		self.depth_frame_ = None
		self.depth_aquired_ = False
		self.color_to_depth_const_x_ = 480/640 	# manual calibration with 2 Images
		self.color_to_depth_const_y_ = 330/480
		self.color_to_depth_const_dx_ = 162 	# origin of Color's coords system in Depth coords
		self.color_to_depth_const_dy_ = 75

		self.get_logger().info("Tracker has been started!")

	def callback_depth(self, msg):
		if (self.debug_flag_loop_ == True):
			self.get_logger().info("Depth image aquired...")
		current_frame_depth = self.br.imgmsg_to_cv2(msg, desired_encoding='16UC1')
		self.depth_frame_ = current_frame_depth
		self.depth_aquired_ = True

	def callback_tracker(self, msg):
		if (self.debug_flag_loop_ == True):
			self.get_logger().info("Tracker callback begins...")
		try:
			cv_img = self.br.imgmsg_to_cv2(msg, "bgr8")
		except CvBridgeError as e:
			print(e)
			return
		
		hsv = cv.cvtColor(cv_img, cv.COLOR_BGR2HSV)
		yellowLower1 = (19, 100, 110)		# HSV ranges for yellow
		yellowUpper1 = (40, 255, 255)
		yellowLower2 = (24, 60, 70)
		yellowUpper2 = (50, 255, 255)
		mask_img1 = cv.inRange(hsv, yellowLower1, yellowUpper1)
		mask_img2 = cv.inRange(hsv, yellowLower2, yellowUpper2)
		mask_img = mask_img1 + mask_img2
		contours = cv.findContours(mask_img, cv.RETR_TREE, cv.CHAIN_APPROX_SIMPLE) # contours, hierarchy = cv.fi
		cv.drawContours(cv_img, contours, -1, (0,255,0), 2)

		black_img = np.zeros(cv_img.shape, 'uint8')
		cx = None
		cy = None
		for c in contours:
			area = cv.contourArea(c)
			((x,y), radius) = cv.minEnclosingCircle(c)
			if (area > 800) and (radius < 70):
				cv.drawContours(cv_img, [c], -1, (255,0,255), 2)
				cx, cy = self.get_contour_center(c)
				cv.circle(cv_img, (cx,cy), (int)(radius), (0,255,255), 3)
				cv.circle(black_img, (cx,cy), (int)(radius), (0,255,255), 3)
				cv.circle(black_img, (cx,cy), 5, (150,0,255), -1)
				cv.putText(black_img, "Yellow", (cx - 5*int(radius), cy - int(1.5*radius)),cv.FONT_HERSHEY_SIMPLEX, 2, (255, 255, 255), 4)
				if (self.debug_flag_loop_ == True):
					self.get_logger().info("x: " + str(cx) + ", y: " + str(cy))
					self.get_logger().info("Object radius: " + str(radius))
					self.get_logger().info("Object area: " + str(area))
		target_ = Point()
		if (cx != None) and (cy != None):
			target_.x = float(cx)
			target_.y = float(cy)
			target_.z = 0.0
			colorArray_ = np.array(cv_img)
			if (self.debug_flag_loop_ ==True):
				self.get_logger().info("single element [BGR]: " + str((colorArray_[cy][cx][None])/255))
			if (self.depth_aquired_ == True):
				dx = int(self.color_to_depth_const_x_*cx + self.color_to_depth_const_dx_)
				dy = int(self.color_to_depth_const_y_*cy + self.color_to_depth_const_dy_)
				cv.circle(self.depth_frame_, (dx,dy), 20, (0,255,255), 3)
				depthArray_ = np.array(self.depth_frame_)
				object_depth_ = depthArray_[dy][dx]
				if (object_depth_ > 1.0): 	# ensure that it not detects something very close
					self.object_height_ = self.H_ - object_depth_ # [mm]
				if (self.debug_flag_loop_ == True):
					self.get_logger().info("Object height: " + str(self.object_height_))
				target_.z = self.object_height_
			self.target_pub_.publish(target_)
			finished_ = Bool()
			finished_.data = False
			self.finished_pub_.publish(finished_)  		# flag to ROSbot that manipulator DID NOT finish
		else:	# no object detected
			target_.z = 0.0
			self.target_pub_.publish(target_)			# message to manipulator that there is no objets left
			if self.debug_flag_loop_ == True:
				self.get_logger().info("Robot empty! Sending message to ROSbot")
			finished_ = Bool()   
			finished_.data = True
			self.finished_pub_.publish(finished_)  		# flag to ROSbot that manipulator DID finish
		# publishing to rviz2
		cv_img_msg = self.br.cv2_to_imgmsg(cv_img, encoding="passthrough")
		cv_img_msg.header.frame_id = "map"
		self.output_color_pub_.publish(cv_img_msg)
		black_img_msg = self.br.cv2_to_imgmsg(black_img, encoding="passthrough")
		black_img_msg.header.frame_id = "map"
		self.output_black_pub_.publish(black_img_msg)
		if (self.depth_aquired_ == True):
			depth_frame_msg = self.br.cv2_to_imgmsg(self.depth_frame_, '16UC1')
			depth_frame_msg.header.frame_id = "map"
			self.output_depth_pub_.publish(depth_frame_msg)
		cv.waitKey(50)
		if self.debug_flag_loop_ == True:
			self.get_logger().info("----------------------")

	def get_contour_center(self, contour):
		M = cv.moments(contour)
		cx = -1
		cy = -1
		if(M['m00'] != 0):
			cx = int(M['m10']/M['m00'])
			cy = int(M['m01']/M['m00'])
		return cx, cy

def main(args=None):
	rclpy.init(args=args)
	node = TrackerNode()
	rclpy.spin(node)
	rclpy.shutdown()

if __name__ == "__main__":
	main()
