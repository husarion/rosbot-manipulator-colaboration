#!/usr/bin/env python3
import time
# import math # unused?
import rclpy
from rclpy.node import Node
from functools import partial

from open_manipulator_msgs.srv import SetJointPosition
from open_manipulator_msgs.srv import SetKinematicsPose
# from sensor_msgs.msg import JointState  # try without this
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

class GrabberFromImageCordsNode(Node):
    def __init__(self):
        super().__init__("grabber_from_image_cords")

        self.debug_flag_ = True 	# If true, node prints all debug data
        self.storage_flag_ = True   # True - making tower with objects ; False - throwing objects away

        if self.storage_flag_ == True:
            self.storage_Z_ = 0.000
            self.storage_coords_ = [0.245, -0.040, self.storage_Z_]     # Z will change
            self.rosbot_with_plate_height_ = 0.168                      # hardcoded height of ROSbot with printed plate [m]
       
        self.home_joint_objective_ = [0.000, -1.049, 0.357, 0.703, 0.010]   # initial home pose
        self.object_position_ = None
        self.H_ = 0.70                      # [m] hardcoded height of Realsense above the ground
        self.manip_offset_ = 0.04           # [m] hardcoded height of manipulator's stand
        self.camera_width_in_mm_ = 710.0    # [mm] hardcoded width of camera view measured on the ground 
        self.camera_width_in_pixels_ = 640  # [px] hardcoded width resolution of Intel Realsense D435
        self.camera_length_in_pixels_ = 480 # [px] hardcoded length resolution of Intel Realsense D435  
        self.one_pixel_in_mm_ = self.camera_width_in_mm_/self.camera_width_in_pixels_
        self.path_time_ = 3.0               # [s] time of single manipulator move
        self.rosbot_present_ = False
        self.object_count_ = 0              # counter how many objects manipulator grabbed

        self.target_sub_ = self.create_subscription(Point, '/target_on_image', self.callback_grab, 0)
        self.arrived_sub_ = self.create_subscription(Bool, '/rosbot_arrived', self.callback_arrived, 10)

        self.get_logger().info("Grabber from image cords - has been started!")
        self.call_joint_move_service(self.home_joint_objective_, 2*self.path_time_)     # move home
        if self.debug_flag_ == True:
            self.gripper("open")

    def callback_arrived(self, msg):
        self.rosbot_present_ = msg.data

    def callback_grab(self, msg):
        self.object_position_ = msg
        if (self.object_position_.x != None) and (self.rosbot_present_ == True):
            xMID = 0.0          # middle of camera in robot's coords (here top mid)
            yMID = (self.camera_length_in_pixels_/2)/1000
            cx = self.object_position_.x
            cy = self.object_position_.y
            cz = self.object_position_.z
            if (cz != 0.0):
                x = (self.one_pixel_in_mm_*(-cx + self.camera_width_in_pixels_/2))/1000     # [m]
                y = (self.one_pixel_in_mm_*cy)/1000                                         # [m]
                z = cz/1000                                                                 # [m]
                z = z - self.manip_offset_ + 0.006          # + experimental Z increment (tuning surface of the grip)
                if self.storage_flag_ == True:
                    self.storage_Z_ += z - self.rosbot_with_plate_height_
                    if self.object_count_ > 0:
                        self.storage_Z_ += self.manip_offset_
                    self.storage_coords_[2] = self.storage_Z_
                    self.storage_coords_above_ = [self.storage_coords_[0], self.storage_coords_[1], self.storage_coords_[2] + 0.04]
                Rx = abs(x-xMID)        # [m] x radius from camera'a mid (dealing with parallax error)
                Ry = abs(y-yMID)        # [m] y radius from camera'a mid (dealing with parallax error)
                delta_x = Rx*z/self.H_
                delta_y = Ry*z/self.H_
                if (x < xMID):
                    x_real = x + delta_x
                else:
                    x_real = x - delta_x
                if (y < yMID):
                    y_real = y + delta_y
                else:
                    y_real = y - delta_y
                if self.debug_flag_ == True:
                    self.get_logger().info("X Radius: " + str(Rx))
                    self.get_logger().info("Y Radius: " + str(Ry))
                    self.get_logger().info("delta_x: " + str(delta_x))
                    self.get_logger().info("delta_y: " + str(delta_y))
                    self.get_logger().info("X: " + str(x) + " -> " + str(x_real))
                    self.get_logger().info("Y: " + str(y) + " -> " + str(y_real))
                coords_real = [x_real, y_real, z]               # Real Coords without paralax error
                coords_real_above = [x_real, y_real, z + 0.04]
                self.object_count_ += 1
                if self.debug_flag_ == True:
                    self.get_logger().info("Grab start!" + str(coords_real))
                    self.get_logger().info("Gripper opening")
                self.gripper("open")
                if self.debug_flag_ == True:
                    self.get_logger().info("Moving above object " + str(self.object_count_) + " " + str(coords_real_above))
                self.call_task_move_service(coords_real_above, self.path_time_)
                if self.debug_flag_ == True:
                    self.get_logger().info("Moving to object " + str(self.object_count_) + " " + str(coords_real))
                self.call_task_move_service(coords_real, self.path_time_)
                self.gripper("close")
                if self.debug_flag_ == True:
                    self.get_logger().info("Moving above obejct " + str(self.object_count_) + " " + str(coords_real_above))
                self.call_task_move_service(coords_real_above, self.path_time_)
                if self.debug_flag_ == True:
                    self.get_logger().info("Joint move home")
                self.call_joint_move_service(self.home_joint_objective_, self.path_time_)   # move home
                if self.storage_flag_ == True:
                    if self.debug_flag_ == True:
                        self.get_logger().info("Moving above the storage with Z: " + str(self.storage_coords_above_[2]))
                    self.call_task_move_service(self.storage_coords_above_, self.path_time_)
                    if self.debug_flag_ == True:
                        self.get_logger().info("Moving to storage with Z: " + str(self.storage_coords_[2]))
                    self.call_task_move_service(self.storage_coords_, self.path_time_)
                self.gripper("open")
                if self.storage_flag_ == True:
                    if self.debug_flag_ == True:
                        self.get_logger().info("Joint move home")
                    self.call_joint_move_service(self.home_joint_objective_, self.path_time_)   # move home
                self.get_logger().info("Object number " + str(self.object_count_) + " finished!")
            else:
                self.get_logger().warn("Object not detected! (z = 0.0)")

    # TASK MOVE
    def call_task_move_service(self, objective, path_time):
        client = self.create_client(SetKinematicsPose, "/goal_task_space_path")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Controller Server...")
        request = SetKinematicsPose.Request()
        # request.planning_group = "abc" # Planning Group which set MoveIt! configuration
        request.end_effector_name = "gripper"
        request.kinematics_pose.pose.position.x = objective[0]
        request.kinematics_pose.pose.position.y = objective[1]
        request.kinematics_pose.pose.position.z = objective[2]
        request.path_time = path_time
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_task_move, objective = objective, path_time = path_time))
        time.sleep(path_time)
    def callback_task_move(self, future): #def callback_task_move(self, future, objective, path_time):
        try:
            response = future.result()
            if not response.is_planned:
                self.get_logger().error("Grab could not be executed!")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

    # GRIPPER
    def gripper(self, action):
        if action == "close":
            if self.debug_flag_ == True:
                self.get_logger().info("Gripper " + action)
            gripper_pos = -0.010
        elif action == "open":
            if self.debug_flag_ == True:
                self.get_logger().info("Gripper " + action)
            gripper_pos = 0.010
        else:
            self.get_logger().error("gripper function error")
        self.call_tool_control_service([gripper_pos], 1.0)
        time.sleep(1.0)
    def call_tool_control_service(self, objective, path_time):
        client = self.create_client(SetJointPosition, "/goal_tool_control")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Controller Server...")
        request = SetJointPosition.Request()
        # request.planning_group = "abc" # Planning Group which set MoveIt! configuration
        request.joint_position.joint_name = ["gripper"]
        request.joint_position.position = objective
        request.path_time = path_time
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_tool_control, objective = objective, path_time = path_time))
    def callback_tool_control(self, future): #def callback_tool_control(self, future, objective, path_time):
        try:
            response = future.result()
            if not response.is_planned:
                self.get_logger().error("Move could not be executed!")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))
    
    #JOIN MOVE (used for homing)
    def call_joint_move_service(self, objective, path_time):
        client = self.create_client(SetJointPosition, "/goal_joint_space_path")
        while not client.wait_for_service(1.0):
            self.get_logger().warn("Waiting for Controller Server...")
        request = SetJointPosition.Request()
        # request.planning_group = "abc" # Planning Group which set MoveIt! configuration
        request.joint_position.joint_name = ["joint1","joint2","joint3","joint4","gripper"]
        request.joint_position.position = objective
        request.path_time = path_time
        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_joint_move, objective = objective, path_time = path_time))
        time.sleep(path_time)
    def callback_joint_move(self, future): #def callback_joint_move(self, future, objective, path_time):
        try:
            response = future.result()
            if not response.is_planned:
                self.get_logger().error("Move could not be executed!")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    rclpy.init(args=args)
    node = GrabberFromImageCordsNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
