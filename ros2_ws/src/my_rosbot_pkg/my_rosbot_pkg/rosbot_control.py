#!/usr/bin/env python3
from multiprocessing.connection import wait # ???
from unittest import result #???
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Bool
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

# This node belongs to ROSbot

class RosbotControlNode(Node):
    def __init__(self):
        super().__init__("rosbot_control")

        self.nav = BasicNavigator()
        self.point_A_ = PoseStamped()
        self.point_B_ = PoseStamped()
        self.times_clicked_ = 0                 # counter for clicking points on Rviz
        self.timer_period_ = 0.1
        self.manip_finished_hz_counter_ = 0     # counter for /manip_finished messages publishing with 15 Hz - 
        self.state_ = -1                        # state machine state
        self.continue_ = False                  # flag for coming back to start after manipulator finishes

        self.arrived_pub_ = self.create_publisher(Bool, '/rosbot_arrived', 10)
        self.finished_sub_ = self.create_subscription(Bool, '/manip_finished', self.callback_continue, 10)
        self.point_sub_ = self.create_subscription(PointStamped, '/clicked_point', self.callback_point, 10)
        self.timer_ = self.create_timer(self.timer_period_, self.callback_input)
        self.get_logger().info("Rosbot control has been started!")


    def callback_point(self,msg):
        self.times_clicked_ += 1
        if self.times_clicked_ == 1:
            self.point_A_.pose.position.x = msg.point.x
            self.point_A_.pose.position.y = msg.point.y
            self.point_A_.pose.position.z = msg.point.z # unnses
            self.point_A_.header = msg.header
            self.nav.setInitialPose(self.point_A_)  # Sets the initial pose of the robot
            self.nav.waitUntilNav2Active()          # if autostarted, else use lifecycleStartup() - PROBABLY UNNESECERY
            self.get_logger().info("Point A:    x: " + str(self.point_A_.pose.position.x) + " y: " + str(self.point_A_.pose.position.y))
        elif self.times_clicked_ == 2:
            self.point_B_.pose.position.x = msg.point.x
            self.point_B_.pose.position.y = msg.point.y
            self.point_B_.pose.position.z = msg.point.z # unnses
            self.point_B_.header = msg.header
            self.get_logger().info("Point B:    x: " + str(self.point_B_.pose.position.x) + " y: " + str(self.point_B_.pose.position.y))
            self.state_ = 0
        else:
            self.get_logger().warn("You can only click 2 points (starting point, then manipulator's area point")
            self.state_ = -1

    
    def callback_input(self):
        msg = Bool()            
        msg.data = False        # flag that rosbot has not arrived
        if self.state_ == -1:
            self.get_logger().info("Waiting for 2 clicked points...")
        elif self.state_ == 0:  # just wait
            time.sleep(5.0) 
            self.state_ += 1
        elif self.state_ == 1:  # A -> B           
            self.nav.goToPose(self.point_B_)
            self.state_ += 1
        elif self.state_ == 2:  # waiting for a task to be completed...
            while not self.nav.isTaskComplete():
                None
            result = self.nav.getResult()
            if str(result) == "TaskResult.UNKNOWN":
                self.get_logger().info('UNKNOWN!!!')
            elif str(result) == "TaskResult.SUCCEEDED":
                self.get_logger().info('Goal succeeded!')
            elif str(result) == "TaskResult.CANCELED":
                self.get_logger().info('Goal was canceled!')
            elif str(result) == "TaskResult.FAILED":            # GOAL FILED actions can be added here
                self.get_logger().info('Goal failed!')
            self.continue_ = False
            msg.data = True     # flag that robot has arrived
            self.state_ += 1
        elif self.state_ == 3:  # waiting for manipulator to finish...
            self.get_logger().info("Waiting for manipulator to finish...")
            if self.continue_ == True and self.manip_finished_hz_counter_ > 75:    # counting untill 75 (5 seconds)
                msg.data = False     # flag that robot has arrived
                self.get_logger().info("Manipulator finished - message recrived")
                self.state_ += 1
                self.arrived_pub_.publish(msg)
                time.sleep(4.0) # just wait
            else:
                msg.data = True
        elif self.state_ == 4:  # B -> A
            self.nav.goToPose(self.point_A_)
            self.state_ += 1
        elif self.state_ == 5:  # waiting for a task to be completed...
            while not self.nav.isTaskComplete():
                None
            result = self.nav.getResult()
            result = self.nav.getResult()
            if str(result) == "TaskResult.UNKNOWN":
                self.get_logger().info('UNKNOWN!!!')
            elif str(result) == "TaskResult.SUCCEEDED":
                self.get_logger().info('Goal succeeded!')
            elif str(result) == "TaskResult.CANCELED":
                self.get_logger().info('Goal was canceled!')
            elif str(result) == "TaskResult.FAILED":
                self.get_logger().info('Goal failed!')
            self.state_ += 1
        elif self.state_ == 6:
            self.get_logger().info("ROSbot arrived at the start position")
            time.sleep(5.0) # just wait
            self.get_logger().info("Countdown for the next program loop...")
            wait_time_ = 10 # [s]
            for i in range(wait_time_,0,-1):
                self.get_logger().info(str(i))
                time.sleep(1)
            # Feature that you can choose 2 diffrent points, but may be useless (A and B are const!)
            # self.times_clicked_ = 0     # uncomment if A and B have to change.
            # self.state_ = -1            # uncomment if A and B have to change.
            self.state_ = 0            # uncomment if A and B are const.
        else:
            self.get_logger().warn("Unscpected behavior with state = " + str(self.state_))
        self.arrived_pub_.publish(msg)


    def callback_continue(self, msg):
        self.continue_ = msg.data
        if self.continue_ == True:
            self.manip_finished_hz_counter_ += 1
        if self.continue_ == False:
            self.manip_finished_hz_counter_ = 0


def main(args=None):
    rclpy.init(args=args)
    node = RosbotControlNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
