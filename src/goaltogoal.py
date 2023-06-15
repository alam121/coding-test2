#!/usr/bin/env python

import rospy
import math
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
from math import pow, atan2, sqrt
from std_msgs.msg import Int8
import time

class TurtleBot:
    def __init__(self):
        rospy.init_node('turtlebot_controller', anonymous=True)
        self.velocity_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.pose_subscriber = rospy.Subscriber("/odom", Odometry, self.update_pose)
        self.goal_location_subscriber = rospy.Subscriber("/goal_location", Int8, self.goal_location_callback)
        self.pose = Odometry()
        self.rate = rospy.Rate(10)

    def update_pose(self, data):
        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)
        self.pose.pose.pose.orientation.x = round(self.pose.pose.pose.orientation.x, 4)
        self.pose.pose.pose.orientation.y = round(self.pose.pose.pose.orientation.y, 4)
        self.pose.pose.pose.orientation.z = round(self.pose.pose.pose.orientation.z, 4)
        self.pose.pose.pose.orientation.w = round(self.pose.pose.pose.orientation.w, 4)

    def goal_location_callback(self, data):
        goal_number = data.data
        goal_pose = Odometry()
        if goal_number == 1:
            goal_pose.pose.pose.position.x = 1
            goal_pose.pose.pose.position.y = 1
        elif goal_number == 2:
            goal_pose.pose.pose.position.x = 2
            goal_pose.pose.pose.position.y = 0
        elif goal_number == 3:
            goal_pose.pose.pose.position.x = 3
            goal_pose.pose.pose.position.y = 1
        else:
            rospy.loginfo("Invalid goal number. Robot will not navigate.")
            return
        self.move2goal(goal_pose)

    def euclidean_distance(self, goal_pose):
        return sqrt(pow((goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y), 2))

    def linear_vel(self, goal_pose, constant=1.5):
        return self.euclidean_distance(goal_pose)

    def steering_angle(self, goal_pose):
        return atan2(goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y,
                     goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x)

    def angular_vel(self, goal_pose, constant=6):
        orientation_list = [self.pose.pose.pose.orientation.x, self.pose.pose.pose.orientation.y,
                            self.pose.pose.pose.orientation.z, self.pose.pose.pose.orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        rospy.loginfo('roll %d pitch %d yaw %d', roll, pitch, yaw)
        return (self.steering_angle(goal_pose) - yaw)

    def move2goal(self, goal_pose):
        error_angle = 0
        error_linear = 0
        sumError_angle = 0
        sumError_linear = 0
        rateError_linear = 0
        rateError_angle = 0
        currentTime = time.time()
        previousTime = time.time()
        lastError_linear = 0
        lastError_angle = 0
        distance_tolerance = 0.1

        vel_msg = Twist()
        while self.euclidean_distance(goal_pose) >= distance_tolerance:
            currentTime = time.time()
            elapsedTime = (currentTime - previousTime)
            error_linear = self.linear_vel(goal_pose)
            error_angle = self.angular_vel(goal_pose)
            sumError_linear = sumError_linear + (error_linear * elapsedTime)
            rateError_linear = (error_linear - lastError_linear) / elapsedTime
            sumError_angle = sumError_angle + (error_angle * elapsedTime)
            rateError_angle = (error_angle - lastError_angle) / elapsedTime
            vel_pid = (0.30 * error_linear) + (0 * sumError_linear) + (0 * rateError_linear)
            ang_pid = (0.3 * error_angle) + (0 * sumError_angle) + (0 * rateError_angle)

            vel_msg.linear.x = vel_pid
            vel_msg.linear.y = 0
            vel_msg.linear.z = 0
            vel_msg.angular.x = 0
            vel_msg.angular.y = 0
            vel_msg.angular.z = ang_pid

            self.velocity_publisher.publish(vel_msg)

            previousTime = currentTime
            lastError_angle = error_angle
            lastError_linear = error_linear

            self.rate.sleep()
            print(self.euclidean_distance(goal_pose))

        rospy.loginfo("Reached the goal")
        vel_msg.linear.x = 0
        vel_msg.angular.z = 0
        self.velocity_publisher.publish(vel_msg)

if __name__ == '__main__':
    try:
        x = TurtleBot()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass

