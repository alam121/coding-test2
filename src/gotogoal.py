#!/usr/bin/env python

import rospy
from std_msgs.msg import Int8

if __name__ == '__main__':
    rospy.init_node('goal_location_publisher', anonymous=True)
    goal_location_publisher = rospy.Publisher('/goal_location', Int8, queue_size=10)
    rate = rospy.Rate(1)  # Publish rate (1 Hz)

    while not rospy.is_shutdown():
        # Get the goal location from the user
        goal_location = int(input("Enter goal location (1, 2, or 3): "))

        # Check if the entered goal location is valid
        if goal_location in [1, 2, 3]:
            goal_location_publisher.publish(goal_location)
            rospy.loginfo("Published goal location: {}".format(goal_location))
        else:
            rospy.loginfo("Invalid goal location. Please enter 1, 2, or 3.")

        rate.sleep()

