#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <iostream>
#include <sstream>

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_location_publisher");
    ros::NodeHandle nh;
    ros::Publisher goal_location_publisher = nh.advertise<std_msgs::Int8>("/goal_location", 10);

    while (ros::ok()) {
        std_msgs::Int8 goal_location;
        std::string input;
        std::cout << "Enter goal location (1, 2, or 3): ";
        std::cin >> input;

        std::istringstream iss(input);
        int value;
        if (!(iss >> value)) {
            ROS_INFO("Invalid goal location. Please enter a valid integer.");
            continue;
        }

        goal_location.data = value;

        if (goal_location.data >= 1 && goal_location.data <= 3) {
            goal_location_publisher.publish(goal_location);
            ROS_INFO("Published goal location: %d", goal_location.data);
        } else {
            ROS_INFO("Invalid goal location. Please enter 1, 2, or 3.");
        }

        ros::spinOnce();  // Process any pending callbacks

        // Wait for a short duration to allow other callbacks to be processed
        ros::Duration(0.1).sleep();
    }

    return 0;
}

