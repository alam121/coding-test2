#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int8.h>
#include <cmath>

class TurtleBot {
public:
    TurtleBot() {
        ros::NodeHandle nh;
        velocity_publisher = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        pose_subscriber = nh.subscribe("/odom", 10, &TurtleBot::update_pose, this);
        goal_location_subscriber = nh.subscribe("/goal_location", 10, &TurtleBot::goal_location_callback, this);
    }

    void update_pose(const nav_msgs::Odometry::ConstPtr& data) {
        pose = *data;
    }

    void goal_location_callback(const std_msgs::Int8::ConstPtr& data) {
        int goal_number = data->data;
        nav_msgs::Odometry goal_pose;
        if (goal_number == 1) {
            goal_pose.pose.pose.position.x = -1;
            goal_pose.pose.pose.position.y = 0;
        }
        else if (goal_number == 2) {
            goal_pose.pose.pose.position.x = -3;
            goal_pose.pose.pose.position.y = 0;
        }
        else if (goal_number == 3) {
            goal_pose.pose.pose.position.x = 2;
            goal_pose.pose.pose.position.y = 2;
        }
        else {
            ROS_INFO("Invalid goal number. Robot will not navigate.");
            return;
        }
        move2goal(goal_pose);
    }

    double euclidean_distance(const nav_msgs::Odometry& goal_pose) {
        double dx = goal_pose.pose.pose.position.x - pose.pose.pose.position.x;
        double dy = goal_pose.pose.pose.position.y - pose.pose.pose.position.y;
        return std::sqrt(dx*dx + dy*dy);
    }

    double linear_vel(const nav_msgs::Odometry& goal_pose, double constant = 1.5) {
        return euclidean_distance(goal_pose);
    }

    double steering_angle(const nav_msgs::Odometry& goal_pose) {
        return std::atan2(goal_pose.pose.pose.position.y - pose.pose.pose.position.y,
                          goal_pose.pose.pose.position.x - pose.pose.pose.position.x);
    }

    double angular_vel(const nav_msgs::Odometry& goal_pose, double constant = 6) {
        double yaw = tf::getYaw(pose.pose.pose.orientation);
        return (steering_angle(goal_pose) - yaw);
    }

    void move2goal(const nav_msgs::Odometry& goal_pose) {
        double distance_tolerance = 0.1;
        geometry_msgs::Twist vel_msg;

        while (euclidean_distance(goal_pose) >= distance_tolerance) {
            double vel_pid = 0.30 * linear_vel(goal_pose);
            double ang_pid = 0.3 * angular_vel(goal_pose);

            vel_msg.linear.x = vel_pid;
            vel_msg.angular.z = ang_pid;

            velocity_publisher.publish(vel_msg);

            ros::spinOnce();
            ROS_INFO("Euclidean distance to goal: %f", euclidean_distance(goal_pose));

        }

        ROS_INFO("Reached the goal");
        vel_msg.linear.x = 0;
        vel_msg.angular.z = 0;
        velocity_publisher.publish(vel_msg);
    }

private:
    ros::Publisher velocity_publisher;
    ros::Subscriber pose_subscriber;
    ros::Subscriber goal_location_subscriber;
    nav_msgs::Odometry pose;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "turtlebot_controller");
    TurtleBot turtlebot;
    ros::spin();
    return 0;
}

