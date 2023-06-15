#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class GoalSubscriber {
private:
    ros::NodeHandle nh_;
    ros::Subscriber goal_location_sub_;
    MoveBaseClient ac_;

public:
    GoalSubscriber() : ac_("move_base", true) {
        goal_location_sub_ = nh_.subscribe("/goal_location", 1, &GoalSubscriber::goalLocationCallback, this);
        ROS_INFO("Waiting for the move_base action server to come up");
        ac_.waitForServer();
    }

    void goalLocationCallback(const std_msgs::Int8::ConstPtr& msg) {
        int goal_number = msg->data;

        move_base_msgs::MoveBaseGoal goal;

        // Set the goal target pose based on the goal number
        if (goal_number == 1) {
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = -1.5;
            goal.target_pose.pose.position.y = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;
        } else if (goal_number == 2) {
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = 0.0;
            goal.target_pose.pose.position.y = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;
        } else if (goal_number == 3) {
            goal.target_pose.header.frame_id = "map";
            goal.target_pose.header.stamp = ros::Time::now();
            goal.target_pose.pose.position.x = 2.0;
            goal.target_pose.pose.position.y = 0.0;
            goal.target_pose.pose.orientation.w = 1.0;
        } else {
            ROS_INFO("Invalid goal number. Robot will not navigate.");
            return;
        }

        ROS_INFO("Sending goal");
        ac_.sendGoal(goal);

        // Wait for the result
        ac_.waitForResult();

        if (ac_.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
            ROS_INFO("Goal reached successfully!");
        else
            ROS_INFO("Failed to reach the goal.");
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "goal_subscriber");

    GoalSubscriber goal_subscriber;

    ros::spin();

    return 0;
}

