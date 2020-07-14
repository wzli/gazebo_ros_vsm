#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <std_srvs/Empty.h>

#include <random>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
    ros::init(argc, argv, "random_walk");
    ros::NodeHandle nh;
    MoveBaseClient ac("move_base", true);
    move_base_msgs::MoveBaseGoal goal;

    ros::ServiceClient clear_costmap = nh.serviceClient<std_srvs::Empty>("move_base/clear_costmaps");
    ros::ServiceClient clear_unknown_space = nh.serviceClient<std_srvs::Empty>("move_base/clear_unknown_space");

    std::random_device random_device;
    std::mt19937 random_generator(random_device());

    ros::NodeHandle pnh("~");
    std::string frame;
    pnh.param<std::string>("frame", goal.target_pose.header.frame_id, "odom");

    double max_x, max_y, min_x, min_y;
    pnh.param<double>("min_x", min_x, -10.0f);
    pnh.param<double>("min_y", min_y, -10.0f);
    pnh.param<double>("max_x", max_x, 10.0f);
    pnh.param<double>("max_y", max_y, 10.0f);
    std::uniform_real_distribution<double> x_dist(min_x, max_x);
    std::uniform_real_distribution<double> y_dist(min_y, max_y);
    std::uniform_real_distribution<double> w_dist(-1, 1);

    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }

    while(ros::ok()) {
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose.position.x = x_dist(random_generator);
        goal.target_pose.pose.position.y = y_dist(random_generator);
        double w = w_dist(random_generator);
        goal.target_pose.pose.orientation.z = std::sqrt(1 - (w * w));
        goal.target_pose.pose.orientation.w = w;
        ROS_WARN("Sending goal to %f %f", goal.target_pose.pose.position.x,
            goal.target_pose.pose.position.y);
        ac.sendGoal(goal);
        ac.waitForResult();
        ROS_WARN("Goal result %s", ac.getState().getText().c_str());
        if(ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_WARN("Clear Costmaps");
            std_srvs::Empty empty;
            clear_costmap.call(empty);
            clear_unknown_space.call(empty);
        }
    }
    return 0;
}
