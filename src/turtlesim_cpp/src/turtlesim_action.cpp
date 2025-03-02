#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <turtlesim/msg/pose.hpp>
#include <cpp_node/action/go_to_pose.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <rclcpp/callback_group.hpp>
#include <cmath>

class ControllerNode : public rclcpp::Node {
public:
    using GoToPose = cpp_node::action::GoToPose;
    using GoalHandleGoToPose = rclcpp_action::ServerGoalHandle<cpp_node::action::GoToPose>;
    std::shared_ptr<GoalHandleGoToPose> active_goal_;


    ControllerNode() : Node("turt_controller") {
        RCLCPP_INFO(this->get_logger(), "Node Started");

        // Default goal position
        desired_x_ = 5.544;
        desired_y_ = 5.544;

        // Initialize error variables
        err_dist_ = 0;
        err_theta_ = 0;
        goal_completed_ = false;

        // Create publisher for velocity commands
        vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);

        // Create action server
        action_server_ = rclcpp_action::create_server<GoToPose>(
            this, "GoToPose",
            std::bind(&ControllerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&ControllerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&ControllerNode::handle_accepted, this, std::placeholders::_1));
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
    rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_sub_;

    double desired_x_, desired_y_;
    double err_dist_, err_theta_;
    bool goal_completed_;
   // std::shared_ptr<GoalHandleGoToPose> active_goal_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const GoToPose::Goal> goal) {
        RCLCPP_INFO(this->get_logger(), "Received a goal...");
        if (std::abs(goal->desired_x_pos) <= 11 && std::abs(goal->desired_y_pos) <= 11) {
            RCLCPP_INFO(this->get_logger(), "Goal accepted: x=%.2f, y=%.2f", goal->desired_x_pos, goal->desired_y_pos);
            return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        }
        RCLCPP_WARN(this->get_logger(), "Goal rejected: x=%.2f, y=%.2f", goal->desired_x_pos, goal->desired_y_pos);
        return rclcpp_action::GoalResponse::REJECT;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        std::thread([this, goal_handle]() {
            execute(goal_handle);
        }).detach();
    }

    void execute(const std::shared_ptr<GoalHandleGoToPose> goal_handle) {
        RCLCPP_INFO(this->get_logger(), "Executing goal...");
        active_goal_ = goal_handle;
        desired_x_ = goal_handle->get_goal()->desired_x_pos;
        desired_y_ = goal_handle->get_goal()->desired_y_pos;

        pose_sub_ = this->create_subscription<turtlesim::msg::Pose>(
            "/turtle1/pose", 10, 
            std::bind(&ControllerNode::pose_callback, this, std::placeholders::_1));
    }

    void pose_callback(const turtlesim::msg::Pose::SharedPtr msg) {
        if (!active_goal_) return;

        auto feedback = std::make_shared<GoToPose::Feedback>();
        feedback->current_x_pos = msg->x;
        feedback->current_y_pos = msg->y;
        active_goal_->publish_feedback(feedback);

        double err_x = desired_x_ - msg->x;
        double err_y = desired_y_ - msg->y;
        err_dist_ = std::sqrt(err_x * err_x + err_y * err_y);
        double desired_theta = std::atan2(err_y, err_x);
        err_theta_ = desired_theta - msg->theta;

        while (err_theta_ > M_PI) err_theta_ -= 2.0 * M_PI;
        while (err_theta_ < -M_PI) err_theta_ += 2.0 * M_PI;

        double Kp_dist = 0.2, Ki_dist = 0.05, Kd_dist = 0.02;
        double Kp_theta = 1.5, Ki_theta = 0.18, Kd_theta = 0.1;

        static double integral_dist = 0.0, previous_err_dist = 0.0;
        static double integral_theta = 0.0, previous_err_theta = 0.0;

        double l_v = (err_dist_ >= 0.1) ? (Kp_dist * err_dist_ + Ki_dist * integral_dist + Kd_dist * (err_dist_ - previous_err_dist)) : 0.0;
        previous_err_dist = err_dist_;

        double a_v = (std::abs(err_theta_) >= 0.08) ? (Kp_theta * err_theta_ + Ki_theta * integral_theta + Kd_theta * (err_theta_ - previous_err_theta)) : 0.0;
        previous_err_theta = err_theta_;

        if (err_dist_ < 0.1 && std::abs(err_theta_) < 0.08) {
            RCLCPP_INFO(this->get_logger(), "Goal reached successfully!");
            active_goal_->succeed(std::make_shared<GoToPose::Result>());
            active_goal_.reset();
        } else {
            publish_velocity(l_v, a_v);
        }
    }

    void publish_velocity(double l_v, double a_v) {
        auto msg = geometry_msgs::msg::Twist();
        msg.linear.x = l_v;
        msg.angular.z = a_v;
        vel_pub_->publish(msg);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControllerNode>());
    rclcpp::shutdown();
    return 0;
}
