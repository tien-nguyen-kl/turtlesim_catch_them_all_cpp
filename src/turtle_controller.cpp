#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "turtlesim/msg/pose.hpp"
#include <cmath>
#include "geometry_msgs/msg/twist.hpp"

using namespace std::placeholders;

class TurtleController : public rclcpp::Node{
public:
    TurtleController() : Node("turtle_controller") {
        alive_turtle_sub_ = this->create_subscription<my_robot_interfaces::msg::TurtleArray>("alive_turtles",10,std::bind(&TurtleController::callbackTurtleSubscription, this, _1));
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose",10,std::bind(&TurtleController::callbackPoseSubscription, this, _1));
        catch_turtle_client_ = this->create_client<my_robot_interfaces::srv::CatchTurtle>("catch_turtle");
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10),std::bind(&TurtleController::controlLoop, this));
        cmd_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("turtle1/cmd_vel",10);
    }

private:
    void callbackTurtleSubscription(const std::shared_ptr<my_robot_interfaces::msg::TurtleArray> msg){
        alive_turtles_ = msg->living_turtles;
        if(!alive_turtles_.empty()){
            find_closest_turtle(alive_turtles_);
        }
        spawn_turtle_on = true;
    }

    void callbackPoseSubscription(const std::shared_ptr<turtlesim::msg::Pose> pose){
        pose_.x = pose->x;
        pose_.y = pose->y;
        pose_.theta = pose->theta;
    }

    void find_closest_turtle(const std::vector<my_robot_interfaces::msg::Turtle> &alive_turtles){
        auto best_match = my_robot_interfaces::msg::Turtle();
        double min_dist = INFINITY;
        double best_diff_theta;
        for (auto turtle : alive_turtles){
            double dx = turtle.x - pose_.x;
            double dy = turtle.y - pose_.y;
            double dist = sqrt(dx*dx + dy*dy);
            if (dist < min_dist){
                min_dist = dist;
                best_match = turtle;
                best_diff_theta = atan2(dy,dx);
            }
        }
        closest_turtle = best_match;
        steering_angle = best_diff_theta;
    }

    void controlLoop(){

        if (spawn_turtle_on && (closest_turtle.name != "")){
            // RCLCPP_INFO(this->get_logger(), "closest_turtle x: %.2f, y: %.2f, name: %s", closest_turtle.x, closest_turtle.y, closest_turtle.name.c_str());
            // RCLCPP_INFO(this->get_logger(), "pose_ x: %.2f, y: %.2f", pose_.x, pose_.y);
            double dx = closest_turtle.x - pose_.x;
            double dy = closest_turtle.y - pose_.y;
            double dist = sqrt(dx*dx + dy*dy);

            auto cmd = geometry_msgs::msg::Twist();
            if (dist > 0.5){
                cmd.linear.x = 2*dist;
                double steering_angle = atan2(dy,dx);
                double angle_diff = (steering_angle-pose_.theta);
                if (angle_diff > M_PI)
                {
                    angle_diff -= 2 * M_PI;
                }
                else if (angle_diff < -M_PI)
                {
                    angle_diff += 2 * M_PI;
                }

                cmd.angular.z = 6*angle_diff;
    
            }
            else{
                cmd.linear.x = 0;
                cmd.angular.z = 0;
                RCLCPP_INFO(this->get_logger(), "Reached target turtle, send kill request %s", closest_turtle.name.c_str());

                catchTargetTurtle(closest_turtle.name);
                closest_turtle.name = "";
            }
            cmd_publisher_->publish(cmd);


        }

    }   

    void catchTargetTurtle(const std::string &turtle_name){
        // auto catch_request = my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr(); //Do not use this, this create a null-shared pointer, leads to undefined behavior
        auto catch_request = std::make_shared<my_robot_interfaces::srv::CatchTurtle::Request>();
        catch_request->name = turtle_name;
        RCLCPP_INFO(this->get_logger(), "Going to send request %s", turtle_name.c_str());

        catch_turtle_client_->async_send_request(catch_request, std::bind(&TurtleController::callbackCatchTargetTurtle, this, _1));
    }

    void callbackCatchTargetTurtle(rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedFuture future){
        auto response = future.get();
        if (!response->success){
            RCLCPP_ERROR(this->get_logger(), "Failed to remove turtle");
        }
    }
    rclcpp::Subscription<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtle_sub_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Client<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_client_;
    turtlesim::msg::Pose pose_;
    double steering_angle;
    my_robot_interfaces::msg::Turtle closest_turtle;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_publisher_;
    bool spawn_turtle_on = false;
};

int main(int argc, const char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleController>());
    rclcpp::shutdown();
}