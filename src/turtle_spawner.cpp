#include "rclcpp/rclcpp.hpp"
#include <cmath>
#include "turtlesim/srv/kill.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "my_robot_interfaces/msg/turtle.hpp"
#include "my_robot_interfaces/msg/turtle_array.hpp"
#include "my_robot_interfaces/srv/catch_turtle.hpp"

using namespace std::placeholders;
using namespace std::chrono;

class TurtleSpawnerNode : public rclcpp::Node{
public: 
    TurtleSpawnerNode() : Node("turtle_spawner"), turtle_counter_(0){
        this->declare_parameter("turtle_name_prefix", "turtle");
        this->declare_parameter("spawn_frequency", 1.0);

        turtle_name_prefix = this->get_parameter("turtle_name_prefix").as_string();
        spawn_frequency = this->get_parameter("spawn_frequency").as_double();

        turtle_spawn_client_ = this->create_client<turtlesim::srv::Spawn>("spawn");
        turtle_kill_client_ = this->create_client<turtlesim::srv::Kill>("kill");
        alive_turtle_publisher_ = this->create_publisher<my_robot_interfaces::msg::TurtleArray>("alive_turtles",10);
        catch_turtle_server_ = this->create_service<my_robot_interfaces::srv::CatchTurtle>("catch_turtle",std::bind(&TurtleSpawnerNode::callbackCatchTurtleService, this, _1, _2));
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int)(1000.0 / spawn_frequency)), std::bind(&TurtleSpawnerNode::spawnNewTurtle, this));
    }

private:

    void PublishAliveTurtles(){
        auto msg = my_robot_interfaces::msg::TurtleArray();
        msg.living_turtles = alive_turtles_;
        alive_turtle_publisher_->publish(msg);
    }

    double RandCoordinate(){
        return double(std::rand()) / (double(RAND_MAX) + 1);
    }

    void spawnNewTurtle(){
        
        turtle_counter_+=1;
        turtle_name_ = turtle_name_prefix + std::to_string(turtle_counter_);
        double x = RandCoordinate() * 10.0;
        double y = RandCoordinate() * 10.0;
        double theta = RandCoordinate() * 2 * M_PI;
        
        callSpawnTurtleService(turtle_name_, x, y, theta);

    }

    void callSpawnTurtleService(std::string turtle_name, double x, double y, double theta){
        
        auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
        request->name = turtle_name;
        request->x = x;
        request->y = y;
        request->theta = theta;

        // auto turtle_to_save_ = my_robot_interfaces::msg::Turtle(); //Do not use this, this create a null-shared pointer, leads to undefined behavior
        turtle_to_save_.name = turtle_name;
        turtle_to_save_.x = x;
        turtle_to_save_.y = y;
        turtle_to_save_.theta = theta;

        turtle_spawn_client_->async_send_request(request, std::bind(&TurtleSpawnerNode::callbackSpawnTurtleService, this, _1));
    }

    void callbackSpawnTurtleService(rclcpp::Client<turtlesim::srv::Spawn>::SharedFuture future){
        auto response = future.get();
        if (response->name != ""){            
            alive_turtles_.push_back(turtle_to_save_);
            PublishAliveTurtles();
        }   
    }

    void callbackCatchTurtleService(my_robot_interfaces::srv::CatchTurtle::Request::SharedPtr request,
                                    my_robot_interfaces::srv::CatchTurtle::Response::SharedPtr response){
        turtle_to_kill_ = request->name;
        RCLCPP_INFO(this->get_logger(), "Receive kill request %s", turtle_to_kill_.c_str());
        // auto turtle_to_kill_request = turtlesim::srv::Kill::Request::SharedPtr();
        auto turtle_to_kill_request = std::make_shared<turtlesim::srv::Kill::Request>();
        turtle_to_kill_request->name = turtle_to_kill_;                                
        
        //Call Kill service
         while(!turtle_kill_client_->wait_for_service(1.0s)){
            RCLCPP_INFO(this->get_logger(), "Waiting for Turtlesim Node to start...");
         }

         RCLCPP_INFO(this->get_logger(), "Now send kill request %s", turtle_to_kill_.c_str());

         turtle_kill_client_->async_send_request(turtle_to_kill_request,std::bind(&TurtleSpawnerNode::callbackTurtleKillRequest, this, _1));

         response->success = true;
    }

    void callbackTurtleKillRequest(rclcpp::Client<turtlesim::srv::Kill>::SharedFuture future){
        auto response = future.get();
        for(int i = 0; i < (int)alive_turtles_.size(); i++){
            if(alive_turtles_.at(i).name == turtle_to_kill_){
                alive_turtles_.erase(alive_turtles_.begin() + i);
                PublishAliveTurtles();
                break;
            }
        }
    }

    int turtle_counter_;
    std::string turtle_name_prefix;
    float spawn_frequency;
    std::string turtle_name_;
    std::vector<my_robot_interfaces::msg::Turtle> alive_turtles_;
    rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr turtle_spawn_client_;
    rclcpp::Client<turtlesim::srv::Kill>::SharedPtr turtle_kill_client_;
    rclcpp::Publisher<my_robot_interfaces::msg::TurtleArray>::SharedPtr alive_turtle_publisher_;
    rclcpp::Service<my_robot_interfaces::srv::CatchTurtle>::SharedPtr catch_turtle_server_;
    rclcpp::TimerBase::SharedPtr timer_;
    my_robot_interfaces::msg::Turtle turtle_to_save_;
    std::string turtle_to_kill_;

};


int main(int argc, char* argv[]){
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TurtleSpawnerNode>());
    rclcpp::shutdown();
}