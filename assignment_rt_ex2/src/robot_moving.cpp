#include <string>
#include <chrono>
#include <memory>
#include <cinttypes>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odom.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/srv/spawnentity.hpp"

using std::placeholders::_1;


class robot_pathway:public rclcpp::Node
{
public:
	robot_pathway():Node("my_robot_pathway")
	{
		// initialization pulisher
		publisher_robot_velocity_ =this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
		
		//initialization subscriber
		subscription_robot_position_ = this->create_subscription<nav_msgs::msg::Odom>("/odom", 10, std::bind(&robot_pathway::get_pos_callback, this, _1));
		subscription_robot_velocity_= this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10, std::bind(&robot_pathway::get_vel_callback, this, _1));

		// initialization client1
		client2_= this->create_client<turtlesim::srv::Spawn>("/spawn");
		// initialization client2	
		client1_= this->create_client<turtlesim::srv::Kill>("/kill");
		
		while(!client1_->wait_for_service(std::chrono::seconds(1))){
			if (!rclcpp::ok()) {
				RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
		}
		
		while(!client2_->wait_for_service(std::chrono::seconds(1))){
			if(!rclcpp::ok()){
				RCLCPP_ERROR(this->get_logger(), "client interrupted while waiting for service to appear.");
				return;
			}
			RCLCPP_INFO(this->get_logger(), "waiting for service to appear...");
		}
		timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&robot_pathway::move_callback, this));
		
	}
		//function to call the kill service
	void call_client1(const std::string &turtle_name){
		// instance for the request
		auto request = std::make_shared<turtlesim::srv::Kill::Request>();
		// giving params
		request->name = turtle_name;
	  	// calling service
	 	auto result = client1_->async_send_request(request);
        	RCLCPP_INFO(this->get_logger(), "called service'/kill' for '%s'", turtle_name.c_str());
    	}
		  
	//function to call the spawn service
	void call_client2(float pos_x,float pos_y, float theta, const std::string &turtle_name){
		// declaration of the request
		auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
		// filling the request
		request->x = pos_x;
		request->y = pos_y;
		request->theta = theta;
		request->name = turtle_name;
		// sending request
		auto result = client2_->async_send_request(request);
	}
		
	void set_velocity(geometry_msgs::msg::Twist vel){
			RCLCPP_INFO(this->get_logger(), "settatt velocita");
		robot_vel_=vel;
		publisher_robot_velocity_->publish(robot_vel_);
	}		
				
	
		
		
	
private:
	void move_callback(){
		geometry_msgs::msg::Twist final_vel;
					RCLCPP_INFO(this->get_logger(), " dentro: %f ", pos_x_);
		float radius = 1;
					
		if(pos_x_ > 10-radius){
			final_vel.linear.x = robot_vel_.linear.x;  
			final_vel.angular.z = robot_vel_.linear.x /radius;  
		} else if(pos_x_ < radius+1){
			RCLCPP_INFO(this->get_logger(), " dentro: %f ", pos_x_);
			final_vel.linear.x = robot_vel_.linear.x;
			final_vel.angular.z = -robot_vel_.linear.x/(radius);  
		} else {
			final_vel.linear.x = robot_vel_.linear.x;
			final_vel.angular.z = 0.0;  
		}
		
		
		if (pos_x_ >= 10-radius  && pos_x_ <=10 && pos_y_ >= 10  && pos_y_ <=10  ){
			RCLCPP_INFO(this->get_logger(), "qui");
			final_vel.linear.x =0;
			final_vel.linear.y =0;
		}
		
		robot_vel_=final_vel;
		publisher_robot_velocity_->publish(robot_vel_);
		
		
		
	}
	
	void get_pos_callback(const turtlesim::msg::Pose::SharedPtr msg){
		pos_x_= msg->x;
		pos_y_= msg->y;
		pos_theta_= msg->theta;
	}

	void get_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
		RCLCPP_INFO(this->get_logger(), " velocita Rilevata: %f %f %f",msg->linear.x,msg->linear.y,msg->angular.z);
		robot_vel_ = *msg;
	}

// attributes
	rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr subscription_robot_position_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_robot_velocity_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_robot_velocity_;
	rclcpp::Client<turtlesim::srv::Kill>::SharedPtr client1_;
	rclcpp::Client<turtlesim::srv::Spawn>::SharedPtr client2_;
	float pos_x_;
	float pos_y_;
	float pos_theta_;
	geometry_msgs::msg::Twist robot_vel_;
	rclcpp::TimerBase::SharedPtr timer_;
	

};

int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<robot_pathway>();
 
  geometry_msgs::msg::Twist vel_input ;
  std::cout<< "give linear velocity along x"<< std::endl;std::cin >> vel_input.linear.x;
  std::cout<< "give linear velocity along y"<< std::endl;std::cin >> vel_input.linear.y;
  node->set_velocity(vel_input);
  

  
  
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  
  
  return 0;
}

