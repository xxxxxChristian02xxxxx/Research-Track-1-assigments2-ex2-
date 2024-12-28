#include <string>
#include <chrono>
#include <memory>
#include <cinttypes>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "gazebo_msgs/srv/spawn_entity.hpp"
#include "gazebo_msgs/srv/delete_entity.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2/convert.h"


using std::placeholders::_1;


class robot_pathway:public rclcpp::Node
{
public:
	robot_pathway():Node("my_robot_pathway")
	{
		// initialization pulisher
		publisher_robot_velocity_ =this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
		
		//initialization subscriber
		subscription_robot_position_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry", 10, std::bind(&robot_pathway::get_pos_callback, this, _1));
		subscription_robot_velocity_= this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10, std::bind(&robot_pathway::get_vel_callback, this, _1));

		// initialization client1
		client2_= this->create_client<gazebo_msgs::srv::SpawnEntity>("/spawn_entity");
		// initialization client2	
		client1_= this->create_client<gazebo_msgs::srv::DeleteEntity>("/delete_entity");
		
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
	void call_client1(const std::string &robot_name){
		// instance for the request
		auto request = std::make_shared<gazebo_msgs::srv::DeleteEntity::Request>();
		// giving params
		request->name = robot_name;
	  	// calling service
	 	auto result = client1_->async_send_request(request);
        	RCLCPP_INFO(this->get_logger(), "called service'/kill' for '%s'", robot_name.c_str());
    	}
		  
	//function to call the spawn service
	void call_client2(float pos_x, float pos_y, const std::string &robot_name) {
		auto request = std::make_shared<gazebo_msgs::srv::SpawnEntity::Request>();
		request->name = robot_name;
		//request->xml = "<sdf>...</sdf>"; // inserted for completeness, not using the spawn client
		request->robot_namespace = "/namespace";
		request->initial_pose.position.x = pos_x;
		request->initial_pose.position.y = pos_y;
		request->initial_pose.position.z = 0.0;
		//request->initial_pose.orientation = tf2::toMsg(tf2::Quaternion(0, 0, theta));
		auto result = client2_->async_send_request(request);
	}

	void set_velocity(geometry_msgs::msg::Twist vel){
		RCLCPP_INFO(this->get_logger(), "settatt velocita");
		robot_vel_=vel;
		publish_velocity();
	}		
	void publish_velocity(){
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
		
		publish_velocity();
	}
	
	void get_pos_callback(const nav_msgs::msg::Odometry::SharedPtr msg){
		pos_x_= msg->pose.pose.position.x;
		pos_y_= msg->pose.pose.position.y;
	}

	void get_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
		robot_vel_ = *msg;
	}

	// attributes
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_robot_position_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_robot_velocity_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_robot_velocity_;
	rclcpp::Client<gazebo_msgs::srv::DeleteEntity>::SharedPtr client1_;
	rclcpp::Client<gazebo_msgs::srv::SpawnEntity>::SharedPtr client2_;
	float pos_x_;
	float pos_y_;
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

