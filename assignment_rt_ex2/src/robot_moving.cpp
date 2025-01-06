#include <string>
#include <chrono>
#include <memory>
#include <cinttypes>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"



class robot_pathway:public rclcpp::Node
{
public:
	robot_pathway():Node("my_robot_pathway")
	{
		// initialization pulisher
		publisher_robot_velocity_ =this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
		
		//initialization subscriber
		subscription_robot_position_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&robot_pathway::get_pos_callback, this, std::placeholders::_1));
		subscription_robot_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10, std::bind(&robot_pathway::get_vel_callback, this, std::placeholders::_1));

		
		timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&robot_pathway::move_callback, this));
		
	}
	

	
	// function to set velocty
	void set_velocity(geometry_msgs::msg::Twist vel){
		//RCLCPP_INFO(this->get_logger(), "settatt velocita %f %f",vel.linear.x, vel.linear.y);
		robot_vel_=vel;
		publish_velocity();
	}		
	void publish_velocity(){
		publisher_robot_velocity_->publish(robot_vel_);
	}
private:
	// movement as S shape
	void move_callback(){
		geometry_msgs::msg::Twist final_vel;
		float radius = 0.7;


	if (pos_x_ > 8 - radius) {
	    final_vel.linear.x = robot_vel_.linear.x;  
	    final_vel.angular.z = robot_vel_.linear.x / radius; 
	} else if (pos_x_ < -8 + radius) {
	    final_vel.linear.x = robot_vel_.linear.x;
	    final_vel.angular.z = -robot_vel_.linear.x / radius;  
	} else if (pos_y_ > 8 - radius) {
	    final_vel.linear.x = robot_vel_.linear.x;  
	    final_vel.angular.z = robot_vel_.linear.x / radius;  
	} else if (pos_y_ < -8 + radius) {
	    final_vel.linear.x = robot_vel_.linear.x;  
	    final_vel.angular.z = -robot_vel_.linear.x / radius;  
	} else if ((pos_y_ >= 10 && pos_y_ <= 13) || 
		   (pos_y_ <= -10 && pos_y_ >= -13) || 
		   (pos_x_ >= 13 && pos_x_ <= 16) || 
		   (pos_x_ <= -13 && pos_x_ >= -16)) {
	    final_vel.linear.x = 0;
	    final_vel.linear.y = 0;
	} else {
	    final_vel.linear.x = robot_vel_.linear.x;
	    final_vel.angular.z = 0.0;  
	}
		
		

		robot_vel_ = final_vel;
		publish_velocity();
	}

	// callback for position
	void get_pos_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

		pos_x_= msg->pose.pose.position.x;
		pos_y_= msg->pose.pose.position.y;
	}
	// callback for velocity
	void get_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){

		robot_vel_ = *msg;
	}

	// attributes
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_robot_position_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_robot_velocity_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_robot_velocity_;


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

