#include <string>
#include <chrono>
#include <memory>
#include <cinttypes>
#include <chrono>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "service_msgs/srv/velocity_set.hpp"
#include "service_msgs/srv/get_robot_position.hpp"
#include "service_msgs/msg/num.hpp"

using namespace std::chrono_literals;
using GetRobotPosition = service_msgs::srv::GetRobotPosition;



class robot_pathway:public rclcpp::Node
{
public:
	robot_pathway():Node("my_robot_pathway")
	{
		// initialization pulisher
		publisher_robot_velocity_ =this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10);
		// publisher per il custom msg
		publisher_num_ = this->create_publisher<service_msgs::msg::Num>("/num",10);
		//initialization subscriber
		subscription_robot_position_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&robot_pathway::get_pos_callback, this, std::placeholders::_1));
		subscription_robot_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10, std::bind(&robot_pathway::get_vel_callback, this, std::placeholders::_1));
		pos_feet_robot_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_feet",10);

		//service_ = this->create_service<service_msgs::srv::VelocitySet>("/set_velocity", std::bind(&robot_pathway::set_velocity_service, this, std::placeholders::_1,std::placeholders::_2)); // definisco callback del server
		
		//client_ = this->create_client<service_msgs::srv::GetRobotPosition>("/get_robot_position"); // definisco client
		//client_vel = this->create_client<service_msgs::srv::VelocitySet>("/set_velocity");
		
		timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&robot_pathway::move_callback, this));
		timer2_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&robot_pathway::num_callback, this));
		timer3_pos_feet_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&robot_pathway::pos_robot_feet_callback, this));
		count_ = 0;

	}
	/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
    // function to call the service and to load it lato client
/*     void call_get_robot_position_service(float linear_x)
    {
        std::shared_ptr<GetRobotPosition::Request> request = std::make_shared<service_msgs::srv::GetRobotPosition::Request>();
       	request->x = linear_x;

        while (!client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto result = client_->async_send_request(request);
    	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
  		{
    		RCLCPP_ERROR(this->get_logger(), "service call failed :(");
 		}
 		 this->response_=result.get();
  			    RCLCPP_INFO(this->get_logger(), "fatto1");		
    }
	// variables to store the request and the response client side
  	std::shared_ptr<GetRobotPosition::Response> response_;



	void call_set_velocity_service(float linear_x,float linear_y)
    {
        std::shared_ptr<service_msgs::srv::VelocitySet::Request> request = std::make_shared<service_msgs::srv::VelocitySet::Request>();
       	request->vel_x = linear_x;
		request->vel_y = linear_y;

        while (!client_vel->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        auto result = client_vel->async_send_request(request);
    	if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), result) != rclcpp::FutureReturnCode::SUCCESS)
  		{
    		RCLCPP_ERROR(this->get_logger(), "service call failed :(");
 		}
 		 this->response2_=result.get();
 			    RCLCPP_INFO(this->get_logger(), "fatto2");

    }
	// variables to store the request and the response client side
  	std::shared_ptr<service_msgs::srv::VelocitySet::Response> response2_;


	// service callback function lato server service_, function called to see whats inside the request e fill the renponse 
	// questa funzione è una funzione di server , server side , quindi modifico sia la response che la request
    void set_velocity_service(const std::shared_ptr<service_msgs::srv::VelocitySet::Request> request,
                              std::shared_ptr<service_msgs::srv::VelocitySet::Response> response)
    {
        geometry_msgs::msg::Twist vel;
        vel.linear.x = request->vel_x;
        vel.linear.y = request->vel_y;
        set_velocity(vel);
	    RCLCPP_INFO(this->get_logger(), "Received request: vel_x=%.2f, vel_y=%.2f", vel.linear.x, vel.linear.y);

        response->success = true;
    }
 */
/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////



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

	void num_callback()
	{
		auto message = service_msgs::msg::Num();                               // CHANGE
		message.num = this->count_++;                                        // CHANGE
		publisher_num_->publish(message);
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


	void pos_robot_feet_callback(){
		nav_msgs::msg::Odometry msg;
		msg.pose.pose.position.x = pos_x_*3.28;
		msg.pose.pose.position.y = pos_y_*3.28;
		pos_feet_robot_publisher_->publish(msg);
	}






	// attributes
	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_robot_position_;
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_robot_velocity_;
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_robot_velocity_;
	rclcpp::Publisher<service_msgs::msg::Num>::SharedPtr publisher_num_;
	/* rclcpp::Service<service_msgs::srv::VelocitySet>::SharedPtr service_; // definition of the server for the service set_velocity
	rclcpp::Client<service_msgs::srv::GetRobotPosition>::SharedPtr client_; // definition of the client fot the service get_robot_position
	rclcpp::Client<service_msgs::srv::VelocitySet>::SharedPtr client_vel;
 */


	// publisher for feet position
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pos_feet_robot_publisher_;


	float pos_x_;
	float pos_y_;
	geometry_msgs::msg::Twist robot_vel_;
	rclcpp::TimerBase::SharedPtr timer_;
	rclcpp::TimerBase::SharedPtr timer2_;
	rclcpp::TimerBase::SharedPtr timer3_pos_feet_;
	size_t count_;


};
int tempo=0;
int main(int argc, char * argv[])
{
  rclcpp::init(argc,argv);
  auto node = std::make_shared<robot_pathway>();

  geometry_msgs::msg::Twist vel_input ;
  std::cout<< "give linear velocity along x"<< std::endl;std::cin >> vel_input.linear.x;
  std::cout<< "give linear velocity along y"<< std::endl;std::cin >> vel_input.linear.y;
  node->set_velocity(vel_input);
	RCLCPP_INFO(node->get_logger(), "tempo: %d", tempo);

	
	//if(tempo>20){ 
	/* node->call_set_velocity_service(vel_input.linear.x*10,vel_input.linear.y*10);
	node->call_get_robot_position_service(vel_input.linear.x); */
	//}
	tempo++;	
  rclcpp::spin(node);
  rclcpp::shutdown();
  
  
  
  return 0;
}

