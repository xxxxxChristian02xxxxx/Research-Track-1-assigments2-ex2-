/**
 * \file robot_moving.cpp
 * \brief this files deals with the robot movement in an S shape, given a input veloicty
 * \author Christian Negri Ravera
 * \version 0.1
 * \date 27/02/2025
 * 
 * 
 * \details
 * 
 * Publishes to: <br>
 *  /cmd_vel: [geometry_msgs/msg/Twist] <br>
 *  /num: [service_msgs/msg/num] <br>
 *  /odom_feet: [nav_msgs/msg/odometry] <br>
 * 
 * Subscribes to: <br>
 * /cmd_vel:  [geometry_msgs/msg/Twist] <br>
 * /odom: [nav_msgs/msg/odometry] <br>
 * 
 * Services to: <br>
 * /get_robot_posisition: [service_msgs/srv/get_robot_position] <br>
 * 
 * Description: <br>
 * The file shows the implementation of the node robot_pathway: which manages the movement of the robot in an S shape, given a input velocity. <br>
 * It is possible to set the velocity of the robot and, based on how close it is to the border of the map , it stops 
 * Besides, some topics are exploited to obtain and publish the position and velocity of the robot.
 * Usage of the custom message /num gives the possibility of seeing the counting going on
 * Finally, the connection to the server gives the possibility to the user to start or stop the robot, which require the past velocity (parameters)
 */



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
using GetRobotPosition = service_msgs::srv::GetRobotPosition; ///< alias for the service to get the robot position


/**
 * \class robot_pathway
 * This class will deal with the robot movement and interface with the user to set the velocity
 */
class robot_pathway:public rclcpp::Node
{
public:
	/**
	 * \brief robot_pathway, constructor of the class
	 * \param none 
	 * \return none
	 * 
	 * Description: <br>
	 * Initializes all publishers and subscribers, as long as the global parameters used to communicate with the server
	 */
	robot_pathway():Node("my_robot_pathway")
	{
		publisher_robot_velocity_ =this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel",10); 	///<initialization publisher of the velocity 
		publisher_num_ = this->create_publisher<service_msgs::msg::Num>("/num",10); 					///<initialization publisher of the custom message increasing num 
		pos_feet_robot_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_feet",10); 	///<initialization publisher of the position of robot in feet

		subscription_robot_position_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&robot_pathway::get_pos_callback, this, std::placeholders::_1)); 		///<initialization subscription of the position
		subscription_robot_velocity_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel",10, std::bind(&robot_pathway::get_vel_callback, this, std::placeholders::_1));	///<initialization subscription of the position

		timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&robot_pathway::move_callback, this)); 						///<initialization timer to move the robot each 0.5 seconds
		timer2_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&robot_pathway::num_callback, this));						///<initialization timer to publish the number each 0.5 seconds
		timer3_pos_feet_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&robot_pathway::pos_robot_feet_callback, this)); 	///<initialization timer to publish robot position in feet each 0.5 seconds
		count_ = 0; 	///<initialization counter for the custom message

		/**
		 * \brief Initializes parameters to store the robot's velocity.
		 *
		 * Description: <br>
		 * These parameters allow the robot to resume movement after being stopped.
		 */
		this -> declare_parameter ( " vel_x " , 0.0 ); 
		this -> declare_parameter ( " vel_y " , 0.0 ); 
	}

	/**
	 * \brief set_velocity, function to set the velocity of the robot
	 * \param vel [geometry_msgs::msg::Twist] velocity to be set
	 * \return none
	 * 
	 * Dscription: <br>
	 * The function obtains the velocity in input from the user and sets the velocity reference
	 */
	void set_velocity(geometry_msgs::msg::Twist vel){
		robot_vel_=vel;
		publish_velocity();
	}

	/**
	 * \brief publish_velocity, function to publish the velocity of the robot
	 * \param none
	 * \return none
	 * 
	 * Description: <br>
	 * The function, when called, publishes the velocity reference to the robot topic
	 * Stores the velocity composition x and y in parameters to ensure consistency in velocity values when the robot is stopped and restarted
	 */		
	void publish_velocity(){
		publisher_robot_velocity_->publish(robot_vel_);
        float vel_x = robot_vel_.linear.x;
        float vel_y = robot_vel_.linear.y;
        this->set_parameter(rclcpp::Parameter("vel_x", vel_x));
        this->set_parameter(rclcpp::Parameter("vel_y", vel_y));
    }
private:
	/**
	 * \brief move_callback, function to manage the motion of the rbot in an S shape 
	 * \param none
	 * \return none
	 * 
	 * Description: <br>
	 * The function is called every 0.5 seconds; at each iteration it checks if the position of the robot is close to the border of the map.
	 * In case the robot is too close to the border it inverts the angular velocity to make the robot turn and do an S shape
	 * In case the robot is not close to the border of the map, it keeps moving with the same velocity
	 * If the border is overpassed the robot stops
	 */
	void move_callback(){
	geometry_msgs::msg::Twist final_vel;
	float radius = 0.7;		///< radius of the curve
	if (pos_x_ > 8 - radius) { 							// right border
	    final_vel.linear.x = robot_vel_.linear.x;  
	    final_vel.angular.z = robot_vel_.linear.x / radius; 
	} 
	else if (pos_x_ < -8 + radius) { 					// left border
	    final_vel.linear.x = robot_vel_.linear.x;
	    final_vel.angular.z = -robot_vel_.linear.x / radius;  
	}
	else if (pos_y_ > 8 - radius) { 					// upper border
	    final_vel.linear.x = robot_vel_.linear.x;  
	    final_vel.angular.z = robot_vel_.linear.x / radius;  
	} 
	else if (pos_y_ < -8 + radius) { 					// lower border
	    final_vel.linear.x = robot_vel_.linear.x;  
	    final_vel.angular.z = -robot_vel_.linear.x / radius;  
	}
	else if ((pos_y_ >= 10 && pos_y_ <= 13) || 			// overpassed border
		   (pos_y_ <= -10 && pos_y_ >= -13) || 
		   (pos_x_ >= 13 && pos_x_ <= 16) || 
		   (pos_x_ <= -13 && pos_x_ >= -16)) { 			
	    final_vel.linear.x = 0;
	    final_vel.linear.y = 0;
	} 
	else {												// inside the boredrs
	    final_vel.linear.x = robot_vel_.linear.x;
	    final_vel.angular.z = 0.0;  
	}
		robot_vel_ = final_vel;
		publish_velocity();
	}

	/**
	 * \brief num_callback, function to publish the cussom message with the count
	 * \param none
	 * \return none
	 * 
	 * Description: <br>
	 * The function in called every 0.5 seconds and assigns the value of the count to the custom message nd published it
	 */
	void num_callback()
	{
		auto message = service_msgs::msg::Num();                             
		message.num = this->count_++;                                        
		publisher_num_->publish(message);
	}

	/**
	 *  \brief get_pos_callback, function to obtain the position of the robot
	 * 	\param none
	 * 	\return none
	 * 
	 * Description: <br>
	 * The function in called every 0.5 seconds and assigns to the coordinate reference variables x and y the position of the robot
	 */
	void get_pos_callback(const nav_msgs::msg::Odometry::SharedPtr msg){

		pos_x_= msg->pose.pose.position.x;
		pos_y_= msg->pose.pose.position.y;
	}

	/**
	 *  \brief get_vel_callback, function to obtain the velocity of the robot
	 * 	\param none
	 * 	\return none
	 * 
	 * Description: <br>
	 * The function in called every 0.5 seconds and assigns to the velocity reference variable the velocity of the robot
	 */
	void get_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg){
		robot_vel_ = *msg;
	}

	/**
	 *  \brief pos_robot_feet_callback, function publishes the position of the robot in feet
	 * 	\param none
	 * 	\return none
	 * 
	 * Description: <br>
	 * The function in called every 0.5 seconds and publishes the value of the reference position variables converted in feet
	 */
	void pos_robot_feet_callback(){
		nav_msgs::msg::Odometry msg;
		msg.pose.pose.position.x = pos_x_*3.28;
		msg.pose.pose.position.y = pos_y_*3.28;
		pos_feet_robot_publisher_->publish(msg);
	}



	rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_robot_position_; 		///< attribute to subscribe to position
	rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_robot_velocity_;	///< attribute to subscribe to velocity
	rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_robot_velocity_; 			///< attribute to publish to position
	rclcpp::Publisher<service_msgs::msg::Num>::SharedPtr publisher_num_; 						///< attribute to publish custom message
	rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr pos_feet_robot_publisher_;			///< attribute to publish to position in feet
	float pos_x_;																				///< attribute to reference coordinate x
	float pos_y_;																				///< attribute to reference coordinate y									
	geometry_msgs::msg::Twist robot_vel_;														///< attribute to reference of the velocity of the robot
	rclcpp::TimerBase::SharedPtr timer_;														///< attribute to timer for motion
	rclcpp::TimerBase::SharedPtr timer2_;														///< attribute to timer for number 
	rclcpp::TimerBase::SharedPtr timer3_pos_feet_;												///< attribute to timer for position in feet
	size_t count_;																				///< attribute to count for custom message 	

};


/**
* \brief main function.
* \param argc num of input argument if any.
* \param argv pointer to array of arguments if any.
* \return always zero since it is the main function.
*
* Description: <br>
* The main function initializes the node and.
* Asks the user to input the linear velocities along x and y and sets the velocity of the robot.
* Spins the node to keep it alive.
*/
int main(int argc, char * argv[])
{
	// initialization of the node
	rclcpp::init(argc,argv);
	// initialize the class robot_pathway
	auto node = std::make_shared<robot_pathway>();

	// asking the user for the velocity and setting the velocity of the robot
	geometry_msgs::msg::Twist vel_input ;		
	std::cout<< "give linear velocity along x"<< std::endl;std::cin >> vel_input.linear.x; 
	std::cout<< "give linear velocity along y"<< std::endl;std::cin >> vel_input.linear.y;
	node->set_velocity(vel_input); 

	// spinning the node
  	rclcpp::spin(node);
  	rclcpp::shutdown();
  
  
  
  	return 0;
}

