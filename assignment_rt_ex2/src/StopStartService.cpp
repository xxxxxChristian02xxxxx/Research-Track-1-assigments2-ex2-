/**
 * \file StopStartService.cpp
 * \brief StopStartService to manage the stop and sart of the robot through service
 * \author Christian Negri Ravera
 * \version 0.1
 * \date 27/02/2025
 * 
 * \details
 * 
 * Publishes to: <br>
 *  /cmd_vel: [geometry_msg/msg/twist] <br>
 * 
 * Subscribes to: <br>
 *  none <br>
 * 
 * \service: <br>
 *  /stop_start: [service_msgs/srv/stop_start] <br>
 * 
 * Description: <br>
 * The file implemnets the StopStartServer class to manage the service to make the robot strt or stop, based on the choice of the user.
 * If the input to the server is true, the robot will start.
 * If the input to the server is false, the robot will stop.
 * If the robot will restart, the velocity assigned is obtained from the global paramters.
 * The service publishes also the velocity of the robot
 */



#include <inttypes.h>
#include <memory>
#include "service_msgs/srv/stop_start.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include "rclcpp/rclcpp.hpp"

using StopStart = service_msgs::srv::StopStart;
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;



/**
 * \class StopStartServer
 * 
 * Description: <br>
 * The class implements the behavious of the service which allows the user to stop or start the robot.
 * The service is called /stop_start and it is of type StopStart.
 */
class StopStartServer : public rclcpp::Node
{
public:

/**
 * \brief StopStartServer, cosntructor of the class
 * \param none
 * \return none
 * 
 * Description: <br>
 * The constructor creates the instances of the service and publisher of velocity and binds them to the respective variables service_ and publisher_.
 * The variable stopped_ is initialized to false, meaning that the robot is moving.
 */
  StopStartServer(): Node("stop_start_server")
  {
    service_ = this->create_service<service_msgs::srv::StopStart>("/stop_start", std::bind(&StopStartServer::handle_service, this, _1, _2, _3));    ///< creation of the servce and binding to the service variable
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);   ///< creation of the publisher of the velocity
    stopped_ = false;        ///< assign te value false to the var stopped: robot is moving, useful to process better the state of the robot
  }
private:

/**
 * \brief handle_service, function to manage the service behaviour
 * \param request_header header of the request
 * \param request request of the service
 * \param response response of the service
 * \return none
 * 
 * Description: <br>
 * The function is called when the service is requested. It checks the value of the request and, based on the value of the variable stop_start, it stops or starts the robot.
 * if the robots is stopped the velocity is set to zero, otherwise the velocity is set to value of the parameters vel_x and vel_y.
 * The function publishes the velocity of the robot and sets the response to true.
 */
void handle_service(  const std::shared_ptr<rmw_request_id_t> request_header,
                      const std::shared_ptr<service_msgs::srv::StopStart::Request> request,
                      std::shared_ptr<service_msgs::srv::StopStart::Response> response)
    {
        float vel_x = this->get_parameter("vel_x").as_double();   ///< storing the value of the parameter referred to velocity along x
        float vel_y = this->get_parameter("vel_y").as_double();   ///< storing the value of the parameter referred to velocity along y
        geometry_msgs::msg::Twist vel;

        
        if (!request->stop_start){       // checking if the request is to stop the robot: velocity of the robot is set to zero
                vel.linear.x = 0.0;
                vel.linear.y = 0.0;
                vel.angular.z = 0.0;
                stopped_ = true;
            }
        else{                          // checking if the request is to start the robot: velocity of the robot is set to the previous value of the paramter velocity
          if (request->stop_start){
                vel.linear.x = vel_x;
                vel.linear.y = vel_y;
                stopped_ = false;
            }
        }
          publisher_->publish(vel); // publishing the velocity of the robot
          response->success = true; // setting response to true as completed correctly
 }
    rclcpp::Service<service_msgs::srv::StopStart>::SharedPtr service_;    ///< attribute to reference the service
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;   ///< attribute to reference publisher of velocity
    bool stopped_;                                                        ///< attribute to reference the user choice to whether stop or start the robot
   
};


/** 
 * \brief main function
 * \param argc number of arguments
 * \param argv pointer to array of arguments
 * \return always zero, since it is the main function 
 * 
 * Description: <br>
 * The main function initializes the node and spins the node to keep it alive.
 */
int main(int argc, char * argv[])
{
  // initialization of the node
  rclcpp::init(argc, argv);

  // Spinning the node to keep it alive and initilize the service
  rclcpp::spin(std::make_shared<StopStartServer>());
  rclcpp::shutdown();
  return 0;
}