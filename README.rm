# Exercise 2: Robot Movement in Gazebo

## Overview
This exercise involves creating a ROS 2 package called `assignments_rt_ex2` to control the robot's movement in a Gazebo simulation environment. The project uses the `robot_urdf` package for the robot description.

The main node of this project is called **`robot_moving`**, which is responsible for managing the robot's movement.

---

## Robot Moving Node

### Class Description
The **`RobotMoving`** class handles robot movement and interaction within the Gazebo environment.

#### Private Attributes:
- `subscription_robot_position_`:  
  **Type:** `rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr`  
  **Purpose:** Subscribes to the robot's odometry to get its position.

- `subscription_robot_velocity_`:  
  **Type:** `rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr`  
  **Purpose:** Subscribes to the velocity topic to retrieve the robot's current velocity.

- `publisher_robot_velocity_`:  
  **Type:** `rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr`  
  **Purpose:** Publishes velocity commands to control the robot.

- `pos_x_`: **Type:** `float`  
  **Purpose:** Stores the robot's position on the x-axis.

- `pos_y_`: **Type:** `float`  
  **Purpose:** Stores the robot's position on the y-axis.

- `robot_vel_`:  
  **Type:** `geometry_msgs::msg::Twist`  
  **Purpose:** Stores the robot's velocity data.

- `timer_`:  
  **Type:** `rclcpp::TimerBase::SharedPtr`  
  **Purpose:** Calls the `move_callback` method periodically.

---

#### Private Methods:
- `move_callback()`:  
  **Type:** `void`  
  **Purpose:** Decides the robot's movement pattern. Implemented with an "S-shaped" pathway.

- `get_pos_callback(const nav_msgs::msg::Odometry::SharedPtr msg)`:  
  **Type:** `void`  
  **Purpose:** Subscribes to the odometry topic and retrieves the robot's position.

- `get_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)`:  
  **Type:** `void`  
  **Purpose:** Subscribes to the `cmd_vel` topic to obtain the robot's velocity.

---

#### Public Methods:
- `set_velocity(geometry_msgs::msg::Twist vel)`:  
  **Type:** `void`  
  **Purpose:** Allows the user to set the robot's velocity manually.

- `publish_velocity()`:  
  **Type:** `void`  
  **Purpose:** Publishes the robot's velocity commands.

- Constructor:  
  Initializes subscribers, publishers, and the timer for periodic calls to `move_callback`.

---

### Adding the Graph

#### Generated RQT Graph
To visualize the node and topic connections in this package, the following graph has been generated:

![RQT Graph](./grafo_rt1_ex2.png)

---


