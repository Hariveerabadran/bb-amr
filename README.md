# BB-Bot: Autonomous Café Assistant

An autonomous butler robot designed for the **French Door Café**, capable of efficiently handling and delivering food orders using **ROS 2**, **Gazebo**, and **Nav2**. The robot simulates a café environment where it reduces human workload and improves service speed during busy hours.

## Objective

To create a mobile robot system that autonomously:
- Accepts and manages customer orders
- Navigates between kitchen and tables
- Handles multiple real-world scenarios like order cancellation, no responses, and chaining deliveries

## Behavior Logic

The robot follows these logic flows:

- **Normal Order:** Home → Kitchen → Table → Kitchen → Home
- **Order Cancelled (before pickup):** Home → Kitchen (skip table) → Home
- **Order Cancelled (on delivery):** Home → Kitchen → Table (cancelled) → Kitchen → Home
- **No Response at Table:** Skips to next delivery
- **Multiple Orders:** Chains deliveries to multiple tables

## Project Structure

### ROS 2 Workspace: `bb-amr_ws`

- pkg: **cafe_interfaces**  
  Custom `.srv` file for service-client communication:
  ```
  string tables
  bool order
  bool cancel
  string mode
  ---
  string msg
  ```

- pkg: **gazebo_sim**  
    Gazebo & RViz simulation environment:

  ### Launch simulation: 
  ```sh
      ros2 launch gazebo_sim gazebo.launch.py
  ```
  ### Launch Nav2 in RViz:  
  ```sh
      ros2 launch gazebo_sim nav2.launch.py
  ```
  ### Teleop control:  
  ```sh
      ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/cmd_vel_key
  ```
  
  ***Note: Twist_mux is used for here priority control keyboard (/cmd_vel_key) over nav2 (/cmd_vel) out:= (/bb_botamr/cmd_vel )***

  ### Remapped Topics:
    > amr: /cmd_vel → /bb_botamr/cmd_vel  
    > lidar: /scan → /bb_botamr/scan


- pkg: **bb_ui**  
  PyQt5-based GUI client to interact with the robot using ROS 2 services and Nav2:

  ### Start service node:
  ```sh  
      ros2 run bb_ui cafe_service
  ```
  ### Start UI client:  
  ```sh
      ros2 run bb_ui ui
  ```
  ###  Shortcut alias (add to `.bashrc`):
  ```sh
      alias bb_ui='python3 /src/bb_ui/bb_ui/ui_main.py'
  ```
  
## UI Design (PyQt5)

  ![1](https://github.com/user-attachments/assets/c12364b4-ca07-4daa-bf6c-419c0f6a2537)

**GUI elements include:**
- **Order** — Place an order
- **Cancel** — Cancel an order
- **Received** — Confirm order received
- **Order Taken** — Confirm kitchen order
- **Clear** — Clear kitchen log

Each button triggers a ROS 2 service request, handled by the backend logic.

## Simulation Output
- ### Gazebo Simulation
  ![3](https://github.com/user-attachments/assets/6bfedb35-1e40-4519-9898-0795da80771c)

  
- ### RViz Display
  ![4](https://github.com/user-attachments/assets/f92714b1-65d8-4472-b375-72067bb5adf6)

  
- ### Nav2 Costmap
  ![2](https://github.com/user-attachments/assets/2b8a36fc-d3b5-405c-9ad7-a10fc2e0124b)
 
- ### link : Video: [Watch Video](https://drive.google.com/file/d/1WwZBbk_NA83ExoaZOY6AEDBvepFqq9jq/view?usp=sharing)

## Conclusion

This project showcases a ROS 2-based solution for mobile service robots, integrating:
- Custom service interfaces
- UI-based client control
- Nav2 autonomous navigation
- Real-world logic handling

## Dependencies

- ROS 2 (tested with Humble)
- Gazebo (outdated !)
  ```sh
  sudo apt install ros-humble-gazebo-ros-pkgs
  ```
- Slam toolbox
  ```sh
  sudo apt installros-humble-slam-toolbox
  ```
- Nav2 stack
  ```sh
  sudo apt install ros-humble-nav2-bringup
  ```
- twist mux
  ```sh
  sudo apt install ros-humble-twist-mux
  ```
- PyQt5
  ```sh
  pip install pyqt5
  ```
