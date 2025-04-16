#! /usr/bin/env python3

import rclpy
from rclpy.node import Node
from cafe_interfaces.srv import Order
from tf_transformations import quaternion_from_euler

from threading import Thread
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time

class ROS2Service(Node):

    def __init__(self):
        super().__init__('CAFE_server')
        self.srv = self.create_service(Order, 'order_gui', self.inc_table)

        self.navigator = BasicNavigator()
        self.navigator.waitUntilNav2Active()

        self.yaw = [1.57,3.14, 0.0, -1.57]
        self.table_list={'H':(-1.5, -4.3, *quaternion_from_euler(0,0,self.yaw[0],'ryxz')[2:4]), 
                         'K':(-6.0, -3.5, *quaternion_from_euler(0,0,self.yaw[1],'ryxz')[2:4])}
        self.mode=None
        self.current_state = 'idle'
        self.skip_table=list()
        self.skip = False

        Thread(target=self.run_nav, daemon=True).start()

    def inc_table(self, request, response):
        self.get_logger().info(f'Incoming request  table:{request.tables} order:{request.order} cancel:{request.cancel} mode:{request.mode}')

        if request.cancel:
            self.get_logger().info(f"Cancel received for table {request.tables}")
            self.get_logger().info("Navigation task canceled.")

            if self.current_state == 'to_kitchen':
                self.get_logger().info("Canceled before reaching kitchen. Returning home.")
                self.navigator.cancelTask()
                self.move_pos('Home', 'H')
                self.table_list.clear()
                self.table_list={'H':(-1.5, -4.3, *quaternion_from_euler(0,0,self.yaw[0],'ryxz')[2:4]), 
                         'K':(-6.0, -3.5, *quaternion_from_euler(0,0,self.yaw[1],'ryxz')[2:4])}

            elif self.current_state == 'to_table':
                self.get_logger().info(f"Table{request.tables} canceled ")
                self.skip_table.append(request.tables)

            else:
                self.get_logger().info("Robot is idle or returning. No action needed.")

            response.msg = f"Canceled order for table {request.tables}"

        elif request.tables and request.order:            
            if request.tables == '1': self.table_list[request.tables] = (3.0, 0.4, *quaternion_from_euler(0,0,self.yaw[2],'ryxz')[2:4])
            elif request.tables == '2': self.table_list[request.tables] = (8.5, 0.4, *quaternion_from_euler(0,0,self.yaw[2],'ryxz')[2:4])
            elif request.tables == '3': self.table_list[request.tables] = (4.0, -3.0, *quaternion_from_euler(0,0,self.yaw[3],'ryxz')[2:4])

            self.get_logger().info(f"Order received for table {request.tables}")
            response.msg = f"Order registered for table {request.tables}"

        elif request.mode == 'Kc':
            self.get_logger().info("Kitchen confirmation received.")
            response.msg = "Kitchen confirmed"
            self.mode = request.mode
        
        elif request.tables and request.mode in ['T1', 'T2', 'T3']:
            self.get_logger().info(f"Confirmation received from table {request.tables}.")
            response.msg = f"Table {request.mode} confirmed"
            self.mode = request.mode

        return response

    def run_nav(self):
        while rclpy.ok():
            key = list(self.table_list.keys())
            if len(key) > 2:
                self.move_pos('Kitchen', 'K')
                if not self.wait_for_confirmation('Kc', timeout=10):
                    self.get_logger().info("Kitchen did not confirm. Returning home.")
                    self.move_pos('Home','H')
                    self.table_list.pop(key[2])

            while len(self.table_list.keys()) > 2:
                self.skip = False
                key = list(self.table_list.keys())
                current_table = key[2]

                if current_table in self.skip_table:
                    self.table_list.pop(current_table)
                    self.skip_table.remove(current_table)
                    self.skip = True
                    self.get_logger().info(f"Skipping table {current_table}")
                    continue

                self.move_pos(f'Table {current_table}', current_table)
                if not self.wait_for_confirmation(f'T{current_table}', timeout=10):
                    self.get_logger().info(f"No confirmation at table {current_table}. Returning to kitchen then home.")
                    self.skip = False
                    self.move_pos('Kitchen', 'K')
                    time.sleep(1)
                    self.move_pos('Home', 'H')

                self.table_list.pop(current_table)

            if self.skip:
                self.move_pos('Kitchen', 'K')
            time.sleep(1)
            if self.current_state != 'idle' and self.current_state != 'returning':
                self.move_pos('Home', 'H')

    def move_pos(self,pos_name:str, pos:str):
        if pos == 'K':
            self.current_state = 'to_kitchen'
        elif pos.isnumeric():
            self.current_state = 'to_table'
        else:
            self.current_state = 'returning'
    
        goal_pose = self._create_pose(*self.table_list.get(pos))
        self.navigator.goToPose(goal_pose)
        while not self.navigator.isTaskComplete():
            feedback = self.navigator.getFeedback()
            self.get_logger().info(f"Heading to {pos_name}...")

    def wait_for_confirmation(self, expected_mode, timeout=10):
        start_time = time.time()
        while time.time() - start_time < timeout:
            if self.mode == expected_mode:
                self.mode = None
                return True
            self.mode = None
            time.sleep(1)
        return False 
    
    def _create_pose(self, x, y, z, w):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = z
        pose.pose.orientation.w = w
        return pose
    
def main(args=None):
    rclpy.init(args=args)
    service_node = ROS2Service()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(service_node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        service_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()