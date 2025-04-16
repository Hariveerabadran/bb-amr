#! /usr/bin/env python3

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from tf_transformations import quaternion_from_euler


def main():
    rclpy.init()
    poses = [1.57, 0.0, 3.14]
    qua_list= []
    qua = ()
    for i in range(0, len(poses)):
        qua = quaternion_from_euler(0,0, poses[i],'ryxz')
        qua_list.append(qua)

    print(qua_list)
    nav2 = BasicNavigator()

    i_pos = PoseStamped()
    i_pos.header.frame_id = 'map'
    i_pos.header.stamp = nav2.get_clock().now().to_msg()
    i_pos.pose.position.x = -1.5
    i_pos.pose.position.y = -4.3
    i_pos.pose.orientation.z = qua_list[0][2]
    i_pos.pose.orientation.w = qua_list[0][3]
    nav2.setInitialPose(i_pos)

    nav2.waitUntilNav2Active()

    goal_poses = []
    goal_1 = PoseStamped()
    goal_1.header.frame_id = 'map'
    goal_1.header.stamp = nav2.get_clock().now().to_msg()
    goal_1.pose.position.x = 0.0
    goal_1.pose.position.y = 0.0
    goal_1.pose.orientation.z = qua_list[0][2]
    goal_1.pose.orientation.w = qua_list[0][3]
    goal_poses.append(goal_1)

    goal_2 = PoseStamped()
    goal_2.header.frame_id = 'map'
    goal_2.header.stamp = nav2.get_clock().now().to_msg()
    goal_2.pose.position.x = 1.86
    goal_2.pose.position.y = 2.56
    goal_2.pose.orientation.z = qua_list[1][2]
    goal_2.pose.orientation.w = qua_list[1][3]
    goal_poses.append(goal_2)

    goal_3 = PoseStamped()
    goal_3.header.frame_id = 'map'
    goal_3.header.stamp = nav2.get_clock().now().to_msg()
    goal_3.pose.position.x = 0.0
    goal_3.pose.position.y = 0.0
    goal_3.pose.orientation.z = qua_list[2][2]
    goal_3.pose.orientation.w = qua_list[2][3]
    goal_poses.append(goal_3)

    nav_start = nav2.get_clock().now()
    nav2.followWaypoints(goal_poses)

    i = 0
    while not nav2.isTaskComplete():
        i = i + 1
        feedback = nav2.getFeedback()
        if feedback and i % 5 == 0:
            print('Executing current waypoint: ' + str(feedback.current_waypoint + 1) + '/' + str(len(goal_poses)))

    result = nav2.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    nav2.lifecycleShutdown()

    exit(0)


if __name__ == '__main__':
    main()