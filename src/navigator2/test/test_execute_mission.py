import os
import sys
import time
import unittest

import launch
import launch_ros
import launch_testing.actions
import rclpy
from turtlesim.msg import Pose
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from drone_interfaces.msg import Report
from navigator_interfaces.srv import FreeDrone
from std_srvs.srv import SetBool
from nav_msgs.msg import Odometry



def generate_init_description(context, *args, **kwargs):
    clock_node = Node(
        package='clock_simulation',
        executable='sim_clock',
        name='sim_clock',
        parameters=[{
            'time_scale': 1.0,
            'start_paused': True,
            'publish_rate': 100,
        }],
    )
    drone_node = ComposableNode(
            package='drone_composition',
            plugin='DroneComposition::Drone',
            name='x500_1',
            parameters=[{
                'use_sim_time': True,
                'pose_x': 0.0,
                'pose_y': 0.0,
                'pose_z': 0.0,
            }],
        )
    base_dir = os.path.dirname(os.path.abspath(__file__))
    test_dir = os.path.abspath(os.path.join(base_dir, '../../../tests/test6'))
    controller_node = ComposableNode(
            package='drone_composition',
            plugin='DroneComposition::DroneController',
            name='drone_controller',
            parameters=[{
                'use_sim_time': True,
                'drones_file': os.path.join(test_dir, 'drones.csv'),
            }],
        )
    
    container_node = ComposableNodeContainer(
        name='drone_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container_mt',
        composable_node_descriptions=[drone_node, controller_node],
        output='screen',
    )

    navigation_node = Node(
        package='navigator2',
        executable='navigator2',
        name='navigator',
        parameters=[{
            'use_sim_time': True,
            'missions_file': os.path.join(test_dir, 'missions.csv'),
            'graph_file': os.path.join(test_dir, 'graph.csv'),
            'edges_file': os.path.join(test_dir, 'edges.csv'),
        }]
    )
    
    return [clock_node, container_node, navigation_node]

def generate_test_description():
    return launch.LaunchDescription(
        [
            OpaqueFunction(function=generate_init_description),
            launch.actions.TimerAction(
                period=1.0, actions=[launch_testing.actions.ReadyToTest()]
            ),
        ]
    )

class TestFullWork(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node('test_node')

    def tearDown(self):
        self.node.destroy_node()

    def start_sim_clock(self):
        client_node = rclpy.create_node('min_client')
        min_client = client_node.create_client(
            SetBool, '/sim_clock/pause'
        )

        self.assertTrue(min_client.wait_for_service(timeout_sec=3.0))
        
        req = SetBool.Request()
        req.data = False

        future = min_client.call_async(req)
        rclpy.spin_until_future_complete(client_node, future, timeout_sec=3)

        self.assertTrue(future.done())

        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.success)
        self.assertEqual(response.message, 'Simulation resumed')

        client_node.destroy_node()
   
        return response.success
   

    def test_only_execute_mission(self, proc_output):
        """Check execute one mission"""
        reports_rx = []
        sub = self.node.create_subscription(
            Report, '/x500_1/report',
            lambda msg: reports_rx.append(msg), 100
        )
        
        odom_rx = []
        sub_odom = self.node.create_subscription(
            Odometry, '/x500_1/odometry',
            lambda msg: odom_rx.append(msg), 100
        )

        expected_poses = [
            (0.0, 0.0, 0.0),
            (15.0, 15.0, 15.0),
            (35.0, 35.0, 35.0)
        ]
        
        self.start_sim_clock()

        end_time = time.time() + 15
        while time.time() < end_time:
            rclpy.spin_once(self.node, timeout_sec=0.1)

        self.assertEqual(len(reports_rx), 2)
        self.assertEqual(reports_rx[0].id, 1)
        self.assertEqual(reports_rx[0].model, 'x500')
        self.assertEqual(reports_rx[0].state, 1)
        self.assertLess(abs(reports_rx[0].pose.position.x - 0.0), 0.5)
        self.assertLess(abs(reports_rx[0].pose.position.y - 0.0), 0.5)
        self.assertLess(abs(reports_rx[0].pose.position.z - 0.0), 0.5)

        self.assertEqual(reports_rx[1].id, 1)
        self.assertEqual(reports_rx[1].model, 'x500')
        self.assertEqual(reports_rx[1].state, 0)
        self.assertLess(abs(reports_rx[1].pose.position.x - 35.0), 0.5)
        self.assertLess(abs(reports_rx[1].pose.position.y - 35.0), 0.5)
        self.assertLess(abs(reports_rx[1].pose.position.z - 35.0), 0.5)

        expected_id = 0
        tolerance = 0.1
        cur_sqdistance = (
            (odom_rx[0].pose.pose.position.x - expected_poses[0][0])**2 +
            (odom_rx[0].pose.pose.position.y - expected_poses[0][1])**2 +
            (odom_rx[0].pose.pose.position.z - expected_poses[0][2])**2
        )
        for i in odom_rx:
            new_sqdistance = (
                (i.pose.pose.position.x - expected_poses[expected_id][0])**2 +
                (i.pose.pose.position.y - expected_poses[expected_id][1])**2 +
                (i.pose.pose.position.z - expected_poses[expected_id][2])**2
            )
            self.assertLess(new_sqdistance, cur_sqdistance + tolerance)

            if (new_sqdistance < cur_sqdistance):
                cur_sqdistance = new_sqdistance

            if cur_sqdistance < tolerance and expected_id < len(expected_poses) - 1:
                expected_id += 1
                cur_sqdistance = (
                    (i.pose.pose.position.x - expected_poses[expected_id][0])**2 +
                    (i.pose.pose.position.y - expected_poses[expected_id][1])**2 +
                    (i.pose.pose.position.z - expected_poses[expected_id][2])**2
                )
        