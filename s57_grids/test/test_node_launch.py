# Copyright (c) 2026 Roland Arsenault
# Licensed under BSD license

"""Launch test for s57_grids_node."""

import unittest

import launch
from launch_ros.actions import LifecycleNode
import launch_testing
import launch_testing.actions
import launch_testing.asserts

import rclpy
from rclpy.node import Node


def generate_test_description():
    node = LifecycleNode(
        package='s57_grids',
        executable='s57_grids_node',
        name='s57_grids',
        namespace='',
        emulate_tty=True,
    )
    return launch.LaunchDescription([
        node,
        launch_testing.actions.ReadyToTest(),
    ])


class TestS57GridsNode(unittest.TestCase):
    """Tests that run while the node is alive."""

    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = Node('test_s57_grids')

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def test_node_is_discovered(self):
        """The s57_grids node should appear in the ROS graph."""
        discovered = False
        for _ in range(20):
            node_names = self.node.get_node_names()
            if 's57_grids' in node_names:
                discovered = True
                break
            rclpy.spin_once(self.node, timeout_sec=0.5)
        self.assertTrue(
            discovered,
            f's57_grids not found in node list: {node_names}'
        )


@launch_testing.post_shutdown_test()
class TestS57GridsShutdown(unittest.TestCase):
    """Tests that run after the node is shut down."""

    def test_exit_code(self, proc_info):
        launch_testing.asserts.assertExitCodes(
            proc_info,
            allowable_exit_codes=[0, -2, -15],  # OK, SIGINT, SIGTERM
        )
