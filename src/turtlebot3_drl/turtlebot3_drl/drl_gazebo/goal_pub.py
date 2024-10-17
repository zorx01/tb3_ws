#!/usr/bin/env python3
#
# Copyright 2019 ROBOTIS CO., LTD.
# Modified by Nandagopan.K on 17/10/2024
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Authors: Ryan Shim, Gilbert, Tomas, Nandu

import os
import random
import math
import numpy
import time


from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path


from gazebo_msgs.srv import DeleteEntity, SpawnEntity
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose

import rclpy
from rclpy.qos import QoSProfile
from rclpy.node import Node

from turtlebot3_msgs.srv import RingGoal
import xml.etree.ElementTree as ET
from ..drl_environment.drl_environment import ARENA_LENGTH, ARENA_WIDTH, ENABLE_DYNAMIC_GOALS
from ..common.settings import ENABLE_TRUE_RANDOM_GOALS
import glob

NO_GOAL_SPAWN_MARGIN = 0.3 # meters away from any wall
class DRLGazebo(Node):
    def __init__(self):
        super().__init__('drl_gazebo')

        """************************************************************
        ** Initialise variables
        ************************************************************"""
        current_dir = os.path.dirname(os.path.abspath(__file__))
        while current_dir and not os.path.exists(os.path.join(current_dir, 'src')):
            current_dir = os.path.dirname(current_dir)

        if not current_dir:
            raise FileNotFoundError("Could not find the 'src' directory in the parent directories.")
        gazebo_path = glob.glob(os.path.join(current_dir, 'src', '**', 'turtlebot3_simulations', 'turtlebot3_gazebo'), recursive=True)
        if not gazebo_path:
            raise FileNotFoundError("Could not find the 'turtlebot3_gazebo' package in the src folder.")
        self.entity_dir_path = os.path.join(gazebo_path[0], 'models', 'turtlebot3_drl_world')            
        self.entity_path = os.path.join(self.entity_dir_path, 'goal_box','model.sdf')
        self.entity = open(self.entity_path, 'r').read()
        self.entity_name = 'goal'

        # with open('/tmp/drlnav_current_stage.txt', 'r') as f:
        self.stage = 9#int(f.read())
        print(f"running on stage: {self.stage}, dynamic goals enabled: {ENABLE_DYNAMIC_GOALS}")

        self.prev_x, self.prev_y = -1, -1
        self.goal_x, self.goal_y = 0.5, 0.0

        """************************************************************
        ** Initialise ROS publishers, subscribers and clients
        ************************************************************"""
        # Initialise publishers
        self.goal_pose_pub = self.create_publisher(Pose, 'goal_pose', QoSProfile(depth=10))

        # Initialise client
        # self.delete_entity_client       = self.create_client(DeleteEntity, 'delete_entity')
        # self.spawn_entity_client        = self.create_client(SpawnEntity, 'spawn_entity')
        # self.reset_simulation_client    = self.create_client(Empty, 'reset_simulation')
        # self.gazebo_pause               = self.create_client(Empty, '/pause_physics')

        # Initialise servers
        self.task_succeed_server    = self.create_service(RingGoal, 'task_succeed', self.task_succeed_callback)
        self.task_fail_server       = self.create_service(RingGoal, 'task_fail', self.task_fail_callback)

        # self.obstacle_coordinates   = self.get_obstacle_coordinates()
        
        self.publisher_ = self.create_publisher(Path, '/path', 10)
        self.subscription = self.create_subscription(PoseStamped, '/turtlebot_pose', self.path_callback, 10)
        self.path_msg = Path()
        self.init_callback()
        self.index_g = 0

    """*******************************************************************************
    ** Callback functions and relevant functions
    *******************************************************************************"""

    def init_callback(self):
        # self.delete_entity()
        # self.reset_simulation()
        self.publish_callback()
        print("Init, goal pose:", self.goal_x, self.goal_y)
        time.sleep(1)
        
    def path_callback(self, msg):
        # Append the received pose to the path
        self.path_msg.header = msg.header
        self.path_msg.poses.append(msg)

        # Publish the updated path
        self.publisher_.publish(self.path_msg)
    def publish_callback(self):
        # Publish goal pose
        goal_pose = Pose()
        goal_pose.position.x = self.goal_x
        goal_pose.position.y = self.goal_y
        self.goal_pose_pub.publish(goal_pose)
        # self.spawn_entity()

    def task_succeed_callback(self, request, response):
        self.generate_goal_pose()
        print(f"success: generate a new goal, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response

    def task_fail_callback(self, request, response):

        self.generate_goal_pose()
        print(f"fail: reset the environment, goal pose: {self.goal_x:.2f}, {self.goal_y:.2f}")
        return response



    def generate_goal_pose(self):
        self.prev_x = self.goal_x
        self.prev_y = self.goal_y
        # tries = 0
        # --- Define static goal positions here ---
        goal_pose_list = [[1.0, 0.0], [2.0, -1.5], [0.0, -2.0], [2.0, 2.0], [0.8, 2.0],
                        [-1.9, 1.9], [-1.9,  0.2], [-1.9, -0.5], [-2.0, -2.0], [-0.5, -1.0],
                        [1.5, -1.0], [-0.5, 1.0], [-1.0, -2.0], [1.8, -0.2], [1.0, -1.9]]
        # index = random.randrange(0, len(goal_pose_list))
        self.goal_x = float(goal_pose_list[self.index_g][0])
        self.goal_y = float(goal_pose_list[self.index_g][1])
            # tries += 1
            # if tries > 100:
            #     print("ERROR: distance between goals is small!")
            #     break
        self.publish_callback()
        self.index_g += 1



def main():
    rclpy.init()
    drl_gazebo = DRLGazebo()
    rclpy.spin(drl_gazebo)

    drl_gazebo.destroy()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
