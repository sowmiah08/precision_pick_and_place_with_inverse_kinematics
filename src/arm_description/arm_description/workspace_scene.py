#!/usr/bin/env python3

"""
workspace_scene.py

Dynamic MoveIt Planning Scene manager.

Static objects (already in Xacro):
- table
- robot
- camera
- drop box

Dynamic objects (this script):
- red cube
- blue cube
- remove cube after pick
- attach/detach ready structure

Frame used:
- base_link
"""

import time
import rclpy
from rclpy.node import Node

from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose
from rclpy.qos import QoSProfile


class WorkspaceScene(Node):

    def __init__(self):
        super().__init__("workspace_scene")

        self.pub = self.create_publisher(
            PlanningScene,
            "/planning_scene",
            QoSProfile(depth=10)
        )

        self.get_logger().info("Workspace Scene Started")

        time.sleep(2.0)

        # Spawn first cube
        self.spawn_red_cube()

    # ---------------------------------------------------
    # Generic Box Creator
    # ---------------------------------------------------
    def create_cube(self, name, x, y, z, size=0.04):

        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = "base_link"

        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.BOX
        primitive.dimensions = [size, size, size]

        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z
        pose.orientation.w = 1.0

        obj.primitives.append(primitive)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        return obj

    # ---------------------------------------------------
    # Publish Planning Scene
    # ---------------------------------------------------
    def publish_object(self, collision_obj):

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(collision_obj)

        self.pub.publish(scene)
        time.sleep(1.0)

    # ---------------------------------------------------
    # Remove Object
    # ---------------------------------------------------
    def remove_object(self, name):

        obj = CollisionObject()
        obj.id = name
        obj.header.frame_id = "base_link"
        obj.operation = CollisionObject.REMOVE

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(obj)

        self.pub.publish(scene)
        time.sleep(1.0)

    # ---------------------------------------------------
    # RED CUBE
    # ---------------------------------------------------
    def spawn_red_cube(self):

        self.get_logger().info("Spawning RED cube")

        red_cube = self.create_cube(
            name="red_cube",
            x=0.22,
            y=-0.10,
            z=0.02
        )

        self.publish_object(red_cube)

    # ---------------------------------------------------
    # BLUE CUBE
    # ---------------------------------------------------
    def spawn_blue_cube(self):

        self.get_logger().info("Spawning BLUE cube")

        blue_cube = self.create_cube(
            name="blue_cube",
            x=0.18,
            y=0.10,
            z=0.02
        )

        self.publish_object(blue_cube)

    # ---------------------------------------------------
    # Called after red cube picked
    # ---------------------------------------------------
    def red_done(self):

        self.remove_object("red_cube")
        self.spawn_blue_cube()

    # ---------------------------------------------------
    # Called after blue cube picked
    # ---------------------------------------------------
    def blue_done(self):

        self.remove_object("blue_cube")
        self.get_logger().info("All cubes completed")



def main():

    rclpy.init()
    node = WorkspaceScene()

    rclpy.spin(node)

    rclpy.shutdown()


if __name__ == "__main__":
    main()