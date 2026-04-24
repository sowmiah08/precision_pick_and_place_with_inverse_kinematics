import rclpy
from rclpy.node import Node
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

class Table(Node):
    def __init__(self):
        super().__init__('add_table')
        self.cli = self.create_client(ApplyPlanningScene, '/apply_planning_scene')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            pass
        self.send_scene()

    def send_scene(self):
        scene = PlanningScene()
        scene.is_diff = True

        obj = CollisionObject()
        obj.header.frame_id = "base_link"
        obj.id = "table"

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.8, 0.8, 0.05]

        pose = Pose()
        pose.position.x = 0.35
        pose.position.y = 0.0
        pose.position.z = -0.03

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.ADD

        scene.world.collision_objects.append(obj)

        req = ApplyPlanningScene.Request()
        req.scene = scene
        self.cli.call_async(req)

def main():
    rclpy.init()
    node = Table()
    rclpy.spin_once(node, timeout_sec=2)
    rclpy.shutdown()

if __name__ == '__main__':
    main()