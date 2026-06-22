import rclpy
from rclpy.node import Node
import random
from geometry_msgs.msg import PoseStamped, PointStamped
from nav2_simple_commander.robot_navigator import BasicNavigator

class MissionControl(Node):
    def __init__(self):
        super().__init__('mission_control')
        self.navigator = BasicNavigator()
        
        self.start_pose = PoseStamped()
        self.start_pose.header.frame_id = 'map'
        self.start_pose.pose.position.x = 0.0
        self.start_pose.pose.position.y = 0.0
        self.start_pose.pose.orientation.w = 1.0

        self.navigator.setInitialPose(self.start_pose)
        # Because we are using AMCL now, we wait for amcl!
        self.navigator.waitUntilNav2Active(localizer='amcl')

        self.target_found = False
        self.target_location = None
        self.state = "SEARCHING"

        self.target_sub = self.create_subscription(PointStamped, '/found_object_pose', self.target_callback, 10)
        self.timer = self.create_timer(1.0, self.patrol_logic)
        
        self.get_logger().info("Mission Control: Autonomous Search in Known Map Engaged.")

    def create_pose(self, x, y):
        pose = PoseStamped()
        pose.header.frame_id = 'map'
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.orientation.w = 1.0
        return pose

    def target_callback(self, msg):
        if not self.target_found:
            self.target_found = True
            self.target_location = msg.point
            self.get_logger().info(f"!!! TARGET DETECTED at {msg.point.x:.2f}, {msg.point.y:.2f} !!!")
            
            self.navigator.cancelTask()
            self.state = "RETURNING"

    def patrol_logic(self):
        if self.state == "SEARCHING":
            # If the robot is idle (finished its last random point), give it a new one
            if self.navigator.isTaskComplete():
                # Random coordinates inside the 15x5 arena (leaving a 1m buffer near walls)
                rand_x = random.uniform(-4.0, 4.0) 
                rand_y = random.uniform(-4.0, 4.0)
                
                self.get_logger().info(f"Exploring new area: Heading to X: {rand_x:.2f}, Y: {rand_y:.2f}")
                self.navigator.goToPose(self.create_pose(rand_x, rand_y))

        elif self.state == "RETURNING":
            self.get_logger().info("Target logged. Returning to start point (0,0)...")
            self.navigator.goToPose(self.start_pose)
            self.state = "DONE"

def main(args=None):
    rclpy.init(args=args)
    node = MissionControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()