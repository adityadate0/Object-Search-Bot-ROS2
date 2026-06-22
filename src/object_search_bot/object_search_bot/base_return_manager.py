import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped, PoseWithCovarianceStamped, Twist # 👈 Added Twist
import math


"""

This node handles autonomous robot movement, detecting target and then returning to the base.

"""
class BaseReturnManager(Node):
    def __init__(self):
        super().__init__('base_return_manager')
        
        # 1. Base selection configuration
        self.declare_parameter('selected_base', 'top_left_alcove')
        user_choice = self.get_parameter('selected_base').get_parameter_value().string_value
        
        self.declare_parameter(f'{user_choice}.x', 0.0)
        self.declare_parameter(f'{user_choice}.y', 0.0)
        self.declare_parameter(f'{user_choice}.yaw', 0.0)
        
        self.base_x = self.get_parameter(f'{user_choice}.x').value
        self.base_y = self.get_parameter(f'{user_choice}.y').value
        self.base_yaw = self.get_parameter(f'{user_choice}.yaw').value
        
        self.get_logger().info(f"📍 Base Return Manager Online! Home Base: '{user_choice}'")

        # 2. Action Client for Navigation Goals
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self._goal_handle = None
        
        self.mission_triggered = False
        self.pose_injected = False  
        self.nav2_ready = False  
        self.exploration_active = False
        
        self.sweep_index = 0
        self.search_pattern = [
            {'x': 2.2, 'y': 0.8, 'yaw': 0.0},
            {'x': 1.0, 'y': -1.5, 'yaw': -1.57},
            {'x': -2.0, 'y': -0.5, 'yaw': 3.14},
            {'x': 0.0, 'y': 2.0, 'yaw': 1.57}
        ]

        # 3. 🛠️ HARD OVERRIDE SPEED PUBLISHER: Pins the wheels to absolute zero on arrival
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.target_sub = self.create_subscription(PointStamped, '/found_object_pose', self.target_detected_callback, 10)
        self.initial_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        from nav_msgs.msg import Odometry
        self.spawn_sub = self.create_subscription(Odometry, '/odom', self.spawn_monitoring_callback, 10)        

        self.nav_check_timer = self.create_timer(1.0, self.check_nav2_status)

    def check_nav2_status(self):
        if self._action_client.server_is_ready():
            self.get_logger().info("✅ NAV2 NATIVE CHANNELS ALIGNED! Autonomous search sequence activated.")
            self.nav2_ready = True
            self.nav_check_timer.cancel()
            self.boot_timer = self.create_timer(3.0, self.trigger_autonomous_search_step)

    def trigger_autonomous_search_step(self):
        if not self.nav2_ready or self.mission_triggered or self.exploration_active:
            return

        wp = self.search_pattern[self.sweep_index]
        self.get_logger().info(f"🧭 Search Loop: Advancing to Sweep Target {self.sweep_index + 1}/{len(self.search_pattern)} -> X: {wp['x']}, Y: {wp['y']}")
        
        self.exploration_active = True
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = wp['x']
        goal_msg.pose.pose.position.y = wp['y']
        goal_msg.pose.pose.orientation.z = math.sin(wp['yaw'] / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(wp['yaw'] / 2.0)

        self.send_navigation_goal(goal_msg, is_return_home=False)

    def send_navigation_goal(self, goal_msg, is_return_home=False):
        self._action_client.wait_for_server()
        if is_return_home and self._goal_handle is not None:
            self.get_logger().info("🔄 Intercept verified! Canceling active search loop tracking targets...")
            self._goal_handle.cancel_goal_async()

        send_goal_future = self._action_client.send_goal_async(goal_msg)
        
        # Pass a contextual tracking hook so we know when the home sequence completes
        if is_return_home:
            send_goal_future.add_done_callback(self.home_goal_response_callback)
        else:
            send_goal_future.add_done_callback(self.search_goal_response)

    def search_goal_response(self, future):
        self._goal_handle = future.result()
        if not self._goal_handle.accepted:
            self.exploration_active = False
            return
        self._get_result_future = self._goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.search_step_finished)

    def search_step_finished(self, future):
        if self.mission_triggered:
            return
        self.get_logger().info("🏁 Sweep target resolved cleanly. Scanning next segment...")
        self.sweep_index = (self.sweep_index + 1) % len(self.search_pattern)
        self.exploration_active = False
        self.trigger_autonomous_search_step()

    def home_goal_response_callback(self, future):
        """Monitors acceptance response for the home return sequence."""
        goal_handle = future.result()
        if not goal_handle.accepted:
            return
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.home_arrival_callback)

    def home_arrival_callback(self, future):
        """Triggers the exact millisecond the robot successfully parks back in the alcove."""
        self.get_logger().info("🛑 HOME BASE DETECTED! Initializing asynchronous safety kill sequence...")
        
        # Counter to track how many times we enforce absolute zero
        self.stop_pulses_sent = 0
        
        # 💡 ASYNCHRONOUS OVERWRITE TIMER: Fires every 50ms to completely drown out trailing buffer commands
        self.hard_stop_timer = self.create_timer(0.05, self.enforce_absolute_standstill)

    def enforce_absolute_standstill(self):
        """Repeatedly floods the command velocity channels over a 500ms window."""
        from geometry_msgs.msg import Twist
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.linear.y = 0.0
        stop_msg.linear.z = 0.0
        stop_msg.angular.x = 0.0
        stop_msg.angular.y = 0.0
        stop_msg.angular.z = 0.0
        
        self.cmd_vel_pub.publish(stop_msg)
        self.stop_pulses_sent += 1
        
        # After 10 pulses (500ms total), the motor tracks are completely dead. Cancel the timer.
        if self.stop_pulses_sent >= 10:
            self.hard_stop_timer.cancel()
            self.get_logger().info("🔒 Chassis locked at absolute zero. Search mission complete!")

    def spawn_monitoring_callback(self, msg):
        if self.pose_injected: return
        self.pose_injected = True
        self.destroy_subscription(self.spawn_sub)
        self.init_timer = self.create_timer(2.0, self.publish_forced_initial_pose)

    def publish_forced_initial_pose(self):
        self.init_timer.cancel()  
        init_pose = PoseWithCovarianceStamped()
        init_pose.header.frame_id = 'map'
        init_pose.header.stamp = self.get_clock().now().to_msg() 
        init_pose.pose.pose.position.x = self.base_x
        init_pose.pose.pose.position.y = self.base_y
        init_pose.pose.pose.orientation.z = math.sin(self.base_yaw / 2.0)
        init_pose.pose.pose.orientation.w = math.cos(self.base_yaw / 2.0)
        self.get_logger().info("🔥 Initial Pose injected to AMCL.")
        self.initial_pose_pub.publish(init_pose)    

    def target_detected_callback(self, msg):
        if self.mission_triggered: return  
        self.get_logger().info("🎯 TARGET CONFIRMED BY PERCEPTION! Aborting search patterns...")
        self.mission_triggered = True
        
        import os
        os.system("pkill -f teleop_keyboard")
        os.system("pkill -f teleop")
        
        self.return_to_base()

    def return_to_base(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = self.base_x
        goal_msg.pose.pose.position.y = self.base_y
        goal_msg.pose.pose.orientation.z = math.sin(self.base_yaw / 2.0)
        goal_msg.pose.pose.orientation.w = math.cos(self.base_yaw / 2.0)
        
        self.get_logger().info(f"🚀 Navigating home to Alcove -> X: {self.base_x}, Y: {self.base_y}")
        self.send_navigation_goal(goal_msg, is_return_home=True)

def main(args=None):
    rclpy.init(args=args)
    node = BaseReturnManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()