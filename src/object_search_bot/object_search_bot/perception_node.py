import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PointStamped
from cv_bridge import CvBridge
from ultralytics import YOLO
import math
import cv2
import numpy as np
import os
import yaml # Ensure PyYAML is available
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point
from rclpy.qos import QoSProfile, ReliabilityPolicy

"""
This node subscribes to the RGB and Depth image topics, runs YOLO object detection to find the target object, and then projects the detected object's pixel coordinates into the global map frame. It also exports a new map image with the detected object's location marked on it. The node uses TF2 to handle coordinate transformations and dynamically loads map metadata from the SLAM-generated YAML file to ensure accurate projections.
"""

class PerceptionNode(Node):
    def __init__(self):
        super().__init__('perception_node')
        self.bridge = CvBridge()
        
        # Load YOLO model
        self.model = YOLO('yolov10n.pt') 
        self.target_object = 'laptop' 

        # Map metadata parameters (Update these to match your saved map's .yaml file exactly)
        # You can find these in your saved map.yaml
        self.map_resolution = 0.05  # meters per pixel
        self.map_origin_x = -10.0   # X coordinate of bottom-left pixel
        self.map_origin_y = -10.0   # Y coordinate of bottom-left pixel
        
        # Paths for input and output map assets
        # Adjust these paths to match where your actual map file is stored
        #self.input_map_path = '/root/ros2_ws/src/object_search_bot/maps/office_teardown.pgm'
        #self.output_png_path = '/root/ros2_ws/src/object_search_bot/maps/detected_object_map.png'

        # Base paths for your map assets
        self.map_dir = '/root/ros2_ws/src/object_search_bot/maps/'
        # Change 'my_office_map' to your exact map file prefix name!
        self.map_name = 'office_teardown' 
        
        self.input_map_path = os.path.join(self.map_dir, f"{self.map_name}.pgm")
        self.yaml_path = os.path.join(self.map_dir, f"{self.map_name}.yaml")
        self.output_png_path = os.path.join(self.map_dir, 'detected_object_map.png')

        # Dynamically load metadata from the saved YAML file
        self.load_map_metadata()
        
        
        # Flag to ensure we only export the final PNG map once per run
        self.map_exported = False

        # Initialize Transform listeners
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.target_pub = self.create_publisher(PointStamped, '/found_object_pose', 10)

        # Camera Intrinsics
        self.fx, self.fy = 381.3, 381.3
        self.cx, self.cy = 320.5, 240.5

        self.latest_depth_matrix = None
        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.rgb_sub = self.create_subscription(Image, '/camera/image_raw', self.image_callback, sensor_qos)
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, sensor_qos)
        
        self.get_logger().info(f"BEV Map Exporter Node Active! Target: '{self.target_object}'")

    
    def load_map_metadata(self):
        """Automatically parses resolution and origins from the SLAM map yaml configuration."""
        if not os.path.exists(self.yaml_path):
            self.get_logger().error(f"Map YAML configuration not found at {self.yaml_path}! Using fallbacks.")
            self.map_resolution = 0.05
            self.map_origin_x = -10.0
            self.map_origin_y = -10.0
            return

        with open(self.yaml_path, 'r') as f:
            map_data = yaml.safe_load(f)
            
        self.map_resolution = float(map_data['resolution'])
        # The YAML origin tag contains a list: [X, Y, Yaw]
        self.map_origin_x = float(map_data['origin'][0])
        self.map_origin_y = float(map_data['origin'][1])
        
        self.get_logger().info(f"Loaded Map Meta -> Res: {self.map_resolution}, OriginX: {self.map_origin_x}, OriginY: {self.map_origin_y}")
    
    
    def depth_callback(self, msg):
        self.latest_depth_matrix = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

    def export_bev_map(self, x_map, y_map):
        """Loads the pre-saved map, plots the object coordinate, and saves a PNG."""
        if not os.path.exists(self.input_map_path):
            self.get_logger().error(f"Base map not found at {self.input_map_path}. Cannot export image.")
            return

        # Load the original gray SLAM occupancy grid map
        map_img = cv2.imread(self.input_map_path, cv2.IMREAD_COLOR)
        height, width, _ = map_img.shape

        # Calculate pixel coordinates using the map projection math
        pixel_col = int((x_map - self.map_origin_x) / self.map_resolution)
        pixel_row = int(height - ((y_map - self.map_origin_y) / self.map_resolution))

        # Boundary check to verify the coordinate sits inside the image array bounds
        if 0 <= pixel_col < width and 0 <= pixel_row < height:
            # Draw a solid red circle marker (BGR color space: Red = (0, 0, 255))
            cv2.circle(map_img, (pixel_col, pixel_row), radius=3, color=(0, 0, 255), thickness=1)
            # Add a text label next to the marker
            cv2.putText(map_img, "TARGET", (pixel_col + 12, pixel_row + 5), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)

            # Save out to a standard PNG file asset
            cv2.imwrite(self.output_png_path, map_img)
            self.get_logger().info(f"⭐⭐ BEV MISSION MAP EXPORTED SUCCESSFULLY TO: {self.output_png_path} ⭐⭐")
            self.map_exported = True
        else:
            self.get_logger().error(f"Calculated object pixel ({pixel_col}, {pixel_row}) sits outside map dimensions!")

    def image_callback(self, msg):
        if self.latest_depth_matrix is None or self.map_exported:
            return

        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        results = self.model(cv_image, verbose=False, conf=0.05, device='cpu')

        for r in results:
            boxes = r.boxes
            for box in boxes:
                if self.model.names[int(box.cls[0])] == self.target_object:
                    x1, y1, x2, y2 = box.xyxy[0]
                    u = int((x1.item() + x2.item()) / 2)
                    v = int((y1 + y2) / 2)

                    try:
                        z_depth = float(self.latest_depth_matrix[v, u])
                        if math.isnan(z_depth) or math.isinf(z_depth) or z_depth <= 0.1:
                            continue
                                
                    except IndexError:
                        continue

                    # Project 2D center pixels into explicit 3D camera metrics
                    x_camera = (u - self.cx) * z_depth / self.fx
                    y_camera = (v - self.cy) * z_depth / self.fy
                    
                    point_camera = PointStamped()
                    point_camera.header.frame_id = 'realsense_depth_frame' 
                    point_camera.header.stamp = msg.header.stamp
                    point_camera.point.x = x_camera
                    point_camera.point.y = y_camera
                    point_camera.point.z = z_depth

                    try:
                        # Direct Transform lookup to get the coordinate inside the global map frame
                        transform = self.tf_buffer.lookup_transform(
                            'map', 'realsense_depth_frame', rclpy.time.Time(), rclpy.duration.Duration(seconds=0.5))
                        
                        point_map = do_transform_point(point_camera, transform)
                        self.target_pub.publish(point_map)
                        
                        # Trigger the image processing exporter sequence
                        self.export_bev_map(point_map.point.x, point_map.point.y)
                        
                    except Exception as e:
                        self.get_logger().warn(f"Waiting for map frame transform linkage... {e}", throttle_duration_sec=5.0)

def main(args=None):
    rclpy.init(args=args)
    node = PerceptionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()