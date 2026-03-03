#!/usr/bin/env python3
"""
Detection Converter Node
Subscribe to detection messages in JSON format, convert them to vision_interface/Detections messages and publish.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import String
from vision_interface.msg import Detections, DetectedObject, LineSegments
import json


class DetectionConverter(Node):
    def __init__(self):
        super().__init__('detection_converter')
        
        self.declare_parameter('input_topic', '/camera/robot0_rgbd_camera/detections')
        self.declare_parameter('detections_topic', '/booster_soccer/detection')
        self.declare_parameter('line_segments_topic', '/booster_soccer/line_segments')
        
        input_topic = self.get_parameter('input_topic').value
        detections_topic = self.get_parameter('detections_topic').value
        line_segments_topic = self.get_parameter('line_segments_topic').value
        
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.last_timestamp = None
        self.msg_count = 0
        self.pub_count = 0
        
        self.subscription = self.create_subscription(
            String,
            input_topic,
            self.detection_callback,
            sensor_qos
        )
        
        self.publisher = self.create_publisher(
            Detections,
            detections_topic,
            sensor_qos
        )
        
        self.line_segments_publisher = self.create_publisher(
            LineSegments,
            line_segments_topic,
            sensor_qos
        )
        
        self.get_logger().info(f'Detection converter started')
        self.get_logger().info(f'Subscribing to: {input_topic}')
        self.get_logger().info(f'Publishing detections to: {detections_topic}')
        self.get_logger().info(f'Publishing line segments to: {line_segments_topic}')
    
    def detection_callback(self, msg):
        """Process received JSON format detection messages"""
        try:
            # Parse JSON data
            data = json.loads(msg.data)
            
            self.msg_count += 1
            
            # Message deduplication - check if the timestamp is duplicate
            timestamp = data.get('timestamp', 0.0)
            if timestamp == self.last_timestamp and timestamp > 0:
                self.get_logger().warn(f'Skipping duplicate message with timestamp {timestamp}')
                return
            
            self.last_timestamp = timestamp
            
            # Create Detections message
            detections_msg = Detections()
            
            # Set timestamp - use timestamp from JSON
            timestamp = data.get('timestamp', 0.0)
            if timestamp > 0:
                # Convert float timestamp to ROS2 time
                sec = int(timestamp)
                nanosec = int((timestamp - sec) * 1e9)
                detections_msg.header.stamp.sec = sec
                detections_msg.header.stamp.nanosec = nanosec
            else:
                # If no valid timestamp, use current time
                detections_msg.header.stamp = self.get_clock().now().to_msg()
            
            detections_msg.header.frame_id = f"camera_{data.get('camera_id', 0)}"
            
            # Initialize required array fields (to prevent segmentation faults)
            detections_msg.radar_x = []
            detections_msg.radar_y = []
            # corner_pos needs to contain coordinates for 5 points (x1,y1,x2,y2,x3,y3,x4,y4,x5,y5), total 10 elements
            detections_msg.corner_pos = [0.0] * 10
            
            # Convert each detected object
            for det in data.get('detections', []):
                detected_obj = DetectedObject()
                
                # Set label (type)
                detected_obj.label = det.get('type', 'Unknown')
                
                # Set confidence (default to 1.0 since not present in JSON)
                detected_obj.confidence = 99.0
                
                # Set bbox
                bbox = det.get('bbox', None)
                if bbox and len(bbox) == 4:
                    detected_obj.xmin = int(bbox[0])
                    detected_obj.ymin = int(bbox[1])
                    detected_obj.xmax = int(bbox[2])
                    detected_obj.ymax = int(bbox[3])
                else:
                    detected_obj.xmin = 0
                    detected_obj.ymin = 0
                    detected_obj.xmax = 0
                    detected_obj.ymax = 0
                
                # Set position
                pos = det.get('pos', [0.0, 0.0, 0.0])
                if isinstance(pos, list):
                    detected_obj.position = [float(p) for p in pos]
                else:
                    detected_obj.position = [0.0, 0.0, 0.0]
                
                # Set projected position (use pos as position_projection)
                detected_obj.position_projection = detected_obj.position
                
                # Initialize target_uv (precise pixel positions of ground markers)
                detected_obj.target_uv = []
                
                # Set camera coordinate position
                camera_pos = det.get('camera_pos', [0.0, 0.0, 0.0])
                if isinstance(camera_pos, list):
                    detected_obj.position_cam = [float(p) for p in camera_pos]
                else:
                    detected_obj.position_cam = [0.0, 0.0, 0.0]
                
                # Set radius (can be stored in color field or used as an extension)
                radius = det.get('radius', 0.0)
                detected_obj.color = f"radius_{radius}"  # Temporarily store radius information
                
                # Set confidence
                detected_obj.position_confidence = 1  # Default high confidence
                
                # Add to detection list
                detections_msg.detected_objects.append(detected_obj)
            
            # Publish message
            self.publisher.publish(detections_msg)
            self.pub_count += 1
            
            # Process and publish line segment data
            line_coordinates = data.get('line_coordinates', [])
            line_coordinates_uv = data.get('line_coordinates_uv', [])
            
            if line_coordinates or line_coordinates_uv:
                line_segments_msg = LineSegments()
                
                # Set timestamp
                line_segments_msg.header.stamp = detections_msg.header.stamp
                line_segments_msg.header.frame_id = detections_msg.header.frame_id
                
                # Convert to float32 arrays
                if isinstance(line_coordinates, list):
                    line_segments_msg.coordinates = [float(x) for x in line_coordinates]
                else:
                    line_segments_msg.coordinates = []
                
                if isinstance(line_coordinates_uv, list):
                    line_segments_msg.coordinates_uv = [float(x) for x in line_coordinates_uv]
                else:
                    line_segments_msg.coordinates_uv = []
                
                # Publish line segments message
                self.line_segments_publisher.publish(line_segments_msg)
            
            # Print statistics every 30 messages
            if self.pub_count % 30 == 0:
                self.get_logger().info(
                    f'Stats: received={self.msg_count}, published={self.pub_count}, '
                    f'detections={len(detections_msg.detected_objects)}, camera={data.get("camera_id", 0)}'
                )
            
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Failed to parse JSON: {e}')
            self.get_logger().error(f'Message data: {msg.data[:200]}...')  # Print first 200 characters
        except Exception as e:
            self.get_logger().error(f'Error in detection callback: {e}')
            import traceback
            self.get_logger().error(f'Traceback: {traceback.format_exc()}')


def main(args=None):
    rclpy.init(args=args)
    node = DetectionConverter()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
