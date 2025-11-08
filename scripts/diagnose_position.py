#!/usr/bin/env python3
"""
诊断X轴位置接近0的问题
"""
import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import numpy as np

class DiagnosticNode(Node):
    def __init__(self):
        super().__init__('diagnostic_node')
        self.subscription = self.create_subscription(
            MarkerArray,
            '/fusion_perception/obstacles',
            self.callback,
            10)
        self.frame_count = 0
        
    def callback(self, msg):
        self.frame_count += 1
        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Frame {self.frame_count}: Received {len(msg.markers)} markers")
        
        for i, marker in enumerate(msg.markers):
            pos = marker.pose.position
            scale = marker.scale
            
            self.get_logger().info(f"\nMarker {i} (ID={marker.id}):")
            self.get_logger().info(f"  Frame: {marker.header.frame_id}")
            self.get_logger().info(f"  Position: X={pos.x:.4f}, Y={pos.y:.4f}, Z={pos.z:.4f}")
            self.get_logger().info(f"  Scale:    L={scale.x:.4f}, W={scale.y:.4f}, H={scale.z:.4f}")
            
            # 计算距离
            distance = np.sqrt(pos.x**2 + pos.y**2 + pos.z**2)
            self.get_logger().info(f"  Distance: {distance:.4f}m")
            
            # 分析X轴位置
            if abs(pos.x) < 0.1:
                self.get_logger().warn(f"  ⚠️  X轴位置非常小 ({pos.x:.6f})")
                self.get_logger().warn(f"      这意味着目标在相机光轴正前方（左右偏移很小）")
            
            # 分析坐标系
            if marker.header.frame_id == "center_camera":
                self.get_logger().info(f"  📍 Camera坐标系:")
                self.get_logger().info(f"     X={pos.x:.3f} (左右，左+右-)")
                self.get_logger().info(f"     Y={pos.y:.3f} (上下，上+下-)")
                self.get_logger().info(f"     Z={pos.z:.3f} (深度，前+)")
            elif "body" in marker.header.frame_id.lower():
                self.get_logger().info(f"  🚗 Body坐标系:")
                self.get_logger().info(f"     X={pos.x:.3f} (前后，前+后-)")
                self.get_logger().info(f"     Y={pos.y:.3f} (左右，左+右-)")
                self.get_logger().info(f"     Z={pos.z:.3f} (上下，上+下-)")

def main():
    rclpy.init()
    node = DiagnosticNode()
    
    print("\n" + "="*60)
    print("X轴位置诊断工具")
    print("="*60)
    print("监听话题: /fusion_perception/obstacles")
    print("Ctrl+C 退出")
    print("="*60 + "\n")
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
