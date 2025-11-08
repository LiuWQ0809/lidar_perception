#!/usr/bin/env python3
"""
对比Python和C++版本的输出
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray
import time


class OutputComparator(Node):
    def __init__(self):
        super().__init__('output_comparator')
        
        # 订阅两个话题
        self.python_sub = self.create_subscription(
            MarkerArray,
            '/fusion_perception/obstacles',
            self.python_callback,
            10
        )
        
        self.cpp_sub = self.create_subscription(
            MarkerArray,
            '/fusion_perception/obstacles_cpp',
            self.cpp_callback,
            10
        )
        
        self.python_data = None
        self.cpp_data = None
        self.python_time = None
        self.cpp_time = None
        
        # 创建定时器定期对比
        self.timer = self.create_timer(1.0, self.compare_outputs)
        
        self.get_logger().info("Output Comparator started!")
        self.get_logger().info("Python topic: /fusion_perception/obstacles")
        self.get_logger().info("C++ topic: /fusion_perception/obstacles_cpp")
        self.get_logger().info("=" * 80)
    
    def python_callback(self, msg):
        """处理Python版本的消息"""
        self.python_data = msg
        self.python_time = time.time()
    
    def cpp_callback(self, msg):
        """处理C++版本的消息"""
        self.cpp_data = msg
        self.cpp_time = time.time()
    
    def compare_outputs(self):
        """对比两个版本的输出"""
        if self.python_data is None or self.cpp_data is None:
            self.get_logger().warn("Waiting for data from both topics...")
            return
        
        # 检查数据是否新鲜（1秒内）
        current_time = time.time()
        if current_time - self.python_time > 1.0:
            self.get_logger().warn("Python data is stale")
            return
        if current_time - self.cpp_time > 1.0:
            self.get_logger().warn("C++ data is stale")
            return
        
        # 提取障碍物标记
        python_obstacles = [m for m in self.python_data.markers 
                           if m.ns == "obstacles" and m.action != 3]
        cpp_obstacles = [m for m in self.cpp_data.markers 
                        if m.ns == "obstacles" and m.action != 3]
        
        print(f"\n{'='*80}")
        print(f"Comparison at {time.strftime('%H:%M:%S')}")
        print(f"{'='*80}")
        
        print(f"\nPython: {len(python_obstacles)} obstacles")
        print(f"C++:    {len(cpp_obstacles)} obstacles")
        
        if len(python_obstacles) == 0 and len(cpp_obstacles) == 0:
            print("\nBoth versions: No obstacles detected")
            return
        
        # 对比第一个障碍物（如果都有）
        if len(python_obstacles) > 0 and len(cpp_obstacles) > 0:
            py_obs = python_obstacles[0]
            cpp_obs = cpp_obstacles[0]
            
            print(f"\n{'First Obstacle Comparison':-^80}")
            print(f"\n{'Attribute':<20} | {'Python':<25} | {'C++':<25}")
            print(f"{'-'*20}-+-{'-'*25}-+-{'-'*25}")
            
            # 位置对比
            print(f"{'Position (x,y,z)':<20} | "
                  f"({py_obs.pose.position.x:6.3f}, {py_obs.pose.position.y:6.3f}, {py_obs.pose.position.z:6.3f}) | "
                  f"({cpp_obs.pose.position.x:6.3f}, {cpp_obs.pose.position.y:6.3f}, {cpp_obs.pose.position.z:6.3f})")
            
            # 尺寸对比
            print(f"{'Size (L,W,H)':<20} | "
                  f"({py_obs.scale.x:5.3f}, {py_obs.scale.y:5.3f}, {py_obs.scale.z:5.3f}) | "
                  f"({cpp_obs.scale.x:5.3f}, {cpp_obs.scale.y:5.3f}, {cpp_obs.scale.z:5.3f})")
            
            # 尺寸差异
            diff_x = abs(py_obs.scale.x - cpp_obs.scale.x)
            diff_y = abs(py_obs.scale.y - cpp_obs.scale.y)
            diff_z = abs(py_obs.scale.z - cpp_obs.scale.z)
            print(f"{'Size Difference':<20} | "
                  f"({diff_x:5.3f}, {diff_y:5.3f}, {diff_z:5.3f})")
            
            # 位置差异
            pos_diff_x = abs(py_obs.pose.position.x - cpp_obs.pose.position.x)
            pos_diff_y = abs(py_obs.pose.position.y - cpp_obs.pose.position.y)
            pos_diff_z = abs(py_obs.pose.position.z - cpp_obs.pose.position.z)
            print(f"{'Position Difference':<20} | "
                  f"({pos_diff_x:6.3f}, {pos_diff_y:6.3f}, {pos_diff_z:6.3f})")
            
            # Frame ID对比
            print(f"{'Frame ID':<20} | "
                  f"{py_obs.header.frame_id:<25} | "
                  f"{cpp_obs.header.frame_id:<25}")
            
            # 判断是否一致
            threshold = 0.05  # 5cm阈值
            if diff_x < threshold and diff_y < threshold and diff_z < threshold:
                print(f"\n✓ Sizes are CONSISTENT (within {threshold}m)")
            else:
                print(f"\n✗ Sizes are DIFFERENT (exceeds {threshold}m)")
            
            if pos_diff_x < threshold and pos_diff_y < threshold and pos_diff_z < threshold:
                print(f"✓ Positions are CONSISTENT (within {threshold}m)")
            else:
                print(f"✗ Positions are DIFFERENT (exceeds {threshold}m)")


def main(args=None):
    rclpy.init(args=args)
    
    comparator = OutputComparator()
    
    try:
        rclpy.spin(comparator)
    except KeyboardInterrupt:
        pass
    finally:
        comparator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
