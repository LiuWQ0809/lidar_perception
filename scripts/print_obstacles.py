#!/usr/bin/env python3
"""
订阅融合感知障碍物话题，打印每一帧的目标ID和位置
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import MarkerArray


class ObstaclePrinter(Node):
    def __init__(self):
        super().__init__('obstacle_printer')
        
        # 订阅障碍物话题
        self.subscription = self.create_subscription(
            MarkerArray,
            '/fusion_perception/obstacles',
            self.obstacles_callback,
            10
        )
        
        self.frame_count = 0
        
        self.get_logger().info("Obstacle Printer started!")
        self.get_logger().info("Subscribing to: /fusion_perception/obstacles")
        self.get_logger().info("=" * 60)
    
    def obstacles_callback(self, msg):
        """处理接收到的障碍物消息"""
        self.frame_count += 1
        
        # 过滤出3D框标记（排除文本标记和删除标记）
        # ns="obstacles" 是3D框，ns="labels" 是文本标签
        obstacles = [marker for marker in msg.markers 
                    if marker.ns == "obstacles" and marker.action != 3]  # action=3是DELETEALL
        
        if len(obstacles) == 0:
            self.get_logger().info(f"Frame {self.frame_count}: No obstacles detected")
            return
        
        # 打印帧信息
        print(f"\n{'='*60}")
        print(f"Frame {self.frame_count}: {len(obstacles)} obstacle(s) detected")
        print(f"{'='*60}")
        
        # 按类型分组统计
        obstacle_types = {}
        
        for marker in obstacles:
            # 获取目标信息
            obj_id = marker.id
            pos_x = marker.pose.position.x
            pos_y = marker.pose.position.y
            pos_z = marker.pose.position.z
            
            # 从其他marker中查找对应的文本标签获取类别
            class_name = "person"
            for m in msg.markers:
                if m.ns == "labels" and m.id == (obj_id + 10000):
                    # 文本格式: "ID:139 person"
                    text_parts = m.text.split()
                    if len(text_parts) >= 2:
                        class_name = text_parts[1]
                    break
            
            # 统计
            if class_name not in obstacle_types:
                obstacle_types[class_name] = 0
            obstacle_types[class_name] += 1
            
            # 打印详细信息（body坐标系: X前 Y左 Z上）
            print(f"  ID: {obj_id:3d} | Class: {class_name:10s} | "
                  f"Position: ({pos_x:6.2f}, {pos_y:6.2f}, {pos_z:6.2f}) m | "
                  f"Forward(X): {pos_x:5.2f}m | "  # 前向距离（body X）
                  f"Height(Z): {pos_z:5.2f}m | "   # 高度（body Z）
                  f"Size: ({marker.scale.x:.2f}, {marker.scale.y:.2f}, {marker.scale.z:.2f}) m")
        
        # 打印统计信息
        print(f"\nSummary: ", end="")
        print(", ".join([f"{count}x {cls}" for cls, count in obstacle_types.items()]))


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = ObstaclePrinter()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nShutting down obstacle printer...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
