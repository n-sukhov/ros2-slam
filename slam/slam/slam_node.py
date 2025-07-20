#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
from tf_transformations import quaternion_from_euler
from slam.icp import ICP
import collections

from time import time

class SLAM(Node):
    def __init__(self):
        super().__init__('slam')
    
        self.map_resolution = 0.05
        self.default_width = 10
        self.default_height = 10
        
        self.wall_increment = 50
        self.wall_decrement = 10

        self.map_data = np.zeros((self.default_height, self.default_width), dtype=np.int8)
        self.map_data.fill(-1)
        
        self.map_origin_x = -self.default_width * self.map_resolution / 2
        self.map_origin_y = -self.default_height * self.map_resolution / 2
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 20)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 20)
        
        self.timer = self.create_timer(0.1, self.publish_map)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.icp = ICP(20, 0.0001)
        self.scan_history = collections.deque(maxlen=4)
        self.critical_mean_error = 0.1
        
        self.get_logger().info("SLAM node initialized")

    def scan_callback(self, msg):
        ranges = np.array(msg.ranges)
        valid_indices = (ranges >= msg.range_min) & (ranges <= msg.range_max)
        angles = msg.angle_min + np.arange(len(ranges)) * msg.angle_increment
        
        current_scan = np.vstack((
            ranges[valid_indices] * np.cos(angles[valid_indices]),
            ranges[valid_indices] * np.sin(angles[valid_indices])
        )).T
        
        transformations = []
        
        for saved_scan, saved_pose in self.scan_history:
            try:
                T, mean_error = self.icp.get_T(saved_scan, current_scan)
                T_inv = np.linalg.inv(T)

                dx = T_inv[0, 2]
                dy = T_inv[1, 2]
                dtheta = np.arctan2(T_inv[1, 0], T_inv[0, 0])
                
                global_dx = dx * np.cos(saved_pose[2]) - dy * np.sin(saved_pose[2])
                global_dy = dx * np.sin(saved_pose[2]) + dy * np.cos(saved_pose[2])
                self.get_logger().info(f"mean error = {mean_error}")
                if mean_error < self.critical_mean_error:
                    transformations.append((
                        saved_pose[0] + global_dx,
                        saved_pose[1] + global_dy,
                        saved_pose[2] + dtheta
                    ))
                
            except Exception as e:
                self.get_logger().warn(f"ICP failed: {str(e)}")
        
        if transformations:
            xs, ys, thetas = zip(*transformations)
            
            self.robot_x = np.median(xs)
            self.robot_y = np.median(ys)
            self.robot_theta = np.arctan2(np.median(np.sin(thetas)), np.median(np.cos(thetas)))
        
        self.scan_history.append((current_scan, (self.robot_x, self.robot_y, self.robot_theta)))

        angle_min = msg.angle_min
        angle_increment = msg.angle_increment

        for i, range_val in enumerate(msg.ranges):
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
                
            angle = angle_min + i * angle_increment + self.robot_theta
            obstacle_x = self.robot_x + range_val * np.cos(angle)
            obstacle_y = self.robot_y + range_val * np.sin(angle)
            
            self.expand_map(obstacle_x, obstacle_y)
            self.mark_free_space(self.robot_x, self.robot_y, obstacle_x, obstacle_y)
            self.mark_obstacle(obstacle_x, obstacle_y)

        self.publish_tf()

    def mark_obstacle(self, x, y):
        map_x, map_y = self.world_to_map(x, y)
        if self.map_data[map_y, map_x] < 100:
                self.map_data[map_y, map_x] = min(self.map_data[map_y, map_x] + self.wall_increment, 100)

    def expand_map(self, world_x, world_y, expand_size=50):
        map_x, map_y = self.world_to_map(world_x, world_y)
        h, w = self.map_data.shape
        
        left = max(0, -map_x)
        right = max(0, map_x - (w - 1))
        top = max(0, -map_y)
        bottom = max(0, map_y - (h - 1))
        
        if left > 0 or right > 0 or top > 0 or bottom > 0:
            new_h = h + top + bottom
            new_w = w + left + right
            new_map = np.full((new_h, new_w), -1, dtype=np.int8)
            
            new_map[top:top+h, left:left+w] = self.map_data
            
            self.map_data = new_map
            self.map_origin_x -= left * self.map_resolution
            self.map_origin_y -= top * self.map_resolution

    def mark_free_space(self, x0, y0, x1, y1):
        x0_map, y0_map = self.world_to_map(x0, y0)
        x1_map, y1_map = self.world_to_map(x1, y1)
        
        dx = abs(x1_map - x0_map)
        dy = abs(y1_map - y0_map)
        x = x0_map
        y = y0_map
        sx = -1 if x0_map > x1_map else 1
        sy = -1 if y0_map > y1_map else 1

        xs = []
        ys = []
        if dx > dy:
            err = dx / 2.0
            while x != x1_map:
                xs.append(x)
                ys.append(y)
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1_map:
                xs.append(x)
                ys.append(y)
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy

        self.map_data[ys, xs] -= self.wall_decrement
        self.map_data[ys, xs] = np.maximum(self.map_data[ys, xs], 0)

    def world_to_map(self, world_x, world_y):
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y

    def publish_map(self):
        map_msg = OccupancyGrid()
        map_msg.header.stamp = self.get_clock().now().to_msg()
        map_msg.header.frame_id = 'map'
        
        map_msg.info.resolution = self.map_resolution
        map_msg.info.width = self.map_data.shape[1]
        map_msg.info.height = self.map_data.shape[0]
        map_msg.info.origin.position.x = self.map_origin_x
        map_msg.info.origin.position.y = self.map_origin_y
        map_msg.info.origin.position.z = 0.0
        map_msg.info.origin.orientation.w = 1.0
    
        map_msg.data = self.map_data.flatten().astype(np.int8).tolist()
        
        self.map_pub.publish(map_msg)

    def publish_tf(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'map'
        t.child_frame_id = 'base_link'

        t.transform.translation.x = self.robot_x
        t.transform.translation.y = self.robot_y
        t.transform.translation.z = 0.0
        
        q = quaternion_from_euler(0, 0, self.robot_theta)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    slam_node = SLAM()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()