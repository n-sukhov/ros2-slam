#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
import numpy as np
import math
from collections import deque

class SLAM(Node):
    def __init__(self):
        super().__init__('slam')
    
        self.map_resolution = 0.05
        self.default_width = 10
        self.default_height = 10
        
        self.map_data = np.zeros((self.default_height, self.default_width), dtype=np.int8)
        self.map_data.fill(-1)
        
        self.map_origin_x = -self.default_width * self.map_resolution / 2
        self.map_origin_y = -self.default_height * self.map_resolution / 2
        
        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_theta = 0.0
        
        self.position_history = deque(maxlen=1000)
        
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.odom_sub = self.create_subscription(Odometry, '/diff_drive_base_controller/odom', self.odom_callback, 10)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', 10)
        
        self.timer = self.create_timer(0.1, self.publish_map)
        
        self.tf_broadcaster = TransformBroadcaster(self)
        
        self.last_odom_pos = (0.0, 0.0)
        self.last_laser_ranges = None
        self.stuck_counter = 0
        self.max_stuck_count = 1
        self.is_stuck = False

        self.freeze_odometry = False
        self.last_valid_pose = (0.0, 0.0, 0.0)  

        self.get_logger().info("SLAM node initialized")

    def check_robot_stuck(self, current_ranges):
        if self.last_laser_ranges is None:
            self.last_laser_ranges = np.array(current_ranges)
            return False

        current_ranges = np.array(current_ranges)
        last_ranges = np.array(self.last_laser_ranges)

        valid_mask = ~np.isinf(current_ranges) & ~np.isnan(current_ranges) & \
                    (current_ranges >= 0.1) & (current_ranges <= 3.0)
        if not np.any(valid_mask):
            return False

        current_filtered = current_ranges[valid_mask]
        last_filtered = last_ranges[valid_mask]

        abs_diff = np.abs(current_filtered - last_filtered)
        noise_threshold = 0.05
        significant_changes = np.sum(abs_diff > noise_threshold)

        changerate = significant_changes / len(current_filtered)
        movement_threshold = 0.2

        dx = self.robot_x - self.last_odom_pos[0]
        dy = self.robot_y - self.last_odom_pos[1]
        odom_moved = (dx**2 + dy**2) > 0.01

        if odom_moved and changerate < movement_threshold:
            self.stuck_counter += 1
            if self.stuck_counter >= self.max_stuck_count:
                self.freeze_odometry = True
                return True
        else:
            self.stuck_counter = max(0, self.stuck_counter - 1)
            if self.freeze_odometry and self.stuck_counter == 0:
                self.freeze_odometry = False  

        self.last_odom_pos = (self.robot_x, self.robot_y)
        self.last_laser_ranges = current_ranges
        return False

    def odom_callback(self, msg):
        q = msg.pose.pose.orientation
        if self.freeze_odometry:
            self.robot_x, self.robot_y, self.robot_theta = self.last_valid_pose
        else:
            self.robot_x = msg.pose.pose.position.x
            self.robot_y = msg.pose.pose.position.y
            self.robot_theta = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
            self.last_valid_pose = (self.robot_x, self.robot_y, self.robot_theta)
            self.position_history.append((self.robot_x, self.robot_y))
        
        self.publish_tf()

    def scan_callback(self, msg):
        if self.check_robot_stuck(np.array(msg.ranges)):
            self.is_stuck = True
            return

        self.is_stuck = False 
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        
        for i, range_val in enumerate(msg.ranges):
            if range_val < msg.range_min or range_val > msg.range_max:
                continue
                
            angle = angle_min + i * angle_increment + self.robot_theta
            obstacle_x = self.robot_x + range_val * math.cos(angle)
            obstacle_y = self.robot_y + range_val * math.sin(angle)
            
            self.check_and_expand_map(obstacle_x, obstacle_y)
            self.mark_free_space(self.robot_x, self.robot_y, obstacle_x, obstacle_y)
            
            map_x, map_y = self.world_to_map(obstacle_x, obstacle_y)
            if self.is_in_map(map_x, map_y):
                self.map_data[map_y, map_x] = 100

    def check_and_expand_map(self, world_x, world_y):
        map_x, map_y = self.world_to_map(world_x, world_y)
        expanded = False
        
        while map_x < 0:
            self.expand_map('left')
            map_x, map_y = self.world_to_map(world_x, world_y)
            expanded = True
            
        while map_x >= self.map_data.shape[1]:
            self.expand_map('right')
            map_x, map_y = self.world_to_map(world_x, world_y)
            expanded = True
            
        while map_y < 0:
            self.expand_map('bottom')
            map_x, map_y = self.world_to_map(world_x, world_y)
            expanded = True
            
        while map_y >= self.map_data.shape[0]:
            self.expand_map('top')
            map_x, map_y = self.world_to_map(world_x, world_y)
            expanded = True
            
        return expanded

    def expand_map(self, direction, expand_size=50):
        old_height, old_width = self.map_data.shape
        
        if direction == 'left':
            new_cols = np.zeros((old_height, expand_size), dtype=np.int8)
            new_cols.fill(-1)
            self.map_data = np.hstack((new_cols, self.map_data))
            self.map_origin_x -= expand_size * self.map_resolution
            
        elif direction == 'right':
            new_cols = np.zeros((old_height, expand_size), dtype=np.int8)
            new_cols.fill(-1)
            self.map_data = np.hstack((self.map_data, new_cols))
            
        elif direction == 'top':
            new_rows = np.zeros((expand_size, old_width), dtype=np.int8)
            new_rows.fill(-1)
            self.map_data = np.vstack((self.map_data, new_rows))
            
        elif direction == 'bottom':
            new_rows = np.zeros((expand_size, old_width), dtype=np.int8)
            new_rows.fill(-1)
            self.map_data = np.vstack((new_rows, self.map_data))
            self.map_origin_y -= expand_size * self.map_resolution
            

    def mark_free_space(self, x0, y0, x1, y1):
        x0_map, y0_map = self.world_to_map(x0, y0)
        x1_map, y1_map = self.world_to_map(x1, y1)
        
        dx = abs(x1_map - x0_map)
        dy = abs(y1_map - y0_map)
        x = x0_map
        y = y0_map
        sx = -1 if x0_map > x1_map else 1
        sy = -1 if y0_map > y1_map else 1
        
        if dx > dy:
            err = dx / 2.0
            while x != x1_map:
                if self.is_in_map(x, y) and (self.map_data[y, x] == -1 or self.map_data[y, x] == 100):
                    self.map_data[y, x] = 0
                err -= dy
                if err < 0:
                    y += sy
                    err += dx
                x += sx
        else:
            err = dy / 2.0
            while y != y1_map:
                if self.is_in_map(x, y) and (self.map_data[y, x] == -1 or self.map_data[y, x] == 100):
                    self.map_data[y, x] = 0
                err -= dx
                if err < 0:
                    x += sx
                    err += dy
                y += sy


    def world_to_map(self, world_x, world_y):
        map_x = int((world_x - self.map_origin_x) / self.map_resolution)
        map_y = int((world_y - self.map_origin_y) / self.map_resolution)
        return map_x, map_y

    def is_in_map(self, map_x, map_y):
        return 0 <= map_x < self.map_data.shape[1] and 0 <= map_y < self.map_data.shape[0]

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
        t.child_frame_id = 'odom'
        
        t.transform.translation.x = 0.0
        t.transform.translation.y = 0.0
        t.transform.translation.z = 0.0
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = 0.0
        t.transform.rotation.w = 1.0
        
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    slam_node = SLAM()
    rclpy.spin(slam_node)
    slam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()