#!/usr/bin/env python3
"""
Simulation Integration Test

This module tests the complete integration of the Isaac Humanoid Simulation
and Navigation System components.
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from tf.transformations import euler_from_quaternion
import time
import threading
import subprocess
import signal
import sys
import os
from datetime import datetime


class SimulationIntegrationTest:
    """
    Tests the complete integration of the Isaac Humanoid Simulation and Navigation System
    """
    
    def __init__(self):
        rospy.init_node('simulation_integration_test', anonymous=True)
        
        # Parameters for integration testing
        self.test_duration = rospy.get_param('~test_duration', 60.0)  # seconds
        self.test_scenario = rospy.get_param('~test_scenario', 'basic_navigation')
        self.results_dir = rospy.get_param('~results_dir', '/tmp/simulation_test_results')
        
        # Create results directory
        os.makedirs(self.results_dir, exist_ok=True)
        
        # Publishers and subscribers for monitoring the system
        self.cmd_vel_pub = rospy.Publisher('humanoid/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        
        self.rgb_sub = rospy.Subscriber('/humanoid/camera/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/humanoid/camera/depth/image_raw', Image, self.depth_callback)
        self.lidar_sub = rospy.Subscriber('/humanoid/lidar/scan', LaserScan, self.lidar_callback)
        self.odom_sub = rospy.Subscriber('/humanoid/odom', Odometry, self.odom_callback)
        self.pose_sub = rospy.Subscriber('/visual_slam/pose', PoseStamped, self.pose_callback)
        
        # Internal state
        self.bridge = CvBridge()
        self.start_time = rospy.Time.now()
        self.test_start_time = rospy.Time.now()
        
        # Data collection
        self.rgb_count = 0
        self.depth_count = 0
        self.lidar_count = 0
        self.pose_count = 0
        
        self.position_history = []
        self.velocity_history = []
        
        # Test status
        self.test_active = False
        self.test_results = {
            'status': 'not_started',
            'start_time': None,
            'end_time': None,
            'results': {},
            'issues': []
        }
        
        rospy.loginfo("Simulation Integration Test initialized")
    
    def rgb_callback(self, msg):
        """Handle RGB image messages"""
        self.rgb_count += 1
        
        # Verify image quality metrics
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # Check if image is valid
        if cv_image is not None:
            # Check brightness
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            brightness = np.mean(gray)
            
            if brightness < 10 or brightness > 245:  # Too dark or too bright
                self.test_results['issues'].append(f"RGB image brightness issue: {brightness}")
    
    def depth_callback(self, msg):
        """Handle depth image messages"""
        self.depth_count += 1
        
        # Verify depth image metrics
        cv_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        
        if cv_depth is not None:
            # Check for valid depth values
            valid_depths = cv_depth[np.isfinite(cv_depth) & (cv_depth > 0)]
            
            if len(valid_depths) == 0:
                self.test_results['issues'].append("Depth image has no valid values")
            else:
                # Check depth range
                min_depth = np.min(valid_depths)
                max_depth = np.max(valid_depths)
                
                if min_depth < 0.05 or max_depth > 25.0:  # Outside typical range
                    self.test_results['issues'].append(f"Depth range issue: {min_depth:.2f} - {max_depth:.2f}")
    
    def lidar_callback(self, msg):
        """Handle LIDAR scan messages"""
        self.lidar_count += 1
        
        # Verify LIDAR data quality
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0)]
        
        if len(valid_ranges) == 0:
            self.test_results['issues'].append("LIDAR has no valid ranges")
        else:
            # Check for reasonable range values
            if np.max(valid_ranges) > 25.0:
                self.test_results['issues'].append("LIDAR max range seems too high")
            if np.min(valid_ranges) < 0.05:
                self.test_results['issues'].append("LIDAR min range seems too low")
    
    def odom_callback(self, msg):
        """Handle odometry messages"""
        # Store position and velocity for analysis
        pos = msg.pose.pose.position
        vel = msg.twist.twist.linear
        self.position_history.append([pos.x, pos.y, pos.z])
        self.velocity_history.append([vel.x, vel.y, vel.z])
    
    def pose_callback(self, msg):
        """Handle pose estimation messages"""
        self.pose_count += 1
        
        # Check for pose quality
        pos = msg.pose.position
        orientation_q = msg.pose.orientation
        (roll, pitch, yaw) = euler_from_quaternion([orientation_q.x, orientation_q.y, 
                                                  orientation_q.z, orientation_q.w])
        
        # Store pose data for analysis
        self.position_history.append([pos.x, pos.y, pos.z])
    
    def start_test(self):
        """Start the integration test"""
        rospy.loginfo(f"Starting simulation integration test for scenario: {self.test_scenario}")
        
        self.test_active = True
        self.test_start_time = rospy.Time.now()
        self.test_results['status'] = 'running'
        self.test_results['start_time'] = self.test_start_time.to_sec()
        
        # Execute the test scenario
        if self.test_scenario == 'basic_navigation':
            self.execute_basic_navigation_test()
        elif self.test_scenario == 'sensor_integration':
            self.execute_sensor_integration_test()
        elif self.test_scenario == 'vslam_performance':
            self.execute_vslam_performance_test()
        else:
            rospy.logwarn(f"Unknown test scenario: {self.test_scenario}, using basic_navigation")
            self.execute_basic_navigation_test()
    
    def execute_basic_navigation_test(self):
        """Execute a basic navigation test"""
        rospy.loginfo("Executing basic navigation test...")
        
        # Send a simple navigation goal
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = 5.0
        goal.pose.position.y = 5.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0  # No rotation
        
        rospy.loginfo("Sending navigation goal to (5.0, 5.0)")
        self.goal_pub.publish(goal)
        
        # Monitor for completion within test duration
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < self.test_duration and self.test_active:
            # Continue monitoring sensors and navigation status
            time.sleep(0.1)
    
    def execute_sensor_integration_test(self):
        """Execute a sensor integration test"""
        rospy.loginfo("Executing sensor integration test...")
        
        # Monitor sensor data rates and quality
        initial_rgb_count = self.rgb_count
        initial_depth_count = self.depth_count
        initial_lidar_count = self.lidar_count
        initial_pose_count = self.pose_count
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < self.test_duration and self.test_active:
            # Check data rates
            current_time = (rospy.Time.now() - start_time).to_sec()
            
            if current_time > 0:
                rgb_rate = (self.rgb_count - initial_rgb_count) / current_time
                depth_rate = (self.depth_count - initial_depth_count) / current_time
                lidar_rate = (self.lidar_count - initial_lidar_count) / current_time
                pose_rate = (self.pose_count - initial_pose_count) / current_time
                
                # Log data rates
                if int(current_time) % 10 == 0:  # Log every 10 seconds
                    rospy.loginfo(f"Data rates - RGB: {rgb_rate:.2f}, Depth: {depth_rate:.2f}, "
                                  f"LIDAR: {lidar_rate:.2f}, Pose: {pose_rate:.2f}")
    
    def execute_vslam_performance_test(self):
        """Execute a VSLAM performance test"""
        rospy.loginfo("Executing VSLAM performance test...")
        
        # Record initial pose and position
        initial_position = None
        if len(self.position_history) > 0:
            initial_position = self.position_history[-1]
        
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < self.test_duration and self.test_active:
            # Move the robot to generate visual features
            cmd = Twist()
            cmd.linear.x = 0.2  # Move forward slowly
            cmd.angular.z = 0.1  # Gentle turn
            self.cmd_vel_pub.publish(cmd)
            
            # Monitor pose estimation quality
            time.sleep(0.1)
    
    def stop_test(self):
        """Stop the integration test"""
        rospy.loginfo("Stopping simulation integration test...")
        
        self.test_active = False
        self.test_results['end_time'] = rospy.Time.now().to_sec()
        self.test_results['status'] = 'completed'
        
        # Stop robot movement
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Analyze results
        self.analyze_results()
        
        # Generate test report
        self.generate_test_report()
    
    def analyze_results(self):
        """Analyze the collected test data"""
        rospy.loginfo("Analyzing test results...")
        
        # Calculate data rates
        elapsed_time = (rospy.Time.now() - self.test_start_time).to_sec()
        
        if elapsed_time > 0:
            self.test_results['results']['rgb_rate'] = self.rgb_count / elapsed_time
            self.test_results['results']['depth_rate'] = self.depth_count / elapsed_time
            self.test_results['results']['lidar_rate'] = self.lidar_count / elapsed_time
            self.test_results['results']['pose_rate'] = self.pose_count / elapsed_time
        
        # Analyze position consistency if data available
        if len(self.position_history) > 1:
            positions = np.array(self.position_history)
            position_variance = np.var(positions, axis=0)
            
            self.test_results['results']['position_variance'] = position_variance.tolist()
            self.test_results['results']['position_std'] = np.std(positions, axis=0).tolist()
            self.test_results['results']['total_positions'] = len(self.position_history)
        
        # Check for data loss
        expected_rgb = elapsed_time * 30  # Assuming 30 FPS
        expected_depth = elapsed_time * 30  # Assuming 30 FPS
        expected_lidar = elapsed_time * 10  # Assuming 10 Hz
        expected_pose = elapsed_time * 30  # Assuming 30 Hz from VSLAM
        
        if self.rgb_count < expected_rgb * 0.8:
            self.test_results['issues'].append(f"RGB data loss: {self.rgb_count}/{expected_rgb}")
        if self.depth_count < expected_depth * 0.8:
            self.test_results['issues'].append(f"Depth data loss: {self.depth_count}/{expected_depth}")
        if self.lidar_count < expected_lidar * 0.8:
            self.test_results['issues'].append(f"LIDAR data loss: {self.lidar_count}/{expected_lidar}")
        if self.pose_count < expected_pose * 0.5:  # Lower threshold for pose
            self.test_results['issues'].append(f"Pose data loss: {self.pose_count}/{expected_pose}")
    
    def generate_test_report(self):
        """Generate a detailed test report"""
        report = {
            'test_info': {
                'scenario': self.test_scenario,
                'duration': self.test_duration,
                'start_time': self.test_results['start_time'],
                'end_time': self.test_results['end_time'],
                'elapsed_time': self.test_results['end_time'] - self.test_results['start_time']
            },
            'results': self.test_results['results'],
            'issues': self.test_results['issues'],
            'summary': {
                'status': self.test_results['status'],
                'issues_count': len(self.test_results['issues']),
                'rgb_messages': self.rgb_count,
                'depth_messages': self.depth_count,
                'lidar_messages': self.lidar_count,
                'pose_messages': self.pose_count
            }
        }
        
        # Save report as JSON
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_filename = os.path.join(self.results_dir, f"integration_test_report_{timestamp}.json")
        
        with open(report_filename, 'w') as f:
            import json
            json.dump(report, f, indent=2)
        
        rospy.loginfo(f"Test report saved: {report_filename}")
        
        # Print summary
        rospy.loginfo(f"Integration test summary:")
        rospy.loginfo(f"  Status: {report['summary']['status']}")
        rospy.loginfo(f"  Issues: {report['summary']['issues_count']}")
        rospy.loginfo(f"  Messages - RGB: {report['summary']['rgb_messages']}, "
                      f"Depth: {report['summary']['depth_messages']}, "
                      f"LIDAR: {report['summary']['lidar_messages']}, "
                      f"Pose: {report['summary']['pose_messages']}")
        
        if report['issues']:
            rospy.loginfo("Issues found:")
            for issue in report['issues']:
                rospy.loginfo(f"  - {issue}")


def main():
    test_module = SimulationIntegrationTest()
    
    # Allow time for system initialization
    rospy.sleep(5.0)
    
    try:
        # Start the test
        test_module.start_test()
        
        # Run for the specified duration
        rospy.sleep(test_module.test_duration)
        
        # Stop the test
        test_module.stop_test()
        
        rospy.spin()
        
    except KeyboardInterrupt:
        rospy.loginfo("Interrupted by user")
        test_module.stop_test()


if __name__ == '__main__':
    main()