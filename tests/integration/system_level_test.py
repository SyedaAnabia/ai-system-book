#!/usr/bin/env python3
"""
System-Level Test for Complete Pipeline

This module tests the complete pipeline from sensors through VSLAM, navigation, 
and simulation to ensure all components work together effectively.
"""

import rospy
import numpy as np
import cv2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image, LaserScan
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
from std_msgs.msg import String, Header
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus
import time
import threading
import json
from datetime import datetime
import matplotlib.pyplot as plt
from pathlib import Path


class SystemLevelTest:
    """
    Tests the complete pipeline from sensors through VSLAM, navigation, and simulation
    """
    
    def __init__(self):
        rospy.init_node('system_level_test', anonymous=True)
        
        # Parameters for system-level testing
        self.test_duration = rospy.get_param('~test_duration', 120.0)  # seconds
        self.results_dir = rospy.get_param('~results_dir', '/tmp/system_test_results')
        
        # Create results directory
        Path(self.results_dir).mkdir(parents=True, exist_ok=True)
        
        # Publishers and subscribers for monitoring the complete system
        self.cmd_vel_pub = rospy.Publisher('humanoid/cmd_vel', Twist, queue_size=10)
        self.goal_pub = rospy.Publisher('move_base_simple/goal', PoseStamped, queue_size=10)
        
        # Sensor data
        self.rgb_sub = rospy.Subscriber('/humanoid/camera/rgb/image_raw', Image, self.rgb_callback)
        self.depth_sub = rospy.Subscriber('/humanoid/camera/depth/image_raw', Image, self.depth_callback)
        self.lidar_sub = rospy.Subscriber('/humanoid/lidar/scan', LaserScan, self.lidar_callback)
        
        # Navigation and SLAM data
        self.odom_sub = rospy.Subscriber('/humanoid/odom', Odometry, self.odom_callback)
        self.pose_sub = rospy.Subscriber('/visual_slam/pose', PoseStamped, self.pose_callback)
        self.path_sub = rospy.Subscriber('/humanoid_path', Path, self.path_callback)
        
        # Performance monitoring
        self.diag_sub = rospy.Subscriber('/diagnostics', DiagnosticArray, self.diag_callback)
        
        # Internal state
        self.bridge = CvBridge()
        self.start_time = rospy.Time.now()
        
        # Data collection
        self.rgb_count = 0
        self.depth_count = 0
        self.lidar_count = 0
        self.pose_count = 0
        self.path_count = 0
        
        self.position_history = []
        self.pose_position_history = []
        self.odom_position_history = []
        self.diagnostic_history = []
        self.lidar_ranges_history = []
        
        # Performance metrics
        self.fps_rgb = 0.0
        self.fps_depth = 0.0
        self.fps_lidar = 0.0
        self.fps_pose = 0.0
        
        # Test status
        self.test_active = False
        self.test_results = {
            'status': 'not_started',
            'start_time': None,
            'end_time': None,
            'metrics': {},
            'issues': [],
            'performance': {},
            'components_status': {}
        }
        
        rospy.loginfo("System-Level Test initialized")
    
    def rgb_callback(self, msg):
        """Handle RGB image messages"""
        self.rgb_count += 1
        timestamp = msg.header.stamp.to_sec()
        
        # Verify image quality
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        
        if cv_image is not None:
            # Store image quality metrics
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
            brightness = np.mean(gray)
            sharpness = cv2.Laplacian(gray, cv2.CV_64F).var()
            
            # Record for analysis
            self.diagnostic_history.append({
                'timestamp': timestamp,
                'sensor': 'rgb',
                'brightness': brightness,
                'sharpness': sharpness
            })
    
    def depth_callback(self, msg):
        """Handle depth image messages"""
        self.depth_count += 1
        
        # Verify depth quality
        cv_depth = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        
        if cv_depth is not None:
            valid_depths = cv_depth[np.isfinite(cv_depth) & (cv_depth > 0)]
            if len(valid_depths) > 0:
                mean_depth = np.mean(valid_depths)
                std_depth = np.std(valid_depths)
            else:
                mean_depth, std_depth = 0, 0
            
            # Record for analysis
            self.diagnostic_history.append({
                'timestamp': msg.header.stamp.to_sec(),
                'sensor': 'depth',
                'mean_depth': mean_depth,
                'std_depth': std_depth
            })
    
    def lidar_callback(self, msg):
        """Handle LIDAR scan messages"""
        self.lidar_count += 1
        timestamp = msg.header.stamp.to_sec()
        
        # Store LIDAR data for analysis
        ranges = np.array(msg.ranges)
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > 0)]
        self.lidar_ranges_history.append(valid_ranges)
        
        if len(valid_ranges) > 0:
            mean_range = np.mean(valid_ranges)
            std_range = np.std(valid_ranges)
        else:
            mean_range, std_range = 0, 0
        
        # Record for analysis
        self.diagnostic_history.append({
            'timestamp': timestamp,
            'sensor': 'lidar',
            'mean_range': mean_range,
            'std_range': std_range
        })
    
    def odom_callback(self, msg):
        """Handle odometry messages"""
        pos = msg.pose.pose.position
        self.odom_position_history.append([pos.x, pos.y, pos.z, rospy.Time.now().to_sec()])
    
    def pose_callback(self, msg):
        """Handle pose estimation messages from VSLAM"""
        self.pose_count += 1
        pos = msg.pose.position
        timestamp = msg.header.stamp.to_sec()
        
        self.pose_position_history.append([pos.x, pos.y, pos.z, timestamp])
        
        # Record for analysis
        self.diagnostic_history.append({
            'timestamp': timestamp,
            'sensor': 'vslam_pose',
            'position': [pos.x, pos.y, pos.z]
        })
    
    def path_callback(self, msg):
        """Handle path messages from planner"""
        self.path_count += 1
        # Path points can be analyzed for planning quality
    
    def diag_callback(self, msg):
        """Handle diagnostic messages"""
        for status in msg.status:
            self.diagnostic_history.append({
                'timestamp': rospy.Time.now().to_sec(),
                'component': status.name,
                'level': status.level,
                'message': status.message,
                'values': {v.key: v.value for v in status.values}
            })
    
    def start_test(self):
        """Start the system-level test"""
        rospy.loginfo("Starting system-level test for complete pipeline...")
        
        self.test_active = True
        self.test_start_time = rospy.Time.now()
        self.test_results['status'] = 'running'
        self.test_results['start_time'] = self.test_start_time.to_sec()
        
        # Start monitoring threads
        self.monitoring_thread = threading.Thread(target=self.monitor_performance)
        self.monitoring_thread.start()
        
        # Execute test scenario: navigation to a goal while monitoring all components
        self.execute_navigation_test()
    
    def execute_navigation_test(self):
        """Execute navigation test while monitoring performance"""
        rospy.loginfo("Executing navigation test while monitoring system...")
        
        # Set a navigation goal
        goal = PoseStamped()
        goal.header.stamp = rospy.Time.now()
        goal.header.frame_id = "map"
        goal.pose.position.x = 10.0
        goal.pose.position.y = 10.0
        goal.pose.position.z = 0.0
        goal.pose.orientation.w = 1.0
        
        rospy.loginfo("Sending navigation goal to (10.0, 10.0)")
        self.goal_pub.publish(goal)
        
        # Monitor system for the duration
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < self.test_duration and self.test_active:
            # Continue monitoring all system components
            time.sleep(0.1)
    
    def monitor_performance(self):
        """Monitor system performance metrics in a separate thread"""
        start_time = rospy.Time.now()
        
        while (rospy.Time.now() - start_time).to_sec() < self.test_duration and self.test_active:
            # Calculate data rates
            elapsed = (rospy.Time.now() - self.test_start_time).to_sec()
            
            if elapsed > 0:
                self.fps_rgb = self.rgb_count / elapsed
                self.fps_depth = self.depth_count / elapsed
                self.fps_lidar = self.lidar_count / elapsed
                self.fps_pose = self.pose_count / elapsed
            
            time.sleep(1.0)  # Update rates every second
    
    def stop_test(self):
        """Stop the system-level test"""
        rospy.loginfo("Stopping system-level test...")
        
        self.test_active = False
        self.test_results['end_time'] = rospy.Time.now().to_sec()
        self.test_results['status'] = 'completed'
        
        # Wait for monitoring thread to finish
        if hasattr(self, 'monitoring_thread') and self.monitoring_thread.is_alive():
            self.monitoring_thread.join()
        
        # Stop robot movement
        stop_cmd = Twist()
        self.cmd_vel_pub.publish(stop_cmd)
        
        # Analyze results
        self.analyze_results()
        
        # Generate test report
        self.generate_test_report()
    
    def analyze_results(self):
        """Analyze the collected system-level data"""
        rospy.loginfo("Analyzing system-level test results...")
        
        elapsed_time = (rospy.Time.now() - self.test_start_time).to_sec()
        
        # Performance metrics
        self.test_results['performance'] = {
            'test_duration': elapsed_time,
            'rgb_rate': self.fps_rgb,
            'depth_rate': self.fps_depth,
            'lidar_rate': self.fps_lidar,
            'pose_rate': self.fps_pose,
            'total_rgb_frames': self.rgb_count,
            'total_depth_frames': self.depth_count,
            'total_lidar_scans': self.lidar_count,
            'total_poses': self.pose_count
        }
        
        # Position analysis
        if len(self.pose_position_history) > 1 and len(self.odom_position_history) > 1:
            pose_positions = np.array(self.pose_position_history)[:, :3]
            odom_positions = np.array(self.odom_position_history)[:, :3]
            
            # Calculate path lengths
            pose_path_length = 0
            for i in range(1, len(pose_positions)):
                pose_path_length += np.linalg.norm(pose_positions[i] - pose_positions[i-1])
            
            odom_path_length = 0
            for i in range(1, len(odom_positions)):
                odom_path_length += np.linalg.norm(odom_positions[i] - odom_positions[i-1])
            
            # Calculate drift between pose and odometry
            if len(pose_positions) == len(odom_positions):
                position_drift = np.linalg.norm(pose_positions - odom_positions, axis=1)
                avg_drift = np.mean(position_drift)
                max_drift = np.max(position_drift)
            else:
                avg_drift = max_drift = 0
            
            self.test_results['metrics'].update({
                'pose_path_length': pose_path_length,
                'odom_path_length': odom_path_length,
                'avg_position_drift': avg_drift,
                'max_position_drift': max_drift,
                'position_samples': len(pose_positions)
            })
        
        # LIDAR analysis
        if self.lidar_ranges_history:
            all_ranges = np.concatenate(self.lidar_ranges_history)
            self.test_results['metrics'].update({
                'lidar_min_range': float(np.min(all_ranges)) if len(all_ranges) > 0 else 0,
                'lidar_max_range': float(np.max(all_ranges)) if len(all_ranges) > 0 else 0,
                'lidar_avg_range': float(np.mean(all_ranges)) if len(all_ranges) > 0 else 0,
                'lidar_readings': len(all_ranges)
            })
        
        # Check for component issues
        component_issues = set()
        for diag in self.diagnostic_history:
            if 'level' in diag and diag['level'] > 1:  # Error level
                component_issues.add(diag.get('component', diag.get('sensor', 'unknown')))
        
        if component_issues:
            self.test_results['issues'].append(f"Component errors detected: {', '.join(component_issues)}")
        
        # Check data rates
        expected_rates = {
            'rgb_rate': 30,      # 30 FPS for RGB
            'depth_rate': 30,    # 30 FPS for depth
            'lidar_rate': 10,    # 10 Hz for LIDAR
            'pose_rate': 10      # 10 Hz for pose updates
        }
        
        for rate_name, expected in expected_rates.items():
            actual = self.test_results['performance'].get(rate_name, 0)
            if actual < expected * 0.8:  # Less than 80% of expected
                self.test_results['issues'].append(f"Low {rate_name}: {actual:.2f} vs {expected}")
    
    def generate_test_report(self):
        """Generate a detailed system-level test report"""
        report = {
            'test_info': {
                'name': 'System-Level Pipeline Test',
                'description': 'Complete pipeline test from sensors through VSLAM, navigation, and simulation',
                'start_time': self.test_results['start_time'],
                'end_time': self.test_results['end_time'],
                'duration': self.test_results['performance']['test_duration']
            },
            'performance': self.test_results['performance'],
            'metrics': self.test_results['metrics'],
            'issues': self.test_results['issues'],
            'summary': {
                'status': self.test_results['status'],
                'issues_count': len(self.test_results['issues']),
                'components_tested': [
                    'RGB Camera', 'Depth Camera', 'LIDAR',
                    'VSLAM', 'Path Planner', 'Navigation Controller',
                    'Robot Simulation'
                ]
            }
        }
        
        # Save report as JSON
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        report_filename = Path(self.results_dir) / f"system_level_test_report_{timestamp}.json"
        
        with open(report_filename, 'w') as f:
            json.dump(report, f, indent=2)
        
        rospy.loginfo(f"System-level test report saved: {report_filename}")
        
        # Print summary
        rospy.loginfo(f"System-level test summary:")
        rospy.loginfo(f"  Status: {report['summary']['status']}")
        rospy.loginfo(f"  Issues: {report['summary']['issues_count']}")
        rospy.loginfo(f"  Duration: {report['test_info']['duration']:.2f}s")
        rospy.loginfo(f"  Performance - RGB: {report['performance']['rgb_rate']:.2f} FPS, "
                      f"Depth: {report['performance']['depth_rate']:.2f} FPS, "
                      f"LIDAR: {report['performance']['lidar_rate']:.2f} Hz, "
                      f"Pose: {report['performance']['pose_rate']:.2f} Hz")
        
        if report['issues']:
            rospy.loginfo("Issues found:")
            for issue in report['issues']:
                rospy.loginfo(f"  - {issue}")


def main():
    test_module = SystemLevelTest()
    
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