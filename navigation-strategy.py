#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, String, Bool
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
import json
import math
import threading
import time

class CompetitionNavigationStrategy(Node):
    """
    Node to implement round-specific navigation strategies for the competition.
    Integrates with round_state_manager to handle different behaviors in each round.
    """
    
    def __init__(self):
        super().__init__('competition_navigation_strategy')
        
        # Parameters
        self.declare_parameter('enable_fire_search', True)  # Whether to search for fire in Round 1
        self.declare_parameter('scan_pattern', 'spiral')    # Pattern for fire search (spiral, zigzag)
        self.declare_parameter('pause_between_goals', 3.0)  # Seconds to pause between navigation goals
        
        # State variables
        self.current_round = 1
        self.round_active = False
        self.is_navigating = False
        self.fire_location = None
        self.has_fire_location = False
        self.search_complete = False
        self.scan_pattern = self.get_parameter('scan_pattern').value
        self.pause_duration = self.get_parameter('pause_between_goals').value
        
        # Navigation control lock (for thread safety)
        self.nav_lock = threading.Lock()
        
        # Subscribers
        self.round_sub = self.create_subscription(
            Int32, 
            'competition/current_round', 
            self.round_callback, 
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            'competition/status',
            self.status_callback,
            10
        )
        
        self.fire_location_sub = self.create_subscription(
            PoseStamped,
            'competition/fire_location',
            self.fire_location_callback,
            10
        )
        
        self.map_sub = self.create_subscription(
            OccupancyGrid,
            'map',
            self.map_callback,
            10
        )
        
        self.scan_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10
        )
        
        # Publishers
        self.goal_pub = self.create_publisher(PoseStamped, 'goal_pose', 10)
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.fire_detected_pub = self.create_publisher(Bool, 'fire_detection/active', 10)
        self.fire_location_pub = self.create_publisher(PoseStamped, 'fire_detection/location', 10)
        
        # Timer for strategy execution
        self.strategy_timer = self.create_timer(1.0, self.execute_strategy)
        
        # Initialize
        self.get_logger().info("Competition Navigation Strategy initialized")
        self.get_logger().info(f"Search pattern: {self.scan_pattern}")
    
    def round_callback(self, msg):
        """Handle round updates"""
        self.current_round = msg.data
        self.get_logger().info(f"Round updated to: {self.current_round}")
        
        # Reset state when round changes
        with self.nav_lock:
            self.is_navigating = False
            self.search_complete = False
            
            # Configure fire detection based on round
            self.configure_fire_detection(self.current_round)
    
    def status_callback(self, msg):
        """Process competition status updates"""
        try:
            status = json.loads(msg.data)
            self.round_active = status.get('active', False)
            self.has_fire_location = status.get('has_fire_location', False)
            
            if status.get('round') != self.current_round:
                self.current_round = status.get('round')
                self.get_logger().info(f"Round updated via status to: {self.current_round}")
                
                # Reset state when round changes
                with self.nav_lock:
                    self.is_navigating = False
                    self.search_complete = False
                
                # Configure fire detection based on updated round
                self.configure_fire_detection(self.current_round)
            
        except json.JSONDecodeError:
            self.get_logger().error("Failed to parse status message")
    
    def fire_location_callback(self, msg):
        """Store fire location when received"""
        self.fire_location = msg
        self.has_fire_location = True
        self.get_logger().info(f"Received fire location at: ({msg.pose.position.x}, {msg.pose.position.y})")
    
    def map_callback(self, msg):
        """Process map updates for navigation planning"""
        # Store map info for navigation planning
        # Implementation would depend on how complex your navigation planning is
        pass
    
    def scan_callback(self, msg):
        """Process laser scan data for obstacle detection and fire search"""
        # Process scan data if needed for fire detection or obstacle avoidance
        # This implementation would depend on your specific sensors and strategy
        pass
    
    def configure_fire_detection(self, round_num):
        """Configure fire detection system based on current round"""
        fire_detection_active = False
        
        if round_num == 1:
            # In Round 1, we actively search for fire
            fire_detection_active = True
            self.get_logger().info("Round 1: Fire detection ENABLED")
        else:
            # In Rounds 2 and 3, we use saved fire location
            fire_detection_active = False
            self.get_logger().info(f"Round {round_num}: Fire detection DISABLED, using saved location")
        
        # Publish fire detection configuration
        msg = Bool()
        msg.data = fire_detection_active
        self.fire_detected_pub.publish(msg)
    
    def execute_strategy(self):
        """Execute the appropriate navigation strategy for the current round"""
        # Only proceed if the round is active
        if not self.round_active:
            return
        
        # Use a lock to prevent race conditions
        with self.nav_lock:
            if self.is_navigating:
                # Already executing a navigation task
                return
            
            # Mark as navigating to prevent concurrent execution
            self.is_navigating = True
        
        try:
            # Execute round-specific strategy
            if self.current_round == 1:
                self.execute_round1_strategy()
            elif self.current_round == 2:
                self.execute_round2_strategy()
            elif self.current_round == 3:
                self.execute_round3_strategy()
            else:
                self.get_logger().warn(f"Unknown round: {self.current_round}")
        finally:
            # Reset navigation flag when done
            with self.nav_lock:
                self.is_navigating = False
    
    def execute_round1_strategy(self):
        """Execute Round 1 strategy: Search for fire"""
        if self.search_complete:
            # If we've completed our search pattern, just wait
            self.get_logger().debug("Round 1: Search pattern complete, waiting for round to end")
            return
        
        self.get_logger().info("Executing Round 1 strategy: Searching for fire")
        
        # Generate search pattern if fire detection is enabled
        if self.get_parameter('enable_fire_search').value:
            # This would be replaced with your actual fire search algorithm
            search_points = self.generate_search_pattern()
            
            # Execute the search pattern
            self.execute_search_pattern(search_points)
            
            # Mark search as complete when done
            self.search_complete = True
        else:
            self.get_logger().info("Fire search disabled, waiting for external fire detection")
    
    def execute_round2_strategy(self):
        """Execute Round 2 strategy: Navigate to fire location from Round 1"""
        if not self.has_fire_location or self.fire_location is None:
            self.get_logger().warn("Round 2: No fire location available, cannot navigate")
            return
        
        self.get_logger().info("Executing Round 2 strategy: Navigating to saved fire location")
        
        # Simply publish the saved fire location directly to goal_pose
        # This will bypass teensy's fire detection and use the saved goal from Round 1
        self.goal_pub.publish(self.fire_location)
        
        # Mark as complete to avoid repeatedly sending the goal
        self.search_complete = True
    
    def execute_round3_strategy(self):
        """Execute Round 3 strategy: Navigate to hose release point, then to fire"""
        self.get_logger().info("Executing Round 3 strategy: Hose release and fire approach")
        
        if not self.has_fire_location or self.fire_location is None:
            self.get_logger().warn("Round 3: No fire location available, cannot navigate")
            return
        
        # Step 1: Navigate to the hose release point (this would be predefined or calculated)
        hose_release_point = self.get_hose_release_point()
        self.navigate_to_point(hose_release_point)
        
        # Pause to allow for hose latching
        time.sleep(self.pause_duration)
        
        # Step 2: Navigate to the fire location
        self.navigate_to_fire_location()
        
        # Step 3: Return to starting position
        # This would be implemented based on your specific requirements
    
    def generate_search_pattern(self):
        """
        Generate a search pattern for finding the fire in Round 1
        Returns a list of PoseStamped objects representing goals
        """
        # This is a simplified example - replace with your actual search algorithm
        search_points = []
        
        if self.scan_pattern == "spiral":
            # Generate a spiral pattern
            center_x, center_y = 0.0, 0.0  # Assuming start position is (0,0)
            radius = 0.5  # Starting radius in meters
            angle_step = 0.5  # Radians
            
            for i in range(10):  # Generate 10 points along spiral
                angle = i * angle_step
                radius_i = radius * (1 + i * 0.2)  # Increase radius as we go
                
                x = center_x + radius_i * math.cos(angle)
                y = center_y + radius_i * math.sin(angle)
                
                pose = self.create_pose_stamped(x, y, angle)
                search_points.append(pose)
                
        elif self.scan_pattern == "zigzag":
            # Generate a zigzag pattern
            # Implementation would depend on your specific requirements
            pass
        
        return search_points
    
    def execute_search_pattern(self, search_points):
        """Execute a search pattern by publishing navigation goals"""
        for pose in search_points:
            # Send the goal
            self.goal_pub.publish(pose)
            self.get_logger().info(f"Published search goal: ({pose.pose.position.x}, {pose.pose.position.y})")
            
            # Wait a moment for the robot to process and start moving
            time.sleep(0.5)
            
            # Wait for robot to reach the goal (or timeout)
            # In a real implementation, you would use action feedback or other mechanisms
            # This is a simplified blocking approach
            time.sleep(self.pause_duration)
    
    def navigate_to_fire_location(self):
        """Navigate to the saved fire location"""
        if self.fire_location is None:
            self.get_logger().warn("No fire location available to navigate to")
            return
        
        # Publish the fire location as a navigation goal
        self.goal_pub.publish(self.fire_location)
        self.get_logger().info(f"Navigating to fire at: ({self.fire_location.pose.position.x}, {self.fire_location.pose.position.y})")
    
    def navigate_to_point(self, pose):
        """Navigate to a specified pose"""
        if pose is None:
            self.get_logger().warn("Invalid navigation point")
            return
        
        # Publish the pose as a navigation goal
        self.goal_pub.publish(pose)
        self.get_logger().info(f"Navigating to point: ({pose.pose.position.x}, {pose.pose.position.y})")
    
    def get_hose_release_point(self):
        """Calculate or retrieve the hose release point for Round 3"""
        # This would be implemented based on your specific requirements
        # For now, we'll create a simple point halfway between start and fire
        
        if self.fire_location is None:
            self.get_logger().warn("No fire location available for hose release planning")
            return None
        
        # Create a point halfway to the fire (simple example)
        hose_point = PoseStamped()
        hose_point.header.frame_id = "map"
        hose_point.header.stamp = self.get_clock().now().to_msg()
        
        # Halfway point (adjust this based on your specific requirements)
        hose_point.pose.position.x = self.fire_location.pose.position.x / 2.0
        hose_point.pose.position.y = self.fire_location.pose.position.y / 2.0
        hose_point.pose.position.z = 0.0
        
        # Set orientation (facing the fire)
        angle = math.atan2(
            self.fire_location.pose.position.y - hose_point.pose.position.y,
            self.fire_location.pose.position.x - hose_point.pose.position.x
        )
        
        # Convert angle to quaternion (simplified)
        hose_point.pose.orientation.z = math.sin(angle / 2.0)
        hose_point.pose.orientation.w = math.cos(angle / 2.0)
        
        return hose_point
    
    def create_pose_stamped(self, x, y, angle=0.0):
        """Create a PoseStamped message with the given coordinates and angle"""
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = 0.0
        
        # Convert angle to quaternion (simplified)
        pose.pose.orientation.z = math.sin(angle / 2.0)
        pose.pose.orientation.w = math.cos(angle / 2.0)
        
        return pose

def main(args=None):
    rclpy.init(args=args)
    node = CompetitionNavigationStrategy()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
