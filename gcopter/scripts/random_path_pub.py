#!/usr/bin/env python3

import rospy
import random
import math
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class RandomPathPublisher:
    def __init__(self):
        rospy.init_node('random_path_publisher', anonymous=True)
        
        # Create publisher for the path topic
        self.path_pub = rospy.Publisher('path_topic', Path, queue_size=10)
        
        # Set publishing rate (1 Hz)
        self.rate = rospy.Rate(1)
        
        # Path parameters
        self.num_points = 10  # Number of points in the path
        self.max_x = 10.0     # Maximum x coordinate
        self.max_y = 10.0     # Maximum y coordinate
        self.frame_id = "map" # Reference frame
        
        rospy.loginfo("Random Path Publisher initialized with 45-degree angle constraint")
        
    def generate_random_path(self):
        """Generate a random path with angle constraints between segments"""
        path = Path()
        
        # Set header
        path.header = Header()
        path.header.stamp = rospy.Time.now()
        path.header.frame_id = self.frame_id
        
        # Step size for path generation
        step_size = 2.0  # Distance between consecutive points
        max_angle_change = math.pi / 4  # 45 degrees in radians
        
        # Generate first pose randomly
        prev_x = random.uniform(-self.max_x/2, self.max_x/2)
        prev_y = random.uniform(-self.max_y/2, self.max_y/2)
        prev_direction = random.uniform(-math.pi, math.pi)  # Initial direction
        
        for i in range(self.num_points):
            pose_stamped = PoseStamped()
            
            # Set header for each pose
            pose_stamped.header = Header()
            pose_stamped.header.stamp = rospy.Time.now()
            pose_stamped.header.frame_id = self.frame_id
            
            if i == 0:
                # First point - use initial random position
                pose_stamped.pose.position.x = prev_x
                pose_stamped.pose.position.y = prev_y
                current_direction = prev_direction
            else:
                # Subsequent points - constrain angle change
                # Generate new direction within angle constraints
                angle_change = random.uniform(-max_angle_change, max_angle_change)
                current_direction = prev_direction + angle_change
                
                # Calculate new position
                new_x = prev_x + step_size * math.cos(current_direction)
                new_y = prev_y + step_size * math.sin(current_direction)
                
                # Keep within bounds by reflecting if necessary
                if abs(new_x) > self.max_x:
                    new_x = prev_x - step_size * math.cos(current_direction)
                    current_direction = math.pi - current_direction
                    
                if abs(new_y) > self.max_y:
                    new_y = prev_y - step_size * math.sin(current_direction)
                    current_direction = -current_direction
                
                pose_stamped.pose.position.x = new_x
                pose_stamped.pose.position.y = new_y
                
                # Update previous values
                prev_x = new_x
                prev_y = new_y
                prev_direction = current_direction
            
            pose_stamped.pose.position.z = 0.0
            
            # Set orientation based on direction of travel
            # For the last point, use the direction from previous segment
            if i == self.num_points - 1 and i > 0:
                # Use previous direction for last pose
                yaw = prev_direction
            else:
                yaw = current_direction
            
            pose_stamped.pose.orientation.x = 0.0
            pose_stamped.pose.orientation.y = 0.0
            pose_stamped.pose.orientation.z = math.sin(yaw / 2.0)
            pose_stamped.pose.orientation.w = math.cos(yaw / 2.0)
            
            # Add pose to path
            path.poses.append(pose_stamped)
        
        return path
    
    def run(self):
        """Main loop to publish random paths"""
        # while not rospy.is_shutdown():
        for _ in range(2):  # Publish 10 paths for demonstration
            # Generate and publish random path
            random_path = self.generate_random_path()
            self.path_pub.publish(random_path)
            
            rospy.loginfo(f"Published random path with {len(random_path.poses)} poses")
            
            # Sleep according to the set rate
            self.rate.sleep()

if __name__ == '__main__':
    try:
        publisher = RandomPathPublisher()
        publisher.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Random Path Publisher stopped")