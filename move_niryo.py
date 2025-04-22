#!/usr/bin/env python

import rospy
import moveit_commander
import time
import subprocess
import os
from geometry_msgs.msg import Pose, Point, Quaternion, Vector3
import threading
import math
import copy
from visualization_msgs.msg import Marker
from std_msgs.msg import Header, ColorRGBA

class SimpleNiryoMover:
    def __init__(self):
        """Initialize the ROS node and set up MoveIt"""
        # Initialize ROS node
        moveit_commander.roscpp_initialize([])
        rospy.init_node('simple_niryo_mover', anonymous=True)
        
        # Initialize visualization components first
        self.marker_pub = rospy.Publisher('visualization_marker', Marker, queue_size=10)
        self.test_tube_marker = self.create_test_tube_marker()
        self.cup_marker = self.create_cup_marker()
        self.table_marker, self.grain_markers, self.leg_markers = self.create_table_markers()
        
        # Flag to track if test tube is being carried
        self.test_tube_picked = False
        
        # Create mutex for thread safety when updating markers
        self.marker_mutex = threading.Lock()
        
        # Publish the table and objects first
        rospy.loginfo("Showing table and objects first...")
        self.publish_table_and_objects()
        
        # Give a short delay to allow the markers to be visible first
        time.sleep(2)
        
        # Now start robot processes
        self.start_robot_processes()
        
        # Give time for ROS to initialize
        rospy.loginfo("Waiting for ROS processes to initialize...")
        time.sleep(10)
        
        # Initialize MoveIt components
        try:
            rospy.loginfo("Connecting to robot...")
            self.robot = moveit_commander.RobotCommander()
            self.scene = moveit_commander.PlanningSceneInterface()
            self.move_group = moveit_commander.MoveGroupCommander("arm")
            
            # Set slower velocity for more controlled movement
            self.move_group.set_max_velocity_scaling_factor(0.2)
            
            # Increase planning time for better paths
            self.move_group.set_planning_time(10.0)
            
            # Keep track of end effector link for visualization
            self.end_effector_link = self.move_group.get_end_effector_link()
            
            # Start a thread to continuously publish the markers
            self.marker_thread = threading.Thread(target=self.publish_markers_continuously)
            self.marker_thread.daemon = True
            self.marker_thread.start()
            
            # Wait for the publisher to establish connections
            time.sleep(2)
            
            rospy.loginfo("Simple Niryo Mover initialized successfully!")
        except Exception as e:
            rospy.logerr("Failed to initialize robot: {}".format(e))
            raise
    
    def create_test_tube_marker(self):
        """Create the test tube marker for visualization"""
        test_tube = Marker(
            type=Marker.CYLINDER,
            id=21,
            lifetime=rospy.Duration(),
            pose=Pose(Point(0.2, 0, -0.03), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.02, 0.02, 0.12),  # Adjusted for better visibility
            header=Header(frame_id='base_link'),
            color=ColorRGBA(0.9, 0.1, 0.1, 0.9),  # Red tube
            mesh_use_embedded_materials=False
        )
        return test_tube
    
    def create_cup_marker(self):
        """Create the cup marker for visualization"""
        cup = Marker(
            type=Marker.CYLINDER,
            id=20,
            lifetime=rospy.Duration(),
            pose=Pose(Point(0, 0.3, -0.03), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.08, 0.08, 0.12),
            header=Header(frame_id='base_link'),
            color=ColorRGBA(0.0, 0.3, 0.8, 0.8),  # Blue cup
            mesh_use_embedded_materials=False
        )
        return cup
    
    def create_table_markers(self):
        """Creates all the markers needed for table visualization"""
        # Create table top marker
        table_marker = Marker(
            type=Marker.CUBE,
            id=0,
            lifetime=rospy.Duration(),
            pose=Pose(Point(0, 0, -0.08), Quaternion(0, 0, 0, 1)),
            scale=Vector3(0.8, 0.8, 0.05),
            header=Header(frame_id='base_link'),
            color=ColorRGBA(0.65, 0.3, 0.15, 1.0),
            mesh_use_embedded_materials=False
        )
        
        # Create wooden texture through patterns
        grain_markers = []
        for i in range(6):
            grain = Marker(
                type=Marker.CUBE,
                id=i+1,
                lifetime=rospy.Duration(),
                pose=Pose(Point(0, (i*0.12) - 0.3, -0.079), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.8, 0.04, 0.01),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.3, 0.15, 0.05, 0.8),
                mesh_use_embedded_materials=False
            )
            grain_markers.append(grain)
        
        # Add table legs
        leg_positions = [
            (0.35, 0.35, -0.33),
            (0.35, -0.35, -0.33),
            (-0.35, 0.35, -0.33),
            (-0.35, -0.35, -0.33)
        ]
        
        leg_markers = []
        for i, pos in enumerate(leg_positions):
            leg = Marker(
                type=Marker.CYLINDER,
                id=i+11,
                lifetime=rospy.Duration(),
                pose=Pose(Point(pos[0], pos[1], pos[2]), Quaternion(0, 0, 0, 1)),
                scale=Vector3(0.05, 0.05, 0.5),
                header=Header(frame_id='base_link'),
                color=ColorRGBA(0.55, 0.27, 0.06, 1.0),
                mesh_use_embedded_materials=False
            )
            leg_markers.append(leg)
        
        return table_marker, grain_markers, leg_markers
    
    def calculate_test_tube_offset(self, gripper_pose):
        """Calculate the correct position for the test tube relative to the gripper"""
        # These offsets position the test tube correctly in the gripper
        # We need to account for the orientation of the gripper
        q = gripper_pose.orientation
        
        # Convert the quaternion to a rotation matrix
        # This is a simplified calculation - we mainly care about z-axis orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        
        # Calculate the offset based on the orientation
        z_offset = -0.04  # Reduced from -0.06 to -0.04 for tighter grip
        
        # Adjust position based on gripper orientation
        offset_x = z_offset * math.sin(yaw)
        offset_y = z_offset * -math.cos(yaw)
        offset_z = -0.01  # Reduced from -0.02 to -0.01 for better vertical alignment
        
        new_pose = copy.deepcopy(gripper_pose)
        new_pose.position.x += offset_x
        new_pose.position.y += offset_y
        new_pose.position.z += offset_z
        
        return new_pose
    
    def publish_markers_continuously(self):
        """Continuously publish markers in a separate thread"""
        rate = rospy.Rate(10)  # 10 Hz for marker updates
        last_error_time = 0
        
        while not rospy.is_shutdown():
            try:
                # Update test tube position if picked up
                if self.test_tube_picked:
                    try:
                        current_pose = self.move_group.get_current_pose(self.end_effector_link).pose
                        self.update_test_tube_position(current_pose)
                    except Exception as e:
                        # Only log errors once per second to avoid flooding
                        current_time = time.time()
                        if current_time - last_error_time > 1.0:
                            rospy.logwarn("Failed to get current pose for visualization: {}".format(e))
                            last_error_time = current_time
                
                # Publish markers
                with self.marker_mutex:
                    self.publish_static_markers()
            except Exception as e:
                # Only log errors once per second to avoid flooding
                current_time = time.time()
                if current_time - last_error_time > 1.0:
                    rospy.logwarn("Error in marker thread: {}".format(e))
                    last_error_time = current_time
            
            rate.sleep()
    
    def update_test_tube_position(self, gripper_pose):
        """Update the test tube position to match the end effector"""
        try:
            # Calculate the correct position for the test tube outside the lock
            new_pose = self.calculate_test_tube_offset(gripper_pose)
            
            # Now try to publish the marker without using the lock from the caller
            # since this method might be called while already holding the lock
            try:
                # Update the test tube marker with new position
                self.test_tube_marker.pose = new_pose
                self.test_tube_marker.action = Marker.MODIFY
                
                # Ensure the test tube doesn't clip through the table
                if self.test_tube_marker.pose.position.z < -0.02:
                    self.test_tube_marker.pose.position.z = -0.02
                
                # Publish the updated marker
                self.marker_pub.publish(self.test_tube_marker)
            except Exception as e:
                rospy.logwarn("Error publishing test tube marker: {}".format(e))
        except Exception as e:
            rospy.logwarn("Error calculating test tube position: {}".format(e))
    
    def publish_static_markers(self):
        """Publish all static markers (cup, table, and test tube if not picked up)"""
        # Publish table top
        self.marker_pub.publish(self.table_marker)
        
        # Publish grain patterns
        for grain in self.grain_markers:
            self.marker_pub.publish(grain)
        
        # Publish table legs
        for leg in self.leg_markers:
            self.marker_pub.publish(leg)
        
        # Publish cup
        self.marker_pub.publish(self.cup_marker)
        
        # Publish test tube only if not picked up
        if not self.test_tube_picked:
            self.marker_pub.publish(self.test_tube_marker)
    
    def publish_table_and_objects(self):
        """Publish the table and objects for initial visualization"""
        rate = rospy.Rate(1)  # 1 Hz is enough for initial visualization
        
        # Publish for a few seconds to make sure they're visible
        for _ in range(3):
            # Publish table top
            self.marker_pub.publish(self.table_marker)
            
            # Publish grain patterns
            for grain in self.grain_markers:
                self.marker_pub.publish(grain)
            
            # Publish table legs
            for leg in self.leg_markers:
                self.marker_pub.publish(leg)
            
            # Publish cup
            self.marker_pub.publish(self.cup_marker)
            
            # Publish test tube
            self.marker_pub.publish(self.test_tube_marker)
            
            rate.sleep()
    
    def start_robot_processes(self):
        """Start the necessary ROS processes for the Niryo One robot"""
        try:
            # Try to check if the robot description parameter already exists on the parameter server
            if not rospy.has_param('/robot_description'):
                rospy.loginfo("Robot description not found on parameter server, launching simulation...")
                
                # Launch the standard desktop simulation with RViz
                rospy.loginfo("Launching niryo_one_bringup desktop_rviz_simulation.launch")
                self.simulation_process = subprocess.Popen(
                    ["roslaunch", "niryo_one_bringup", "desktop_rviz_simulation.launch"],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT
                )
                
                # Output process stdout for debugging - in a non-blocking way
                self.start_process_output_monitor(self.simulation_process, "Simulation")
                
                # Wait for robot description to appear on parameter server
                rospy.loginfo("Waiting for robot_description to appear on parameter server...")
                start_time = time.time()
                while not rospy.has_param('/robot_description') and time.time() - start_time < 20.0:
                    time.sleep(0.5)
                    rospy.loginfo("Still waiting for robot_description... ({:.1f}s)".format(time.time() - start_time))
                
                if not rospy.has_param('/robot_description'):
                    rospy.logerr("Timed out waiting for robot_description parameter!")
                else:
                    rospy.loginfo("robot_description parameter found after {:.1f}s".format(time.time() - start_time))
            else:
                rospy.loginfo("Robot description already exists on parameter server.")
            
            # Give some time for the simulation to start up
            time.sleep(5.0)
            
            # Launch MoveIt for motion planning if needed
            if not self.check_if_move_group_running():
                rospy.loginfo("Launching niryo_one_moveit_config move_group.launch")
                self.moveit_process = subprocess.Popen(
                    ["roslaunch", "niryo_one_moveit_config", "move_group.launch"],
                    stdout=subprocess.PIPE, stderr=subprocess.STDOUT
                )
                self.start_process_output_monitor(self.moveit_process, "MoveIt")
            else:
                rospy.loginfo("MoveIt move_group already running.")
            
        except Exception as e:
            rospy.logerr("Failed to start required processes: {}".format(e))
            raise
            
    def check_if_move_group_running(self):
        """Check if MoveIt move_group is already running"""
        try:
            # Check if move_group node exists by looking for a common move_group parameter
            return rospy.has_param('/move_group')
        except:
            return False
    
    def start_process_output_monitor(self, process, name):
        """Start a thread to monitor and print process output for debugging"""
        def monitor_output():
            rospy.loginfo("Starting output monitor for {} process".format(name))
            for line in iter(process.stdout.readline, b''):
                line_str = line.decode('utf-8').strip()
                if line_str:
                    rospy.loginfo("{} output: {}".format(name, line_str))
            rospy.loginfo("{} process output monitor finished".format(name))
        
        # Start the monitor in a separate thread
        monitor_thread = threading.Thread(target=monitor_output)
        monitor_thread.daemon = True
        monitor_thread.start()
    
    def move_to_pose(self, pose, description, max_attempts=3):
        """Moves the robot to a specific pose with better error handling and retry logic."""
        rospy.loginfo("Moving to: {}".format(description))
        
        # Print current pose for debugging
        try:
            current_pose = self.move_group.get_current_pose().pose
            rospy.loginfo("Current position: x={:.3f}, y={:.3f}, z={:.3f}".format(
                current_pose.position.x, 
                current_pose.position.y, 
                current_pose.position.z))
            rospy.loginfo("Target position: x={:.3f}, y={:.3f}, z={:.3f}".format(
                pose.position.x, 
                pose.position.y, 
                pose.position.z))
                
            # Calculate and log distance
            distance = ((pose.position.x - current_pose.position.x)**2 + 
                       (pose.position.y - current_pose.position.y)**2 + 
                       (pose.position.z - current_pose.position.z)**2)**0.5
            rospy.loginfo("Distance to target: {:.3f} meters".format(distance))
        except Exception as e:
            rospy.logwarn("Couldn't get current pose for logging: {}".format(e))
        
        # If test tube is picked up, log that information
        if hasattr(self, 'test_tube_picked') and self.test_tube_picked:
            rospy.loginfo("Moving with test tube picked up - visualization should show test tube moving")
        
        # Use more relaxed constraints for movements after pickup
        is_post_pickup = hasattr(self, 'test_tube_picked') and self.test_tube_picked
        
        # Adjust planning time based on movement type and phase
        if "pick" in description.lower():
            self.move_group.set_planning_time(15.0)  # Increased planning time for pickup
        elif is_post_pickup:
            self.move_group.set_planning_time(20.0)  # Even more planning time after pickup
            # Use more relaxed goal tolerances after pickup
            self.move_group.set_goal_position_tolerance(0.01)  # 1cm position tolerance
            self.move_group.set_goal_orientation_tolerance(0.1)  # More orientation tolerance
        else:
            self.move_group.set_planning_time(10.0)
            # Reset to default tolerances
            self.move_group.set_goal_position_tolerance(0.0001)
            self.move_group.set_goal_orientation_tolerance(0.001)
        
        # Set velocity scaling lower for post-pickup moves
        if is_post_pickup:
            self.move_group.set_max_velocity_scaling_factor(0.1)  # Slower after pickup
        else:
            self.move_group.set_max_velocity_scaling_factor(0.2)
        
        attempt = 0
        success = False
        
        while attempt < max_attempts and not success and not rospy.is_shutdown():
            attempt += 1
            rospy.loginfo("Attempt {}/{} for {}".format(attempt, max_attempts, description))
            
            try:
                # Set the target pose
                self.move_group.set_pose_target(pose)
                
                # Plan and execute with timeout
                success = self.move_group.go(wait=True)
                
                if not success:
                    rospy.logwarn("Failed to move to {} (attempt {}/{})".format(
                        description, attempt, max_attempts))
                    # Try with a slightly different planning time on retry
                    self.move_group.set_planning_time(15.0 + attempt * 2.0)
                    time.sleep(0.5)  # Short delay before retry
            except Exception as e:
                rospy.logerr("Exception during move to {}: {}".format(description, e))
                time.sleep(1.0)  # Longer delay after exception
        
        # Always stop to ensure no residual movement
        self.move_group.stop()
        
        # Clear targets
        self.move_group.clear_pose_targets()
        
        # Log result
        if success:
            rospy.loginfo("Successfully moved to {}".format(description))
            # Verify position
            try:
                achieved_pose = self.move_group.get_current_pose().pose
                rospy.loginfo("Achieved position: x={:.3f}, y={:.3f}, z={:.3f}".format(
                    achieved_pose.position.x, 
                    achieved_pose.position.y, 
                    achieved_pose.position.z))
            except Exception as e:
                rospy.logwarn("Couldn't verify achieved position: {}".format(e))
        else:
            rospy.logerr("Failed to move to {} after {} attempts".format(description, max_attempts))
        
        # Short delay to allow stabilization
        time.sleep(0.8)
        
        return success
    
    def move_to_joint_state(self, joint_goal, description):
        """Move to a specific joint state configuration with improved error handling"""
        rospy.loginfo("Moving to: {}".format(description))
        
        try:
            # Set planning time for this specific movement
            self.move_group.set_planning_time(5.0)
            
            # Print current joint values for debugging
            current_joints = self.move_group.get_current_joint_values()
            rospy.loginfo("Current joint values: {}".format(current_joints))
            rospy.loginfo("Target joint values: {}".format(joint_goal))
            
            # Execute the movement with timeout
            success = self.move_group.go(joint_goal, wait=True)
            
            # Calling stop() ensures that there is no residual movement
            self.move_group.stop()
            
            if success:
                rospy.loginfo("Joint move to {} completed successfully".format(description))
            else:
                rospy.logwarn("Joint move to {} failed".format(description))
                
            # For verification:
            new_joints = self.move_group.get_current_joint_values()
            rospy.loginfo("New joint values after move: {}".format(new_joints))
            
            # Short delay to allow visualization to update
            time.sleep(0.8)
            
            return success
        except Exception as e:
            rospy.logerr("Exception during joint move to {}: {}".format(description, e))
            return False
    
    def create_waypoints(self):
        """Creates all waypoints for the movement sequence with adjusted positions"""
        home_pose = self.move_group.get_current_pose().pose
        
        # Define key movement waypoints
        approach_pose = Pose()
        approach_pose.position.x = 0.2
        approach_pose.position.y = 0
        approach_pose.position.z = 0.2
        approach_pose.orientation = home_pose.orientation
        
        # Add pre-pick position for smoother transition
        pre_pick_pose = Pose()
        pre_pick_pose.position.x = 0.2
        pre_pick_pose.position.y = 0
        pre_pick_pose.position.z = 0.1  # Intermediate height
        pre_pick_pose.orientation = home_pose.orientation
        
        pick_pose = Pose()
        pick_pose.position.x = 0.2
        pick_pose.position.y = 0
        pick_pose.position.z = 0.05  # Increased from 0.03 to 0.05 for better clearance
        pick_pose.orientation = home_pose.orientation

        # Add an intermediate lifting position for smoother transition
        intermediate_lift_pose = Pose()
        intermediate_lift_pose.position.x = pick_pose.position.x
        intermediate_lift_pose.position.y = pick_pose.position.y
        intermediate_lift_pose.position.z = 0.1  # Halfway to the lift position
        intermediate_lift_pose.orientation = pick_pose.orientation
        
        lift_pose = Pose()
        lift_pose.position.x = pick_pose.position.x
        lift_pose.position.y = pick_pose.position.y
        lift_pose.position.z = 0.2
        lift_pose.orientation = pick_pose.orientation

        # Add intermediate step to cup position
        intermediate_to_cup_pose = Pose()
        intermediate_to_cup_pose.position.x = 0.1
        intermediate_to_cup_pose.position.y = 0.15
        intermediate_to_cup_pose.position.z = 0.2
        intermediate_to_cup_pose.orientation = lift_pose.orientation
        
        move_to_cup_pose = Pose()
        move_to_cup_pose.position.x = 0
        move_to_cup_pose.position.y = 0.3
        move_to_cup_pose.position.z = 0.15
        move_to_cup_pose.orientation = lift_pose.orientation

        # Pour pose with proper orientation
        pour_pose = Pose()
        pour_pose.position = move_to_cup_pose.position
        # Create a slightly less extreme pour angle
        pour_pose.orientation.x = 0.5  # Reduced from 0.707 for a less extreme angle
        pour_pose.orientation.y = 0
        pour_pose.orientation.z = 0
        pour_pose.orientation.w = 0.866  # Adjusted to maintain normalized quaternion
        
        return approach_pose, pre_pick_pose, pick_pose, intermediate_lift_pose, lift_pose, intermediate_to_cup_pose, move_to_cup_pose, pour_pose
    
    def get_home_joint_values(self):
        """Get the home position joint values"""
        # This is a typical home position for Niryo One
        joint_values = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        return joint_values
    
    def run_movement_sequence(self):
        """Executes the full movement sequence with improved error handling"""
        rospy.loginfo("Starting simple movement sequence...")
        
        # Get waypoints
        (approach_pose, pre_pick_pose, pick_pose, intermediate_lift_pose, lift_pose, 
         intermediate_to_cup_pose, move_to_cup_pose, pour_pose) = self.create_waypoints()
        
        # Get home joint values
        home_joint_values = self.get_home_joint_values()
        
        # Loop the movement sequence twice
        for loop_count in range(2):
            rospy.loginfo("Starting movement loop {} of 2".format(loop_count + 1))
            
            # Move to home position first
            rospy.loginfo("Moving to home position for consistency")
            if not self.move_to_joint_state(home_joint_values, "Home position"):
                rospy.logerr("Failed to move to home position, aborting sequence")
                return
            
            # Move to approach position
            if not self.move_to_pose(approach_pose, "Approach test tube"):
                rospy.logerr("Failed to approach test tube")
                return
            
            # Move to pre-pick position
            if not self.move_to_pose(pre_pick_pose, "Pre-pick position"):
                rospy.logerr("Failed to reach pre-pick position")
                return
            
            # Move to pick position
            if not self.move_to_pose(pick_pose, "Pick position"):
                rospy.logerr("Failed to reach pick position")
                return
            
            # Try to pick up the test tube, but continue even if visualization fails
            try:
                pickup_successful = self.pick_test_tube()
                rospy.loginfo("Pickup function returned: {}".format(pickup_successful))
            except Exception as e:
                rospy.logwarn("Visualization error during pickup: {}. Continuing with movement sequence.".format(e))
                # Set the pickup flag anyway to ensure visualization continues if possible
                self.test_tube_picked = True
            
            # Continue with movement regardless of pickup visualization success
            rospy.loginfo("Waiting briefly before continuing to lift...")
            time.sleep(0.5)  # Reduced from 1.0 to 0.5 seconds
            
            # Continue after pickup with explicit debug output
            rospy.loginfo("==== CONTINUING TO LIFT AFTER PICKUP ====")
            
            # Forcefully restart the visualization thread if it's stuck
            try:
                if hasattr(self, 'marker_thread') and not self.marker_thread.is_alive():
                    rospy.logwarn("Marker thread died, restarting it...")
                    self.marker_thread = threading.Thread(target=self.publish_markers_continuously)
                    self.marker_thread.daemon = True
                    self.marker_thread.start()
            except Exception as thread_error:
                rospy.logwarn("Error checking marker thread: {}".format(thread_error))
            
            # Move to intermediate lift position
            rospy.loginfo("Attempting to move to intermediate lift position...")
            if not self.move_to_pose(intermediate_lift_pose, "Intermediate lift"):
                rospy.logerr("Failed intermediate lift, trying direct lift")
                # Try direct lift if intermediate fails
                rospy.loginfo("Attempting direct lift as fallback...")
                if not self.move_to_pose(lift_pose, "Direct lift"):
                    rospy.logerr("Failed to lift test tube")
                    try:
                        self.reset_test_tube()
                    except:
                        pass
                    return
                else:
                    rospy.loginfo("Direct lift successful as fallback")
            else:
                rospy.loginfo("Intermediate lift successful")
            
            # Move to lift position
            if not self.move_to_pose(lift_pose, "Lift test tube"):
                rospy.logerr("Failed to complete lift")
                try:
                    self.reset_test_tube()
                except:
                    pass
                return
            
            # Move to intermediate cup position
            if not self.move_to_pose(intermediate_to_cup_pose, "Move toward cup"):
                rospy.logerr("Failed to move toward cup, trying direct approach")
                # Try direct approach if intermediate fails
                if not self.move_to_pose(move_to_cup_pose, "Direct to cup"):
                    rospy.logerr("Failed to reach cup")
                    try:
                        self.reset_test_tube()
                    except:
                        pass
                    return
            
            # Move to cup position
            if not self.move_to_pose(move_to_cup_pose, "Move to cup"):
                rospy.logerr("Failed to reach cup")
                try:
                    self.reset_test_tube()
                except:
                    pass
                return
            
            # Pour into cup
            if not self.move_to_pose(pour_pose, "Pour test tube"):
                rospy.logerr("Failed to pour")
            else:
                # Simulate pouring with a slight delay
                time.sleep(2.0)
            
            # Return to a safe position
            if not self.move_to_pose(lift_pose, "Return to vertical position"):
                rospy.logwarn("Failed to return to vertical, trying home position")
                self.move_to_joint_state(home_joint_values, "Emergency return to home")
                try:
                    self.reset_test_tube()
                except:
                    pass
                return
            
            # Move back to the starting position to place the test tube back gently
            rospy.loginfo("Moving back to approach position")
            if not self.move_to_pose(approach_pose, "Return to approach position"):
                rospy.logwarn("Failed to return to approach position, trying to reset anyway")
            
            # Move down to place the test tube
            rospy.loginfo("Moving down to place the test tube")
            if not self.move_to_pose(pre_pick_pose, "Return to pre-place position"):
                rospy.logwarn("Failed to reach pre-place position, trying to reset anyway")
                
            # Move to final placement position
            rospy.loginfo("Placing test tube back")
            if not self.move_to_pose(pick_pose, "Place test tube back"):
                rospy.logwarn("Failed to reach place position, trying to reset anyway")
                
            # Reset test tube after completion
            try:
                self.place_test_tube_gently()
            except Exception as e:
                rospy.logwarn("Error placing test tube: {}".format(e))
            
            # If this isn't the last loop, wait a bit before starting the next one
            if loop_count < 1:
                rospy.loginfo("Loop {} completed. Waiting before starting next loop...".format(loop_count + 1))
                time.sleep(3.0)
        
        # Return to home after completing all loops
        self.move_to_joint_state(home_joint_values, "Return to home")
        
        rospy.loginfo("Movement sequence completed successfully")
    
    def shutdown(self):
        """Shut down cleanly"""
        rospy.loginfo("Movement sequence completed.")
        moveit_commander.roscpp_shutdown()
        
        # Terminate the processes we started
        if hasattr(self, 'simulation_process'):
            self.simulation_process.terminate()
        if hasattr(self, 'moveit_process'):
            self.moveit_process.terminate()
            
        rospy.loginfo("All processes terminated.")

    def pick_test_tube(self):
        """Handle the picking of the test tube with visualization"""
        # Log before picking
        rospy.loginfo("Simulating picking up test tube")
        
        # Stop any ongoing movement completely
        self.move_group.stop()
        time.sleep(0.5)  # Ensure robot is stable before picking
        
        # Simulate gripper closing
        rospy.loginfo("Simulating gripper closing")
        time.sleep(0.5)
        
        try:
            # Set flag first before attempting to update position - do this without lock
            self.test_tube_picked = True
            rospy.loginfo("Test tube picked up - flag set")
            
            # Now try to update visualization, but don't let it block the sequence
            try:
                # Get current end effector pose first, before attempting to acquire lock
                current_pose = self.move_group.get_current_pose(self.end_effector_link).pose
                
                # Try to acquire lock non-blocking (Python 2 compatible)
                lock_acquired = False
                try:
                    lock_acquired = self.marker_mutex.acquire(False)  # Non-blocking acquire (Python 2 compatible)
                    if lock_acquired:
                        # Update the test tube marker with new position
                        self.update_test_tube_position(current_pose)
                        rospy.loginfo("Test tube position updated")
                finally:
                    # Always release the lock if acquired
                    if lock_acquired:
                        self.marker_mutex.release()
                        
                if not lock_acquired:
                    rospy.logwarn("Could not acquire marker mutex, will update visualization later")
            except Exception as e:
                rospy.logwarn("Failed to update test tube position: {}. Continuing with movement.".format(e))
            
            # Add a stabilization delay after pickup
            time.sleep(0.5)
            rospy.loginfo("Pickup complete, proceeding to next movement")
            
            # Explicitly verify test_tube_picked flag is still set
            rospy.loginfo("Test tube picked flag status: {}".format(self.test_tube_picked))
            
            return True
        except Exception as e:
            rospy.logerr("Error during pickup: {}".format(e))
            # Make sure the flag is set even if there was an error with visualization
            self.test_tube_picked = True
            rospy.loginfo("Set test_tube_picked=True despite error")
            return True  # Return True anyway to continue the sequence

    def reset_test_tube(self):
        """Reset the test tube to its original position on the table"""
        # Simulate gripper opening
        rospy.loginfo("Simulating gripper opening")
        
        # Make sure we're in a safe position before releasing
        try:
            current_pose = self.move_group.get_current_pose().pose
            if current_pose.position.z < 0.15:
                rospy.logwarn("Robot may be too low to safely release test tube, waiting...")
                time.sleep(1.0)  # Give more time to reach a safe height
        except Exception as e:
            rospy.logwarn("Could not verify position before release: {}".format(e))
        
        # Simulate the tube falling gently
        original_pose = Pose(Point(0.2, 0, -0.03), Quaternion(0, 0, 0, 1))
        intermediate_poses = []
        
        try:
            # Get current pose first
            lock_acquired = False
            try:
                lock_acquired = self.marker_mutex.acquire(False)
                if lock_acquired:
                    current_pose = copy.deepcopy(self.test_tube_marker.pose)
                    
                    # Create 5 intermediate poses for smooth animation
                    for i in range(1, 6):
                        fraction = float(i) / 5.0
                        intermediate_pose = Pose()
                        intermediate_pose.orientation = current_pose.orientation  # Keep orientation
                        
                        # Linearly interpolate position
                        intermediate_pose.position.x = current_pose.position.x * (1-fraction) + original_pose.position.x * fraction
                        intermediate_pose.position.y = current_pose.position.y * (1-fraction) + original_pose.position.y * fraction
                        intermediate_pose.position.z = current_pose.position.z * (1-fraction) + original_pose.position.z * fraction
                        
                        intermediate_poses.append(intermediate_pose)
            finally:
                if lock_acquired:
                    self.marker_mutex.release()
                    
            # Now animate the falling
            if intermediate_poses:
                for pose in intermediate_poses:
                    try:
                        with self.marker_mutex:
                            self.test_tube_marker.pose = pose
                            self.test_tube_marker.action = Marker.MODIFY
                            self.marker_pub.publish(self.test_tube_marker)
                        time.sleep(0.05)  # Short delay for animation
                    except Exception as e:
                        rospy.logwarn("Error during test tube animation: {}".format(e))
        except Exception as e:
            rospy.logwarn("Error preparing animation poses: {}".format(e))
            
        # Finally set the original position
        with self.marker_mutex:
            # Reset the flag before updating marker
            self.test_tube_picked = False
            
            # Reset the test tube marker
            self.test_tube_marker.pose = original_pose
            self.test_tube_marker.action = Marker.MODIFY
            self.marker_pub.publish(self.test_tube_marker)
            rospy.loginfo("Test tube reset to original position")

    def place_test_tube_gently(self):
        """Gently place the test tube back to its original position without dropping it"""
        # Simulate gripper opening
        rospy.loginfo("Simulating gripper opening to place test tube")
        
        # Original test tube position on the table
        original_pose = Pose(Point(0.2, 0, -0.03), Quaternion(0, 0, 0, 1))
        
        # Animate the gentle placement
        try:
            # Get current test tube pose
            lock_acquired = False
            current_pose = None
            
            try:
                lock_acquired = self.marker_mutex.acquire(False)
                if lock_acquired:
                    current_pose = copy.deepcopy(self.test_tube_marker.pose)
            finally:
                if lock_acquired:
                    self.marker_mutex.release()
            
            if current_pose:
                # Create small linear motion for gentle placement
                intermediate_poses = []
                for i in range(1, 4):
                    fraction = float(i) / 3.0
                    intermediate_pose = Pose()
                    intermediate_pose.orientation = current_pose.orientation
                    
                    # Very small vertical movement down
                    intermediate_pose.position.x = current_pose.position.x
                    intermediate_pose.position.y = current_pose.position.y
                    intermediate_pose.position.z = current_pose.position.z - (fraction * 0.02)  # Just move down slightly
                    
                    intermediate_poses.append(intermediate_pose)
                
                # Animate the slight downward motion
                for pose in intermediate_poses:
                    try:
                        with self.marker_mutex:
                            self.test_tube_marker.pose = pose
                            self.marker_pub.publish(self.test_tube_marker)
                        time.sleep(0.1)  # Slower for more gentle appearance
                    except Exception as e:
                        rospy.logwarn("Error during gentle placement animation: {}".format(e))
            
            # Set flag before final position update
            self.test_tube_picked = False
            
            # Final position update
            with self.marker_mutex:
                self.test_tube_marker.pose = original_pose
                self.test_tube_marker.action = Marker.MODIFY
                self.marker_pub.publish(self.test_tube_marker)
            
            rospy.loginfo("Test tube placed back gently")
            
        except Exception as e:
            rospy.logwarn("Error during gentle placement: {}".format(e))
            # Fallback to simple reset
            with self.marker_mutex:
                self.test_tube_picked = False
                self.test_tube_marker.pose = original_pose
                self.marker_pub.publish(self.test_tube_marker)


if __name__ == "__main__":
    try:
        mover = SimpleNiryoMover()
        rospy.loginfo("Starting simple movement sequence...")
        time.sleep(2)  # Make sure everything is initialized
        
        try:
            # Run the movement sequence in a try-except block to catch any visualization issues
            mover.run_movement_sequence()
        except Exception as e:
            rospy.logerr("Error during movement sequence: {}".format(e))
            import traceback
            rospy.logerr("Traceback: {}".format(traceback.format_exc()))
            
            # Even if there was an error in the sequence, try to clean up
            try:
                rospy.loginfo("Attempting to reset and return to home position...")
                if hasattr(mover, 'test_tube_picked') and mover.test_tube_picked:
                    mover.reset_test_tube()
                
                if hasattr(mover, 'move_group') and mover.move_group is not None:
                    home_joint_values = mover.get_home_joint_values()
                    mover.move_to_joint_state(home_joint_values, "Emergency return home")
            except Exception as cleanup_error:
                rospy.logerr("Error during cleanup: {}".format(cleanup_error))
        
        # Always attempt to shut down cleanly
        mover.shutdown()
        
    except rospy.ROSInterruptException:
        rospy.logerr("ROS Interrupted")
    except Exception as e:
        rospy.logerr("Error during initialization: {}".format(e))
        import traceback
        rospy.logerr("Traceback: {}".format(traceback.format_exc())) 