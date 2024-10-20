#!/usr/bin/python3

"""
This is the main entry point for the particle filter exercise node. It
subscribes to laser, map, and odometry and creates an instance of
pf.PFLocaliser() to do the localisation.
"""

import rclpy
from rclpy.node import Node
from rclpy.wait_for_message import wait_for_message
from rclpy.qos import QoSProfile, QoSDurabilityPolicy

import pf_localisation.pf
from pf_localisation.util import *

from geometry_msgs.msg import ( PoseStamped, PoseWithCovarianceStamped,
                                PoseArray, Quaternion )
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, Odometry
import pf_localisation

import sys
from copy import deepcopy

class ParticleFilterLocalisationNode(Node):
    def __init__(self):
        super().__init__("pf_localisation")
        # ----- Minimum change (m/radians) before publishing new particle cloud and pose
        self._PUBLISH_DELTA = self.get_parameter_or("publish_delta", 0.1)
        # https://roboticsbackend.com/rclpy-params-tutorial-get-set-ros2-params-with-python/
        
        self._particle_filter = pf_localisation.pf.PFLocaliser(self.get_logger(), self.get_clock())

        self._latest_scan = None
        self._last_published_pose = None
        self._initial_pose_received = False

        self._pose_publisher = self.create_publisher(PoseStamped, "/estimatedpose", 10)
        self._amcl_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, "/amcl_pose", 10)
        self._cloud_publisher = self.create_publisher(PoseArray, "/particlecloud", 10)
        self._tf_publisher = self.create_publisher(TFMessage, "/tf", 10)

        latching_qos = QoSProfile(depth=1,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL)

        self.get_logger().info("Waiting for a map...")
        try:
            success, ocuccupancy_map = wait_for_message(
                OccupancyGrid, self, "/map", qos_profile=latching_qos, time_to_wait=20)  # If no wait_for_message, then use a timer with the callback.
        except Exception as e:
            self.get_logger().error(e)
            self.get_logger().error("""Problem getting a map. Check that you have an activated map_server
run: ros2 run nav2_map_server map_server --ros-args -p yaml_filename:=<path_to_your_map_yaml_file>
configure: ros2 lifecycle set map_server configure
activate: ros2 lifecycle set map_server activate
""")
            sys.exit(1)
        self.get_logger().info("Map received. %d X %d, %f px/m." %
                      (ocuccupancy_map.info.width, ocuccupancy_map.info.height,
                       ocuccupancy_map.info.resolution))
        self._particle_filter.set_map(ocuccupancy_map)
        
        self._laser_subscriber = self.create_subscription(LaserScan, "/base_scan",
                                                  self._laser_callback, 1)
        self._initial_pose_subscriber = self.create_subscription(PoseWithCovarianceStamped, 
                                                         "/initialpose",
                                                         self._initial_pose_callback, 1)
        self._odometry_subscriber = self.create_subscription(Odometry, "/odom", 
                                                     self._odometry_callback, 1)

    def _initial_pose_callback(self, pose):
        """ called when RViz sends a user supplied initial pose estimate """
        self._particle_filter.set_initial_pose(pose)
        self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        self._initial_pose_received = True
        self._cloud_publisher.publish(self._particle_filter.particlecloud)

    def _odometry_callback(self, odometry):
        """
        Odometry received. If the filter is initialised then execute
        a filter predict step with odeometry followed by an update step using
        the latest laser.
        """
        if self._initial_pose_received:
            t_odom = self._particle_filter.predict_from_odometry(odometry)
            t_filter = self._particle_filter.update_filter(self._latest_scan)
            if t_odom + t_filter > 0.1:
                self.get_logger().warning("Filter cycle overran timeslot")
                self.get_logger().info("Odometry update: %fs"%t_odom)
                self.get_logger().info("Particle update: %fs"%t_filter)
    
    def _laser_callback(self, scan):
        """
        Laser received. Store a ref to the latest scan. If robot has moved
        much, republish the latest pose to update RViz
        """
        self._latest_scan = scan
        if self._initial_pose_received:
            if  self._sufficientMovementDetected(self._particle_filter.estimatedpose):
                # ----- Publish the new pose
                self._amcl_pose_publisher.publish(self._particle_filter.estimatedpose)
                estimatedpose =  PoseStamped()
                estimatedpose.pose = self._particle_filter.estimatedpose.pose.pose
                estimatedpose.header.stamp = self._particle_filter.estimatedpose.header.stamp
                estimatedpose.header.frame_id = "map"
                self._pose_publisher.publish(estimatedpose)
                
                # ----- Update record of previously-published pose
                self._last_published_pose = deepcopy(self._particle_filter.estimatedpose)
        
                # ----- Get updated particle cloud and publish it
                self._cloud_publisher.publish(self._particle_filter.particlecloud)
        
                # ----- Get updated transform and publish it
                self._tf_publisher.publish(self._particle_filter.tf_message)
    
    def _sufficientMovementDetected(self, latest_pose):
        """
        Compares the last published pose to the current pose. Returns true
        if movement is more the self._PUBLISH_DELTA
        """
        # ----- Check that minimum required amount of movement has occurred before re-publishing
        latest_x = latest_pose.pose.pose.position.x
        latest_y = latest_pose.pose.pose.position.y
        prev_x = self._last_published_pose.pose.pose.position.x
        prev_y = self._last_published_pose.pose.pose.position.y
        location_delta = abs(latest_x - prev_x) + abs(latest_y - prev_y)

        # ----- Also check for difference in orientation: Take a zero-quaternion,
        # ----- rotate forward by latest_rot, and rotate back by prev_rot, to get difference)
        latest_rot = latest_pose.pose.pose.orientation
        prev_rot = self._last_published_pose.pose.pose.orientation

        q = rotateQuaternion(Quaternion(w=1.0),
                             getHeading(latest_rot))   # Rotate forward
        q = rotateQuaternion(q, -getHeading(prev_rot)) # Rotate backward
        heading_delta = abs(getHeading(q))
        #self.get_logger().info("Moved by %f"%location_delta)
        return (location_delta > self._PUBLISH_DELTA or
                heading_delta > self._PUBLISH_DELTA)

def main(args=None):
    # --- Main Program  ---
    rclpy.init(args=args)
    node = ParticleFilterLocalisationNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
