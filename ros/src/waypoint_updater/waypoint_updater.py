#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from std_msgs.msg import Int32
from scipy.spatial import KDTree

import math

'''
This node will publish waypoints from the car's current position to some `x` distance ahead.

As mentioned in the doc, you should ideally first implement a version which does not care
about traffic lights or obstacles.

Once you have created dbw_node, you will update this node to use the status of traffic lights too.

Please note that our simulator also provides the exact location of traffic lights and their
current status in `/vehicle/traffic_lights` message. You can use this message to build this node
as well as to verify your TL classifier.

TODO (for Yousuf and Aaron): Stopline location for each traffic light.
'''

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. You can change this number
PUBLISHING_RATE = 50
MAX_DECEL = 0.5

class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        self.base_lane = None
        self.pose = None
        self.waypoints_2d = None
        self.waypoint_tree = None

        self.stopline_wp_idx = -1

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('./traffic_waypoint', Int32, self.traffic_cb)
        ## there is no /obstacle_waypoint topic

        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        self.loop() # a loop is used here to set the update frequency

    def loop(self):
        rate = rospy.Rate(PUBLISHING_RATE)
        while not rospy.is_shutdown():
            if self.pose and self.base_lane:
                # closest_waypoint_index = self.get_closest_waypoint_idx()
                # self.publish_waypoints(closest_waypoint_index)
                self.publish_waypoints()
            rate.sleep()

    def get_closest_waypoint_idx(self):
        x = self.pose.pose.position.x
        y = self.pose.pose.position.y
        # first 1 means only 1 item, [1] is the index of the closest coordinate
        closest_idx = self.waypoint_tree.query([x, y], 1)[1]

        closest_coord = self.waypoints_2d[closest_idx]
        prev_coord = self.waypoints_2d[closest_idx-1]

        closest_waypoint_vector = np.array(closest_coord)
        previous_waypoint_vector = np.array(prev_coord)
        current_position_vector = np.array([x, y])

        # this calculates whether the closest waypoint is in front of the car or behind
        # if val > 0, prev -> closest -> current_pose, closest is behind
        # if val < 0, prev -> current_pose -> closest, closest is in front
        val = np.dot(closest_waypoint_vector - previous_waypoint_vector,
                    current_position_vector - closest_waypoint_vector)

        if val > 0:
            closest_idx = (closest_idx + 1) % len(self.waypoints_2d)
        return closest_idx

    def publish_waypoints(self):
        ## publish_waypoints(self, closest_idx):
        ##### without decelerating
        # lane = Lane()           #Lane() is a msg
        # lane.header = self.base_lane.header
        # # if [a, b] b is longer than the length of list, automatically the final item
        # lane.waypoints = self.base_lane.waypoints[closest_idx : closest_idx + LOOKAHEAD_WPS]
        # self.final_waypoints_pub.publish(lane)

        final_lane = self.generate_lane()
        self.final_waypoints_pub.publish(final_lane)

    def generate_lane(self):
        lane = Lane()

        closest_idx = self.get_closest_waypoint_idx()
        farthest_idx = closest_idx + LOOKAHEAD_WPS
        base_waypoints = self.base_lane.waypoints[closest_idx, farthest_idx]

        # if the stop line is too far away, use base_waypoints
        if self.stopline_wp_idx == -1 or (self.stopline_wp_idx >= farthest_idx):
            lane.waypoints = base_waypoints

        #Otherwise set up the decel waypoints
        else:
            lane.waypoints = self.decelerate_waypoints(base_waypoints, closest_idx)

        return lane

    def decelerate_waypoints(self, waypoints, closest_idx):
        temp_waypoints = []
        for i, wp in enumerate(waypoints):
            p = Waypoint()
            p.pose = wp.pose

            # -2 because want the car to stop before the stopline
            stop_idx = max(self.stopline_wp_idx - closest_idx - 2, 0)
            dist_to_stop = self.distance(waypoints, i, stop_idx)
            decel_vel = math.sqrt(2 * MAX_DECEL * dist_to_stop)
            if decel_vel < 1.0:
                decel_vel = 0.0

            # only use decel_vel if decel_vel < current velocity
            p.twist.twist.linear.x = min(decel_vel, wp.twist.twist.linear.x)
            temp_waypoints.append(p)

        return temp_waypoints

    def pose_cb(self, msg):
        self.pose = msg

    def waypoints_cb(self, waypoints):
        self.base_lane = waypoints
        # initialise waypoints_2d
        if not self.waypoints_2d:
            # KDTree is an algotithm to look for closest waypoint in space efficiently
            # it requires a special format
            self.waypoints_2d = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in waypoints.waypoints]
            self.waypoint_tree = KDTree(self.waypoints_2d)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.stopline_wp_idx = msg.data

    def obstacle_cb(self, msg):
        # TODO: Callback for /obstacle_waypoint message. We will implement it later
        pass

    def get_waypoint_velocity(self, waypoint):
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
