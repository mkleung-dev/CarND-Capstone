#!/usr/bin/env python

import rospy
from geometry_msgs.msg import PoseStamped
from styx_msgs.msg import Lane, Waypoint
from scipy.spatial import KDTree
from std_msgs.msg import Int32
import numpy as np
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

LOOKAHEAD_WPS = 50 # Number of waypoints we will publish. Set to 50 so that it can run in Udacity workspace.
MAX_DECELERATION = 5
STOP_BEFORE = 8


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        # Subscriber
        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)
        
        # Publisher
        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # Member Variable
        self.pose = None
        self.waypoints= None
        self.waypoints_xy = None
        self.kdTree = None
        self.traffic_waypoint = None

        self.loop()

    def loop(self):
        rate = rospy.Rate(50)
        while not rospy.is_shutdown():
            if not None in [self.pose, self.waypoints_xy, self.kdTree]:
                # Find the waypoint closest to the current position.
                index = self.kdTree.query([self.pose.position.x, self.pose.position.y], 1)[1]

                # Find the cloest waypoint in front of the current position.
                curr_wpt = np.array(self.waypoints_xy[index])
                prev_wpt = np.array(self.waypoints_xy[index-1])
                curr = np.array([self.pose.position.x, self.pose.position.y])
                v1 = curr_wpt - prev_wpt
                v2 = curr - curr_wpt
                dot_product = np.dot(v1, v2)
                if dot_product > 0:
                    index = (index + 1) % len(self.waypoints)

                lane = Lane()
                lane.waypoints = self.waypoints[index: index + LOOKAHEAD_WPS]

                # Decelerate to stop if there is red traffic light.
                if not self.traffic_waypoint == None:
                    if not self.traffic_waypoint == -1:
                        if (self.traffic_waypoint < index + LOOKAHEAD_WPS):
                            stop_waypoint = self.traffic_waypoint - STOP_BEFORE
                            # Index in the computed waypoints.
                            stop_index = stop_waypoint - index

                            # Set the velocity in the waypoints to stop the vehicle.
                            dist = 0
                            for i in reversed(range(len(lane.waypoints))):
                                p = Waypoint()
                                p.pose = lane.waypoints[i].pose
                                if i >= stop_index:
                                    velocity = 0
                                else:
                                    dist = dist + self.distance(lane.waypoints, i, i + 1)
                                    velocity = math.sqrt(2 * MAX_DECELERATION * dist)
                                    velocity = min(velocity, self.get_waypoint_velocity(lane.waypoints[i]))

                                p.twist.twist.linear.x = velocity 
                                lane.waypoints[i] = p
                
                self.final_waypoints_pub.publish(lane)
            rate.sleep()

    def pose_cb(self, msg):
        # Callback for /current_pose message.
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        # Callback for /base_waypoints message.
        self.waypoints = waypoints.waypoints
        self.waypoints_xy = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
        self.kdTree = KDTree(self.waypoints_xy)

    def traffic_cb(self, msg):
        # Callback for /traffic_waypoint message.
        self.traffic_waypoint = msg.data

    def obstacle_cb(self, msg):
        # Callback for /obstacle_waypoint message.
        pass

    def get_waypoint_velocity(self, waypoint):
        # Get velocity for the selected waypoint
        return waypoint.twist.twist.linear.x

    def set_waypoint_velocity(self, waypoints, waypoint, velocity):
        # Set velocity for the selected waypoint
        waypoints[waypoint].twist.twist.linear.x = velocity

    def distance(self, waypoints, wp1, wp2):
        # Distance between 2 waypoints
        dist = 0
        dl = lambda a, b: math.sqrt((a.x-b.x)**2 + (a.y-b.y)**2  + (a.z-b.z)**2)
        for i in range(wp1, wp2+1):
            dist += dl(waypoints[wp1].pose.pose.position, waypoints[i].pose.pose.position)
            wp1 = i
        return dist

    def kmph2mps(self, velocity_kmph):
        # Convert unit from kilometer per hour to meter per second
        return (velocity_kmph * 1000.) / (60. * 60.)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
