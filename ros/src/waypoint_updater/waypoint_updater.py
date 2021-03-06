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

LOOKAHEAD_WPS = 200 # Number of waypoints we will publish. You can change this number
STOP_BEFORE = 8


class WaypointUpdater(object):
    def __init__(self):
        rospy.init_node('waypoint_updater')

        rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)

        # TODO: Add a subscriber for /traffic_waypoint and /obstacle_waypoint below
        rospy.Subscriber('/traffic_waypoint', Int32, self.traffic_cb)


        self.final_waypoints_pub = rospy.Publisher('final_waypoints', Lane, queue_size=1)

        # TODO: Add other member variables you need below
        self.max_velocity = None
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
                rospy.loginfo("Pose,0,%0.3lf,%0.3lf", self.pose.position.x, self.pose.position.y)
                index = self.kdTree.query([self.pose.position.x, self.pose.position.y], 1)[1]

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

                for i in range(len(lane.waypoints)):
                    rospy.loginfo("Velocity,%d,%0.3f", i, self.get_waypoint_velocity(lane.waypoints[i]))
                
                rospy.loginfo("waypoints,From,%d,To,%d", index, index + LOOKAHEAD_WPS)
                if not self.traffic_waypoint == None:
                    rospy.loginfo("traffic_waypoint,%d", self.traffic_waypoint)
                    if not self.traffic_waypoint == -1:
                        if (self.traffic_waypoint < index + LOOKAHEAD_WPS):
                            rospy.loginfo("handlestop")
                            stop_waypoint = self.traffic_waypoint - STOP_BEFORE
                            org_velocity = lane.waypoints[0].twist.twist.linear.x
                            stop_index = stop_waypoint - index

                            for i in range(len(lane.waypoints)):
                                p = Waypoint()
                                p.pose = lane.waypoints[i].pose
                                if i >= stop_index:
                                    velocity = 0
                                else:
                                    dist = self.distance(lane.waypoints, i, stop_index)
                                    v = math.sqrt(2 * 3 * dist)
                                    velocity = min(v, self.get_waypoint_velocity(lane.waypoints[i]))

                                p.twist.twist.linear.x = velocity 
                                lane.waypoints[i] = p



                # if len(lane.waypoints) < LOOKAHEAD_WPS:
                #     lane.waypoints.extend(self.waypoints[:LOOKAHEAD_WPS - len(lane.waypoints)])

                self.final_waypoints_pub.publish(lane)
            rate.sleep()

    def pose_cb(self, msg):
        # TODO: Implement
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        # TODO: Implement
        self.waypoints = waypoints.waypoints
        self.waypoints_xy = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
        self.kdTree = KDTree(self.waypoints_xy)

    def traffic_cb(self, msg):
        # TODO: Callback for /traffic_waypoint message. Implement
        self.traffic_waypoint = msg.data

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

    def kmph2mps(self, velocity_kmph):
        return (velocity_kmph * 1000.) / (60. * 60.)


if __name__ == '__main__':
    try:
        WaypointUpdater()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start waypoint updater node.')
