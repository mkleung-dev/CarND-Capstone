#!/usr/bin/env python
import rospy
from std_msgs.msg import Int32
from geometry_msgs.msg import PoseStamped, Pose
from styx_msgs.msg import TrafficLightArray, TrafficLight
from styx_msgs.msg import Lane
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from light_classification.tl_classifier import TLClassifier
import cv2
import yaml
import math

import time
from scipy.spatial import KDTree
import numpy as np

STATE_COUNT_THRESHOLD = 3
CLASSIFY_PERIOD = 4

class TLDetector(object):
    def __init__(self):
        rospy.init_node('tl_detector')

        # Load traffic light config
        config_string = rospy.get_param("/traffic_light_config")
        self.config = yaml.load(config_string)

        # Subscriber
        sub1 = rospy.Subscriber('/current_pose', PoseStamped, self.pose_cb)
        sub2 = rospy.Subscriber('/base_waypoints', Lane, self.waypoints_cb)
        '''
        /vehicle/traffic_lights provides you with the location of the traffic light in 3D map space and
        helps you acquire an accurate ground truth data source for the traffic light
        classifier by sending the current color state of all traffic lights in the
        simulator. When testing on the vehicle, the color state will not be available. You'll need to
        rely on the position of the light and the camera image to predict it.
        '''
        sub3 = rospy.Subscriber('/vehicle/traffic_lights', TrafficLightArray, self.traffic_cb)
        sub6 = rospy.Subscriber('/image_color', Image, self.image_cb)

        self.upcoming_red_light_pub = rospy.Publisher('/traffic_waypoint', Int32, queue_size=1)

        # Current pose
        self.pose = None
        # Image from the car
        self.camera_image = None
        # All traffic light signals (Not used if there is traffic classifier).
        self.lights = []

        # For waypoints and their query.
        self.waypoints= None
        self.waypoints_xy = None
        self.kdTree = None

        # Objecj to convert msg to OpenCV image
        self.bridge = CvBridge()
        # Traffic light classifier
        self.light_classifier = TLClassifier()

        # Variable for changing state only if the it occurs for consecutive `STATE_COUNT_THRESHOLD` number
        self.state = TrafficLight.UNKNOWN
        self.last_state = TrafficLight.UNKNOWN
        self.last_wp = -1
        self.state_count = 0
        
        # Member variable for using the image from the camera for every `CLASSIFY_PERIOD` times.
        self.classify_count = 0
        self.classify_state = TrafficLight.UNKNOWN

        """ Used to save image for training classifier.
        self.save_count = 0
        self.prev_save_diff = 0.3
        self.prev_save_state = None
        """
        rospy.spin()

    def pose_cb(self, msg):
        # Callback for /current_pose message.
        self.pose = msg.pose

    def waypoints_cb(self, waypoints):
        # Callback for /base_waypoints message.
        self.waypoints = waypoints.waypoints
        self.waypoints_xy = [[waypoint.pose.pose.position.x, waypoint.pose.pose.position.y] for waypoint in self.waypoints]
        self.kdTree = KDTree(self.waypoints_xy)

    def traffic_cb(self, msg):
        # Callback for /base_waypoints message.
        self.lights = msg.lights

    def image_cb(self, msg):
        # Callback for /vehicle/traffic_lights message.
        """Identifies red lights in the incoming camera image and publishes the index
            of the waypoint closest to the red light's stop line to /traffic_waypoint

        Args:
            msg (Image): image from car-mounted camera

        """
        self.camera_image = msg
        light_wp, state = self.process_traffic_lights()

        # Change state only if the it occurs for consecutive `STATE_COUNT_THRESHOLD` number
        if self.state != state:
            self.state_count = 0
            self.state = state
        elif self.state_count >= STATE_COUNT_THRESHOLD:
            self.last_state = self.state
            light_wp = light_wp if state == TrafficLight.RED else -1
            self.last_wp = light_wp
            self.upcoming_red_light_pub.publish(Int32(light_wp))
        else:
            self.upcoming_red_light_pub.publish(Int32(self.last_wp))
        self.state_count += 1

    def get_closest_waypoint(self, x, y):
        """Identifies the closest path waypoint to the given position
        Args:
            pose (Pose): position to match a waypoint to
        Returns:
            int: index of the closest waypoint in self.waypoints
        """
        index = self.kdTree.query([x, y], 1)[1]

        return index

    def get_light_state(self, light, neg_distance, distance):
        """Determines the current color of the traffic light
        Args:
            light (TrafficLight): light to classify
            neg_distance: Distance after pass the stop line (For saving image for training) 
            distance: Distance for the next stop line (For saving image for training)
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """

        """ Used to save image for training classifier.
        t = time.localtime()
        current_time = time.strftime("%H%M%S", t)
        if ((not (self.prev_save_diff == distance and self.prev_save_state == light.state)) and self.save_count > 0):
            if distance <= 120:
                cv2.imwrite('/home/student/Developer/CarND/CarND-Capstone/train_image_{:02d}/{}/{}_{:08d}_waypoint_{:05.0f}.jpg'.format(0, light.state, current_time, self.save_count, distance), cv_image)
            elif distance > 300 and neg_distance > 30:
                cv2.imwrite('/home/student/Developer/CarND/CarND-Capstone/train_image_{:02d}/{}/{}_{:08d}_waypoint_{:05.0f}.jpg'.format(0, 9999, current_time, self.save_count, distance), cv_image)
        self.prev_save_diff = distance
        self.prev_save_state = light.state
        self.save_count = self.save_count  + 1
        """

        # Use the image from the camera for every `CLASSIFY_PERIOD` (4) times.
        if self.classify_count == 0:
            # Use when there are no traffic light classifier.
            # self.classify_state = light.state
            
            cv_image = self.bridge.imgmsg_to_cv2(self.camera_image, "rgb8")
            self.classify_state = self.light_classifier.get_classification(cv_image)
        self.classify_count = (self.classify_count + 1) % CLASSIFY_PERIOD

        #Get classification
        return self.classify_state

    def process_traffic_lights(self):
        """Finds closest visible traffic light, if one exists, and determines its
            location and color
        Returns:
            int: index of waypoint closes to the upcoming stop line for a traffic light (-1 if none exists)
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
        light = None
        light_wp = None

        # List of positions that correspond to the line to stop in front of for a given intersection
        stop_line_positions = self.config['stop_line_positions']
        if (self.pose):
            car_index = self.get_closest_waypoint(self.pose.position.x, self.pose.position.y)

        min_diff = len(self.waypoints)
        distance = 1000

        min_neg_diff = len(self.waypoints)
        neg_distance = 0
        
        # Find the closest visible traffic light if one exists
        for i in range(len(stop_line_positions)):
            line_pos = stop_line_positions[i]

            line_index = self.get_closest_waypoint(line_pos[0], line_pos[1])
            temp_index_diff = line_index - car_index

            # Compute next stop line distance and the next traffic light
            if (temp_index_diff > 0):
                if temp_index_diff < min_diff:
                    distance = math.sqrt((self.pose.position.x - line_pos[0]) ** 2 + (self.pose.position.y - line_pos[1]) ** 2)
                    min_diff = temp_index_diff
                    light_wp = line_index
                    pre_i = i - 1
                    light = self.lights[i]

            # Compute previous stop line distance
            if (temp_index_diff <= 0):
                if -temp_index_diff < min_neg_diff:
                    neg_distance = math.sqrt((self.pose.position.x - line_pos[0]) ** 2 + (self.pose.position.y - line_pos[1]) ** 2)
                    min_neg_diff = -temp_index_diff

        if light:
            state = self.get_light_state(light, neg_distance, distance)
            return light_wp, state
        return -1, TrafficLight.UNKNOWN

if __name__ == '__main__':
    try:
        TLDetector()
    except rospy.ROSInterruptException:
        rospy.logerr('Could not start traffic node.')
