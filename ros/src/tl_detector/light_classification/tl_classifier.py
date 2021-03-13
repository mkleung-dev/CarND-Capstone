from styx_msgs.msg import TrafficLight
import tensorflow as tf
from keras.models import load_model
import numpy as np
import os
import cv2
from keras import backend as K

import rospy


class TLClassifier(object):
    def __init__(self):
        self.buffer_state = TrafficLight.UNKNOWN
        self.ans_map = [TrafficLight.GREEN, TrafficLight.UNKNOWN, TrafficLight.RED, TrafficLight.YELLOW]
        
        self.session = tf.Session()
        self.graph = tf.get_default_graph()
        with self.graph.as_default():
            with self.session.as_default():
                self.model = load_model(os.getcwd() + '/light_classification/CNN.h5')

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
       
        image = cv2.resize(image, (300, 400))
            
        with self.graph.as_default():
            with self.session.as_default():
                temp = self.model.predict(np.asarray(image)[np.newaxis, :])
        
        self.buffer_state = self.ans_map[np.argmax(temp)]

        return self.buffer_state
