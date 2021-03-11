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
        #TODO load classifier
        #vgg16 = VGG16()
        self.buffer_state = TrafficLight.UNKNOWN
        self.count = 0
        self.ans_map = [TrafficLight.GREEN, TrafficLight.UNKNOWN, TrafficLight.RED, TrafficLight.YELLOW]
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image

        Args:
            image (cv::Mat): image containing the traffic light

        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)

        """
        rospy.loginfo("Got Image {}".format(self.count))
        if self.count == 0:
            K.clear_session() 
            self.model = load_model(os.getcwd() + '/light_classification/CNN.h5')
            
            image = cv2.resize(image, (300, 400))
            image = cv2.cvtColor(image,cv2.COLOR_BGR2RGB)
            #self.model.predict(np.asarray(image))
            #TODO implement light color prediction
            temp = self.model.predict(np.asarray(image)[np.newaxis, :])
            rospy.loginfo("Image Prediction {}".format(temp))
            self.buffer_state = self.ans_map[np.argmax(temp)]

        self.count = (self.count + 1) % 4

        return self.buffer_state
