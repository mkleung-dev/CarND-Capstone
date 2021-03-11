import os
import tensorflow as tf
import matplotlib.pyplot as plt
import numpy as np

from keras.models import Model, Sequential
from keras.layers import Dense, Input
from keras.layers.convolutional import Conv2D
from keras.layers.core import Flatten
from keras.layers.pooling import MaxPooling2D
from keras.optimizers import RMSprop
from keras.layers.pooling import MaxPooling2D

from keras.preprocessing import image
from keras.applications.inception_v3 import InceptionV3
from keras.preprocessing.image import ImageDataGenerator
from keras.models import load_model

import cv2

model = load_model('./CNN.h5')

file_name = './train_image/train/green/0000.jpg'
img = image.load_img(file_name, target_size=(400, 300))
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
print(model.predict(x))
img = cv2.imread(file_name)
img = cv2.resize(img, (300, 400))
img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
x = np.asarray(img)[np.newaxis, :]
print(model.predict(x))

file_name = './train_image/train/none/0000.jpg'
img = image.load_img(file_name, target_size=(400, 300))
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
print(model.predict(x))
img = cv2.imread(file_name)
img = cv2.resize(img, (300, 400))
img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
x = np.asarray(img)[np.newaxis, :]
print(model.predict(x))

file_name = './train_image/train/red/0000.jpg'
img = image.load_img(file_name, target_size=(400, 300))
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
print(model.predict(x))
img = cv2.imread(file_name)
img = cv2.resize(img, (300, 400))
img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
x = np.asarray(img)[np.newaxis, :]
print(model.predict(x))

file_name = './train_image/train/yellow/0000.jpg'
img = image.load_img(file_name, target_size=(400, 300))
x = image.img_to_array(img)
x = np.expand_dims(x, axis=0)
print(model.predict(x))
img = cv2.imread(file_name)
img = cv2.resize(img, (300, 400))
img = cv2.cvtColor(img,cv2.COLOR_BGR2RGB)
x = np.asarray(img)[np.newaxis, :]
print(model.predict(x))