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

from keras.applications.inception_v3 import InceptionV3
from keras.preprocessing.image import ImageDataGenerator

train_dir = './train_image/train/'
validation_dir = './train_image/validation/'

train_datagen = ImageDataGenerator(rescale=1./255,
                                   shear_range=0.2,
                                   zoom_range=0.1,
                                   rotation_range=5.0,
                                   width_shift_range=0.2,
                                   height_shift_range=0.2,
                                   horizontal_flip=True)

validation_datagen = ImageDataGenerator(rescale=1./255)

train_generator = train_datagen.flow_from_directory(train_dir,
                                                    target_size=(400, 300),
                                                    batch_size=32)
validation_generator = validation_datagen.flow_from_directory(validation_dir,
                                                    target_size=(400, 300),
                                                    batch_size=32)

model = Sequential([
    Conv2D(16, (3, 3), activation="relu", input_shape=(400, 300, 3)),
    MaxPooling2D(2, 2),
    Conv2D(16, (3, 3), activation="relu"),
    MaxPooling2D(2, 2),
    Conv2D(32, (3, 3), activation="relu"),
    MaxPooling2D(2, 2),
    Conv2D(32, (3, 3), activation="relu"),
    MaxPooling2D(2, 2),
    Conv2D(64, (3, 3), activation="relu"),
    MaxPooling2D(2, 2),
    Conv2D(64, (3, 3), activation="relu"),
    MaxPooling2D(2, 2),
    Flatten(),
    Dense(512, activation='relu'),
    Dense(4, activation='softmax')
])

model.compile(optimizer=RMSprop(lr=0.001),
              loss='categorical_crossentropy',
              metrics=['accuracy'])

model.summary()

history = model.fit_generator(train_generator, steps_per_epoch=100, epochs=20, validation_data=validation_generator, validation_steps=25)

model.save('CNN_simple.h5')

acc = history.history['accuracy']
val_acc = history.history['val_accuracy']
loss = history.history['loss']
val_loss = history.history['val_loss']

epochs = range(len(acc))
plt.plot(epochs, acc)
plt.plot(epochs, val_acc)
plt.title('Training and validation accuracy')
plt.figure()

plt.plot(epochs, loss)
plt.plot(epochs, val_loss)
plt.title('Training and validation loss')
plt.show()