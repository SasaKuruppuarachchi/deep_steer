#!/usr/bin/env python3
from cv_bridge import CvBridge, CvBridgeError
from keras.models import model_from_json
import rospy
import cv2
import numpy as np
from dronet_perception.msg import CNN_out
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Empty
#from utils import *

from keras import backend as K

TEST_PHASE=0

bridge = CvBridge()

def callback_img(data, target_size, crop_size, rootpath, save_img):
    try:
        image_type = data.encoding
        img = bridge.imgmsg_to_cv2(data, image_type)
    except CvBridgeError as e:
        print (e)

    img = cv2.resize(img, target_size)
    #h,w,c = img.shape
    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    #print(crop_size[0], crop_size[1])
    img = central_image_crop(img, crop_size[0], crop_size[1])

    if rootpath and save_img:
        temp = rospy.Time.now()
        cv2.imwrite("{}/{}.jpg".format(rootpath, temp), img)

    return np.asarray(img, dtype=np.float32) * np.float32(1.0/255.0)


def central_image_crop(img, crop_width, crop_heigth):
    """
    Crops the input PILLOW image centered in width and starting from the bottom
    in height.
    Arguments:
        crop_width: Width of the crop
        crop_heigth: Height of the crop
    Returns:
        Cropped image
    """
    half_the_width = img.shape[1] / 2
    #print((img.shape[0] - crop_heigth), img.shape[0], (half_the_width - (crop_width / 2)), (half_the_width + (crop_width / 2)))
    img = img[int(img.shape[0] - crop_heigth-1): int(img.shape[0]-1),
              int((half_the_width - (crop_width / 2))): int((half_the_width + (crop_width / 2)))]
    img = img.reshape(img.shape[0], img.shape[1], 1)
    return img

def jsonToModel(json_model_path):
    with open(json_model_path, 'r') as json_file:
        loaded_model_json = json_file.read()

    model = model_from_json(loaded_model_json)

    return model


class Dronet(object):
    def __init__(self,
                 json_model_path,
                 weights_path, target_size=(200, 200),
                 crop_size=(150, 150),
                 imgs_rootpath="../models"):

        self.pub = rospy.Publisher("cnn_predictions", CNN_out, queue_size=5)
        self.feedthrough_sub = rospy.Subscriber("state_change", Bool, self.callback_feedthrough, queue_size=1)
        self.land_sub = rospy.Subscriber("land", Empty, self.callback_land, queue_size=1)

        self.use_network_out = False
        self.imgs_rootpath = imgs_rootpath

        # Set keras utils
        K.set_learning_phase(TEST_PHASE)

        # Load json and create model
        model = jsonToModel(json_model_path)
        # Load weights
        model.load_weights(weights_path)
        print("Loaded model from {}".format(weights_path))

        model.compile(loss='mse', optimizer='sgd')
        self.model = model
        self.target_size = target_size
        self.crop_size = crop_size

    def callback_feedthrough(self, data):
        self.use_network_out = data.data

    def callback_land(self, data):
        self.use_network_out = False

    def run(self):
        while not rospy.is_shutdown():
            msg = CNN_out()
            msg.header.stamp = rospy.Time.now()
            data = None
            while data is None:
                try:
                    data = rospy.wait_for_message("camera", Image, timeout=10)
                except:
                    pass

            if self.use_network_out:
                print("Publishing commands!")
            else:
                print("NOT Publishing commands!")

            cv_image = callback_img(data, self.target_size, self.crop_size,
                self.imgs_rootpath, self.use_network_out)
            outs = self.model.predict_on_batch(cv_image[None])
            steer, coll = outs[0][0], outs[1][0]
            msg.steering_angle = steer
            msg.collision_prob = coll
            self.pub.publish(msg)
