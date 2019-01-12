#! /usr/bin/env python2

import roslib
import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
import keras
from keras.models import load_model
from cv_bridge import CvBridge, CvBridgeError
import detect_ball
import time

max_size = 255*1000 # max size of ball allowed
#Number of images for one rotation (TODO: get this via IMU)
one_rot = 250
#Number of Pixels highlighted (output of unsupervised)
mask_size = np.zeros(one_rot)
#Ball Probability (output of supervised)
ball_prob = np.zeros(one_rot)
#List of np arrays: storing images
data = np.zeros([one_rot,2])

t = 0
#make ./file.h5
#Feed Forward wts
new_model = load_model('/home/youknowwho/Documents/ROS/src/image_rcv/src/new_model_num_coluoured.h5')


def supervised(array):
	# a = np.load("/home/ameya/Desktop/MRT/no_ball_h3.npy")
	# Input: np array of images Output: Max probab and index
	b = (new_model.predict(array, batch_size=None, verbose=0, steps=None))
	return np.argmax(b[:,0]),np.max(b[:,0])


if __name__ == '__main__':

	resource = 0
	print "Trying to open resource: " + str(resource)
	cap = cv2.VideoCapture(resource)
	# cap.set(cv2.CAP_PROP_POS_MSEC, 1000)
	if not cap.isOpened():
		print "Error opening resource: " + str(resource)
		print "Maybe opencv VideoCapture can't open it"
		exit(0)

	print "Correctly opened resource, starting to show feed."
	while not rospy.is_shutdown():
		rval, frame = cap.read()
		if rval:
			# cv2.imwrite("/home/youknowwho/Documents/ROS/src/image_send/ball/" + str(t) + ".jpg", frame)
			start = time.time()
			image, mask = detect_ball.detect_ball(frame) # get hysterisis mask
			
			if not np.any(mask):
				continue

			img_array = detect_ball.getMultiWindow(image, mask) # crop image and resize to 50x50
			data[t] = supervised(img_array)
			print "prob", data[t], "\ttime", time.time() - start
			cv2.imshow("mask", mask)
			cv2.imshow("frame", image)
			cv2.imshow("window",img_array[int(data[t][0])])
			t = t + 1

			if t == one_rot:
				# np.save(str(no) + ".npy", data)
				break
    		key = cv2.waitKey(20)
	cv2.destroyWindow("preview")

# how to get output of supervised with sync
# how to store relevant data from output of unsup and sup
# how to determine whether other node has failed
# how to get direction data with sync