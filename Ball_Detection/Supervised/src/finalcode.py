import cv2
import numpy as np
import keras
from keras.models import load_model
# Can be integrated in the unsupervised code

def test(array):
	# a = np.load("/home/ameya/Desktop/MRT/no_ball_h3.npy")
	new_model = load_model('/home/youknowwho/Documents/ROS/src/image_rcv/src/new_model_num_coluoured.h5')
	b = (new_model.predict(array, batch_size=None, verbose=0, steps=None))
	i = 0
	store = 0
	big = 0
	while i< len(b):
		z = big
		big = max(big,b[i][0])
		if big > z:
			store = i
		i = i+1
	return store 

def testvalue(array):
	# a = np.load("/home/ameya/Desktop/MRT/no_ball_h3.npy")
	new_model = load_model('/home/youknowwho/Documents/ROS/src/image_rcv/src/new_model_num_coluoured.h5')
	b = (new_model.predict(array, batch_size=None, verbose=0, steps=None))
	i = 0
	store = 0
	big = 0
	while i< len(b):
		z = big
		big = max(big,b[i][0])
		if big > z:
			store = i
		i = i+1
	return big

def supervised(array):
	global new_model
	# a = np.load("/home/ameya/Desktop/MRT/no_ball_h3.npy")
	b = (new_model.predict(array, batch_size=None, verbose=0, steps=None))
	i = 0
	store = 0
	big = 0
	while i< len(b):
		z = big
		big = max(big,b[i][0])
		# print "prob", i,"\t",b[i][0]
		if big > z:
			store = i
		i = i+1
	# print
	# print
	return store, big