import random 
import numpy as np
import matplotlib.pyplot as plt 
import pdb
import scipy.linalg as la
import time
from sklearn.cluster import KMeans
from sklearn.datasets import make_blobs

x = []
y = []

rad = []
area=[]
obs_cords=[]

numobs = 55

for i in range(numobs):
	pt = (random.randint(0,1000))
	r = (random.randint(-250,250))
	x.append(pt)
	y.append(pt+r)
	rad.append(random.randint(5,25))
	area.append(rad[i]**2)

xcord = []
ycord = []

for i in range(numobs):
	for j in range(rad[i]):
		for k in range(rad[i]):
			xcord.append( x[i]-rad[i]+ 2*j )
			ycord.append( y[i]-rad[i]+ 2*k )
			obs_cords.append(np.array([x[i]-rad[i]+2*j , y[i]-rad[i]+2*k]))

coord = np.column_stack((xcord, ycord))
plt.scatter(xcord,ycord, s=4, facecolors='g')

kmeans = KMeans(n_clusters=25, random_state=170).fit_predict(coord)
plt.scatter(coord[:,0], coord[:,1], c=kmeans)
plt.show()