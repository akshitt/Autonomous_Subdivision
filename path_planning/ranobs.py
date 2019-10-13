import random 
import numpy as np
import matplotlib.pyplot as plt 
import pdb
import scipy.linalg as la

plt.ion()

fig, ax = plt.subplots()

def avoid_obs():
	global curr_cord
	global curr_obs_cord
	curr_cord_left = [curr_cord[0],curr_cord[1]+2]
	curr_cord_right = [curr_cord[0]+2,curr_cord[1]]
	dist = la.norm(curr_cord - curr_obs_cord)
	dist_obs_left=[la.norm(curr_cord_left - obs_cords[i]) for i in range(numobs)]
	dist_obs_right=[la.norm(curr_cord_right - obs_cords[i]) for i in range(numobs)]
	
	if(np.min(dist_obs_left)>np.min(dist_obs_right)):
		curr_cord = curr_cord_left

	else:
		curr_cord = curr_cord_right

def back_on_tracc():
	global curr_cord
	global curr_obs_cord

	if(curr_cord[0]>curr_cord[1]):
		curr_cord = [curr_cord[0],curr_cord[1]+1]

	elif(curr_cord[0]<curr_cord[1]):
		curr_cord = [curr_cord[0]+1,curr_cord[1]]

	array = plot.get_offsets()
	array = np.append(array, curr_cord)
	plot.set_offsets(array)
	fig.canvas.draw()



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

plot = ax.scatter([], [], s=4, facecolors='g', edgecolors='g')
ax.set_xlim(0, 1000)
ax.set_ylim(0, 1000)
array = list(zip(xcord, ycord))
plot.set_offsets(array)
fig.canvas.draw()
		

# plt.scatter(xcord, ycord, s=4, facecolors='r', edgecolors='r')
# plt.show()

# pdb.set_trace()


# obs_cords=np.sort(obs_cords,axis=0)

numobs = len(obs_cords) 

curr_cord = np.zeros(2)
fin_cord=np.array([1000,1000])
step_size=1

while(la.norm(curr_cord-fin_cord)>5):

	dist_obs=[la.norm(curr_cord-obs_cords[i]) for i in range(numobs)]
	array = plot.get_offsets()
	array = np.append(array, curr_cord)
	plot.set_offsets(array)
	fig.canvas.draw()
	
	if(np.min(dist_obs)<10):
		
		curr_obs=np.argmin(dist_obs)
		
		curr_obs_cord = obs_cords[curr_obs]

		while(np.min(dist_obs)<10):
			print(curr_cord)
			# plt.plot(curr_cord)
			dist_obs=[la.norm(curr_cord-obs_cords[i]) for i in range(numobs)]
			array = plot.get_offsets()
			array = np.append(array, curr_cord)
			plot.set_offsets(array)
			fig.canvas.draw()
			avoid_obs()
				
	
	elif(np.min(dist_obs)>30 and curr_cord[0]!=curr_cord[1]):
		while(curr_cord[0]!=curr_cord[1]):
			dist_obs=[la.norm(curr_cord-obs_cords[i]) for i in range(numobs)]
			if(np.min(dist_obs)>10):	
				back_on_tracc()
			else:
				avoid_obs()
						

	else:
		curr_cord+=np.ones(2)*step_size



# plt.show()
# if(la.norm(curr_cord-fin_cord)<=5):


