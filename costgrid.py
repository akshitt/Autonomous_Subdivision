import numpy as np
import math 
import utm
import rospy
import sys
import signal
import pdb
from std_msgs.msg import String, Float32, Float32MultiArray, Float64MultiArray
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
from matplotlib import colors
import time

#---------------------------------------------------------------------------
#SIGINT handler
def sigint_handler(signal, frame):
    #Do something while breaking
    print("\nCtrl C Pressed")
    sys.exit(0)
#---------------------------------------------------------------------------

class cost_grid_class:
	def __init__(self,size=1000,random=False):
		if(random):
			self.cost_grid=np.random.rand(size,size)
		else:
			self.cost_grid=np.zeros((size,size))
		#Source UTM of lab area
		self.source_gps = [19.130815, 72.918408]
		self.target_gps = [19.1246214,72.1502671]

		#Convert to UTM
		self.source_utm = utm.from_latlon(self.source_gps[0], self.source_gps[1])
		self.target_utm = utm.from_latlon(self.target_gps[0], self.target_gps[1])

		#Define UTM Offset such that source is at 500,500
		self.utm_offset = (500 - self.source_utm[0], 500 - self.source_utm[1]) 
		self.scan_range = np.linspace(-0.6, 0.6, 51)

		#Initial Value of current utm
		self.curr_utm = self.source_utm
		self.curr_heading = 0 # in radians
		
		#Initialize current and past grid coordinates (to follow conventions)
		self.curr_grid_coord = (int(self.source_utm[1]+self.utm_offset[1]), int(self.source_utm[0]+self.utm_offset[0]))
		self.past_grid_coord = (int(self.source_utm[1]+self.utm_offset[1]), int(self.source_utm[0]+self.utm_offset[0]))
		self.scan_data = []	



	def gps_callback(self,inp):
		#Convert to UTM
		self.curr_utm = utm.from_latlon(inp.data[0], inp.data[1])
		#Convert to Grid Cords for plotting
		self.curr_grid_coord = (int(self.curr_utm[1]+self.utm_offset[1]), int(self.curr_utm[0]+self.utm_offset[0]))
		print(self.curr_grid_coord)
		#Update cost +10000 to make point appear green
		self.cost_grid[int(self.curr_grid_coord[0])][int(self.curr_grid_coord[1])] += 10000.0
		# cost_grid[int(past_grid_coord[0]), int(past_grid_coord[1])] -= 10000.0
		# past_grid_coord=curr_grid_coord
	#---------------------------------------------------------------------------

	def imu_callback(self,inp):
		self.curr_heading = inp.data[2]
	#---------------------------------------------------------------------------

	def scan_callback(self,inp):
		self.scan_data = inp.ranges
	#---------------------------------------------------------------------------

	def update_cost_grid(self): # assuming North is in y-direction of the grid
	    obstacle_range  = self.curr_heading + self.scan_range
	    if(len(self.scan_data)!=0):
	        for i in range(0,len(self.scan_range)):
	            if(not np.isnan(self.scan_data[i])):
	            	# minus for the convention
		            coord = [self.curr_grid_coord[0] - self.scan_data[i]*math.sin(-obstacle_range[i]), self.curr_grid_coord[1] - self.scan_data[i]*math.cos(-obstacle_range[i])]
		            print("cost grid updated: "+str(coord))
		            self.cost_grid[int(coord[0])][int(coord[1])] += 10.0 #20.0 is just a factor. we can tune it according to requirements of path planning algo

##code to subscribe gps data and heading data to update curr_utm and curr_heading 

# Grid object
grid_obj=cost_grid_class()
signal.signal(signal.SIGINT, sigint_handler)
rospy.Subscriber('LatLon',Float64MultiArray, grid_obj.gps_callback)
rospy.Subscriber('IMU',Float32MultiArray, grid_obj.imu_callback)
rospy.Subscriber('scan',LaserScan, grid_obj.scan_callback)
rospy.init_node('Level4', anonymous=True)
# dynamic code plotting
plt.ion()
fig = plt.figure()
ax = fig.add_subplot(1,1,1) 
cmap = colors.ListedColormap(['red', 'blue','green'])
bounds = [0,20,9000,20000]
norm = colors.BoundaryNorm(bounds, cmap.N)
# cost_grid=np.random.rand(1000,1000)*30
ax.set_xlim(0,1000)
ax.set_ylim(0,1000)
im = ax.imshow(grid_obj.cost_grid, cmap=cmap, norm=norm)

while True:
	# print("updating cost map")
	grid_obj.update_cost_grid()
	im.set_data(grid_obj.cost_grid)
	plt.pause(0.01)
	# print(cost_grid[curr_grid_coord[0]][curr_grid_coord[1]])
	time.sleep(0.1)