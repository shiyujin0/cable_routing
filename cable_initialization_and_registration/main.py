'''
Shiyu Jin(jsy@berkeley.edu) 03/20/2022
Cable initialization and registration example.
Initialization using reeb graph.
Registration using Coherent Point Drift.
'''
import numpy as np
from pycpd import DeformableRegistration
from rope_initialization import Rope_initialization
from matplotlib import pyplot as plt
from functools import partial

def visualize(iteration, error, X, Y, ax):
	plt.cla()
	ax.scatter(X[:, 0],  X[:, 1], color='red', label='Target')
	ax.scatter(Y[:, 0],  Y[:, 1], color='blue', label='Source')
	plt.text(0.87, 0.92, 'Iteration: {:d}'.format(
		iteration), horizontalalignment='center', verticalalignment='center', transform=ax.transAxes, fontsize='x-large')
	ax.legend(loc='upper left', fontsize='x-large')
	plt.draw()
	plt.pause(0.1)

if __name__=="__main__":
	scale_init = 1000 #scaling factor for initializatoin
	scale_reg = 100 #scaling factor for registration
	point_cloud_0 = np.load('data/point_cloud_0.npy') #point cloud of frame 0
	point_cloud_1 = np.load('data/point_cloud_1.npy') #point cloud of frame 1

	#cable initialization with reeb graph
	rope_init = Rope_initialization()
	cable_nodes_pre = rope_init.find_path_through_point_cloud(point_cloud_0/scale_init)*scale_init
	
	#cable registration with coherent point drift(CPD)
	fig = plt.figure()
	fig.add_axes([0, 0, 1, 1])
	callback = partial(visualize, ax=fig.axes[0])
	reg = DeformableRegistration(**{'X': point_cloud_1/scale_reg, 'Y': cable_nodes_pre/scale_reg})
	reg.w = 0.1 #account for outliers
	reg.alpha = 10 #deault 2, more smooth
	reg.beta = 2 #default 2 
	reg.register(callback)
	plt.show()

	#equal distance nodes
	cable_nodes_2D = cable_nodes_pre + np.dot(reg.G, reg.W)*scale_reg
	cable_nodes_2D = rope_init.change_num_of_equal_distance_nodes(cable_nodes_2D)
