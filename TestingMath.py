import math as m
import numpy as np
import time

def RotAxis(axis,theta):
	theta = theta * m.pi/180
	if axis = "x":
		R = np.array([[1,0,0,0],
					 [0,m.cos(theta),-m.sin(theta),0],
					 [0,m.sin(theta),m.cos(theta),0],
					 [0,0,0,1]])
	if axis = "y":
		R = np.array([[m.cos(theta),0,m.sin(theta),0],
					 [0,1,0,0],
					 [-m.sin(theta),0,m.cos(theta),0],
					 [0,0,0,1]])
	if axis = "z":
		R = np.array([[m.cos(theta),-m.sin(theta),0,0],
					 [m.sin(theta),m.cos(theta),0,0],
					 [0,0,1,0],
					 [0,0,0,1]])
	return R

def RotAll(thx,thy,thz):
	Rx = RotAxis("x",thx)
	Ry = RotAxis("y",thy)
	Rz = RotAxis("z",thz)
	R = np.dot(Rx,Ry,Rz)
	return R

def MovA(x,y,z):
	M = np.array([[1,0,0,x],
				  [0,1,0,y],
				  [0,0,1,z],
				  [0,0,0,1]])
	return M


def bP2gP_xformer():
	
	sqrt3 = m.sqrt(3)
	b_1 = np.array([-sqrt3/2],	[-1/2],	[0],[1])
	b_2 = np.array([0],			[-1],	[0],[1))
	b_3 = np.array([sqrt3/2],	[-1/2],	[0],[1])
	b_4 = np.array([sqrt3/2],	[1/2],	[0],[1])
	b_5 = np.array([0],			[1],	[0],[1])
	b_6 = np.array([-sqrt3/2],	[1/2],	[0],[1])
	
	#inputs
	thx
	thy
	thz
	


	tic = time.perf_counter()
	
	R_1tob = RotAll(thx,thy,thz)
	M_1tob
	R_btog
	M_btog
	
	
	P1_1 = np.dot(R_1tob,M_1tob,R_btog,M_btog)

	
	toc = time.perf_counter()
	
	print(np.around(bP, decimals=2))
	print("to")
	print(np.around(gP, decimals=2))
	print(f"Main loop took : {toc - tic:0.6f} seconds to complete.")

#hello
bP2gP_xformer()
