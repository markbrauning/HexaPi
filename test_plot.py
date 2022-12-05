import pygame, time, threading, sys
from pygame.locals import *
from pygame import event
import numpy as np
from numpy.linalg import multi_dot
import math as m


global psc
global jsc
global exit_main
global controller_connected
global refresh_rate



#Global Variables
psc = None

#Pygame global vars:
global HEIGHT, WIDTH
HEIGHT = 750
WIDTH = 750
global purple, blue, red, white, black, seagreen
purple = (100,0,255)
blue=(0,0,255)
red=(255,0,0)
white=(255,255,255)
black=(0,0,0)
seagreen=(32,178,170)

class controller_class:
	def __init__(self,i):
		self.ctrl = pygame.joystick.Joystick(i)
		self.ctrl.init()
		self.ctrl_name = self.ctrl.get_name()
		self.ctrl_id = self.ctrl.get_id()
		self.btn_num = self.ctrl.get_numbuttons()
		self.ax_num = self.ctrl.get_numaxes()
		self.ht_num = self.ctrl.get_numhats()
		self.supported = False
		self.ctrl_mapping = self.controller_mapping(self.ctrl_name)
		if not self.ctrl_mapping == None:
			self.supported = True
	def controller_mapping(self, ctrl_name):
		#PlayStation 3: "Sony PLAYSTATION(R)3 Controller"
		PS3_mapping_name = [["Cross","Circle","Triangle","Square",
							"L1","R1","L2","R2",
							"Select","Start","PS Logo","Left Joystick","Right Joystick",
							"D-pad Up","D-pad Down","D-pad Left","D-pad Right",],
							["Left Joystick x-axis","Left Joystick y-axis","L2 axis",
							"Right Joystick x-axis","Right Joystick y-axis","R2 axis"]]
		PS3_mapping = [["b_cross","b_circle","b_tri","b_square",
						"b_L1","b_R1","b_L2","b_R2",
						"b_select","b_start","b_ps","b_Lj","b_Rj",
						"b_up","b_down","b_L","b_R",], 
						["j_Lx","j_Ly","j_L2",
						"j_Rx","j_Ry","j_R2"]]
		#PlayStation 4: "Wireless Controller"				
		PS4_mapping_name = [["Cross","Circle","Triangle","Square",
							"L1","R1","L2","R2",
							"Share","Options","PS Logo","Left Joystick","Right Joystick"],
							["Left Joystick x-axis","Left Joystick y-axis","L2 axis",
							"Right Joystick x-axis","Right Joystick y-axis","R2 axis"],
							[["D-pad Left", "D-pad Right","D-pad Down", "D-pad Up"]]]
		PS4_mapping = [["b_cross","b_circle","b_tri","b_square",
						"b_L1","b_R1","b_L2","b_R2",
						"b_share","b_options","b_ps","b_Lj","b_Rj"],
						["j_Lx","j_Ly","j_L2",
						"j_Rx","j_Ry","j_R2"],
						[["b_L","b_R","b_down","b_up"]]]
		
		ctrl_mapping = None
		if ctrl_name == "Sony PLAYSTATION(R)3 Controller":
			ctrl_mapping = [PS3_mapping,PS3_mapping_name]
		elif ctrl_name == "Wireless Controller":
			ctrl_mapping = [PS4_mapping,PS4_mapping_name]
		return ctrl_mapping

class psc_update:
	def __init__(self, prev_psc, refresh_rate, jsc):
		idle_toggle = True
		events = pygame.event.get()

		#Buttons:
		self.btn = [0]*jsc.btn_num
		for i in range(jsc.btn_num):
			button_state = jsc.ctrl.get_button(i)
			self.btn[i] = self.button(i, button_state, jsc.ctrl_mapping[1][0][i], prev_psc, refresh_rate)
			setattr(self, jsc.ctrl_mapping[0][0][i], self.btn[i])
			if self.btn[i].state_changed == 1:
				idle_toggle = False
		#Axes:
		self.ax = [0]*jsc.ax_num
		for i in range(jsc.ax_num):
			axis_val = round(jsc.ctrl.get_axis(i),2)
			self.ax[i] = self.axis(i, axis_val, jsc.ctrl_mapping[1][1][i], prev_psc)
			setattr(self, jsc.ctrl_mapping[0][1][i], self.ax[i])
			if self.ax[i].val_changed == 1:
				idle_toggle = False
		#Hats:
		self.ht = [0]*4*jsc.ht_num
		for i in range(jsc.ht_num):
			hat_states = jsc.ctrl.get_hat(i)
			split_states = self.hat_split_states(hat_states)
			for j in range(4*i, 4):
				unshifted_j = j - (i*4)
				self.ht[j] = self.hat(j, split_states[unshifted_j], jsc.ctrl_mapping[0][2][i][unshifted_j], prev_psc, refresh_rate)
				setattr(self, jsc.ctrl_mapping[0][2][i][unshifted_j], self.ht[j])
				if self.ht[j].state_changed == 1:
					idle_toggle = False
		
		#Idle Duration
		if idle_toggle == True:
			try:
				self.idle_count = prev_psc.idle_count + 1
			except:
				#print("Failed to set Idel Count")
				self.idle_count = 0
		else:
			self.idle_count = 0
		
		#Idle Duration in seconds
		self.idle_seconds = round(self.idle_count * refresh_rate,2)
	
	class button:
		def __init__(self, index, state, pname, prev_psc, refresh_rate):
			self.state = state
			self.index = index
			self.name = pname
			#Press Duration
			if self.state == 1:
				try:
					self.pd = prev_psc.btn[index].pd + 1
				except:
					self.pd = 0
			else:
				self.pd = 0
			#Press Duration in seconds
			self.pds = round(self.pd * refresh_rate,2)
			#State changed?
			try:
				if self.state != prev_psc.btn[index].state:
					self.state_changed = 1
				else:
					self.state_changed = 0
			except:
				#should only happen for firt time assignment
				self.state_changed = 0
			#pressed
			if self.state == 1 and self.state_changed == 1:
				self.pressed = 1
			else:
				self.pressed = 0
			#released
			if self.state == 0 and self.state_changed == 1:
				self.released = 1
			else:
				self.released = 0
	
	class axis:
		def __init__(self, index, val, pname, prev_psc):
			self.val = val
			self.index = index
			self.name = pname
			#Value changed?
			try:
				if self.val != prev_psc.ax[index].val:
					self.val_changed = 1
				else:
					self.val_changed = 0
			except:
				#should only happen for firt time assignment
				self.val_changed = 0
	
	def hat_split_states(self, hat_states):
		split_states = [0]*4
		if hat_states[0] == -1:
			split_states[0] = 1
		elif hat_states[0] == 1:
			split_states[1] = 1
		else:
			split_states[0] = 0
			split_states[1] = 0
		if hat_states[1] == -1:
			split_states[2] = 1
		elif hat_states[1] == 1:
			split_states[3] = 1
		else:
			split_states[2] = 0
			split_states[3] = 0
		return split_states
	
	class hat:
		def __init__(self, index, hat_state, hat_name, prev_psc, refresh_rate):
			self.state = hat_state
			self.name = hat_name
			self.index = index
			if self.state == 1:
				try:
					self.pd = prev_psc.ht[self.index].pd + 1
				except:
					print(self.name + " Failed to set press duration")
					self.pd = 0
			else:
				self.pd = 0
			#Press Duration in seconds
			self.pds = round(self.pd * refresh_rate,2)
			#State changed?
			try:
				if self.state != prev_psc.ht[self.index].state:
					self.state_changed = 1
				else:
					self.state_changed = 0
			except:
				#should only happen for firt time assignment
				self.state_changed = 0
			#pressed
			if self.state == 1 and self.state_changed == 1:
				self.pressed = 1
			else:
				self.pressed = 0
			#released
			if self.state == 0 and self.state_changed == 1:
				self.released = 1
			else:
				self.released = 0

class module_select:
	def __init__(self):
		functions = ["Sim_Walk_module"]
		for i in range(len(functions)):
			setattr(self, functions[i], "STOP")
		self.allstopped = True
	def start_stop(self,mod):
		if getattr(self,mod) == "RUN":
			setattr(self, mod, "STOP")
		else:
			setattr(self, mod, "RUN")
			for key, val in self.__dict__.items():
				if mod != key:
					if val == "RUN":
						setattr(self, key, "STOP")
						print(key + " function set to: " + getattr(self,key))
		print(mod + " function set to: " + getattr(self,mod))
		self.all_stopped()
	def all_stopped(self):
		self.allstopped = True
		for key, val in self.__dict__.items():
			if val == "RUN":
				self.allstopped = False

def controller_connection():
	global controller_connected
	global jsc
	global psc

	pygame.init()
	pygame.display.init()
	idle_wait = 60
	while True:
		time.sleep(2)
		if not exit_main:
			attempts = 0
			controller_count = 0
			prev_psc = None
			#print("Checking if controller_connected True or False")
			while not controller_connected:
				pygame.joystick.init()
				attempts = attempts + 1
				controller_count = pygame.joystick.get_count()
				msg = "\rController count: " + str(controller_count) + " Scanning for Controllers... Attempt " + str(attempts)
				sys.stdout.write(msg)
				sys.stdout.flush()
				if controller_count > 0:
					for i in range(controller_count):
						pygame.joystick.init()
						jsc = controller_class(i)
						print("\nController found: " + jsc.ctrl_name)
						if jsc.supported == True:
							print("Controller connected: " + jsc.ctrl_name)
							psc = psc_update(prev_psc, refresh_rate, jsc)
							jid_connected = pygame.joystick.Joystick(i).get_id()
							controller_connected = True
							break
						else:
							controller_connected = False
							print("Controller not supported: " + jsc.ctrl_name)
							controller = None
							pygame.joystick.quit()
							time.sleep(1)
				else:
					pygame.joystick.quit()
					time.sleep(1)
			
			#Check if there is still a controller connected
			while controller_connected:
				time.sleep(0.5)
				idle_time = psc.idle_seconds
				#only check if controller inputs have gone unchanged for 'idle_wait' seconds
				if idle_time > idle_wait:
					msg = "\rController idle for " + str(idle_time) + ". Checking connection status"
					sys.stdout.write(msg)
					sys.stdout.flush()
					pygame.joystick.quit()
					pygame.joystick.init()
					try:
						jid = pygame.joystick.Joystick(jid_connected).get_id()
						jsc = controller_class(jid)
						#print("Controller idle for " + str(idle_time) + " seconds but still connected: " + jsc.ctrl_name)
						time.sleep(idle_wait)
					except:
						print("\nController lost connection: " + jsc.ctrl_name)
						controller_connected = False
				
		else:
			break

#-----------------------Math Functions
def MatScale(Scale):
	return np.array([[Scale,0,0,0],
					 [0,Scale,0,0],
					 [0,0,Scale,0],
					 [0,0,0,1]])
def RotAxis(axis,theta):
	theta = theta * m.pi/180
	if axis == "x":
		R = np.array([[1,0,0,0],
					 [0,m.cos(theta),-m.sin(theta),0],
					 [0,m.sin(theta),m.cos(theta),0],
					 [0,0,0,1]])
	if axis == "y":
		R = np.array([[m.cos(theta),0,m.sin(theta),0],
					 [0,1,0,0],
					 [-m.sin(theta),0,m.cos(theta),0],
					 [0,0,0,1]])
	if axis == "z":
		R = np.array([[m.cos(theta),-m.sin(theta),0,0],
					 [m.sin(theta),m.cos(theta),0,0],
					 [0,0,1,0],
					 [0,0,0,1]])
	return R
def RotAll(thx,thy,thz):
	Rx = RotAxis("x",thx)
	Ry = RotAxis("y",thy)
	Rz = RotAxis("z",thz)
	R = multi_dot([Rx,Ry,Rz])
	return R
def MovA(Vec):
	x= Vec[0][0]
	y= Vec[1][0]
	z= Vec[2][0]
	Vec = np.array([[1,0,0,x],
					[0,1,0,y],
					[0,0,1,z],
					[0,0,0,1]])
	return Vec
def Xlator(V,MT,thx,thy,thz):
	#rotate then move = M*R*V
	P = multi_dot([MovA(MT),RotAll(thx,thy,thz),V])
	return P

#-----------------------Pygame Functions
def txt(msg,color,row,col,screen,font_style):
	pos = cord([-(WIDTH/2)+col*100,(HEIGHT/2)-row*15])
	mesg = font_style.render(msg, True, color)
	screen.blit(mesg, pos)
def txt2d(msg,color,pos,screen,font_style):
	mesg = font_style.render(msg, True, color)
	screen.blit(mesg, cord(pos))
def cord(pos):
	scn_offset_x = WIDTH/2
	scn_offset_y = HEIGHT/2
	x=int(scn_offset_x+pos[0])
	y=int(-scn_offset_y+HEIGHT-pos[1])
	return [x,y]
def cordx(pos):
	scn_offset_x = WIDTH/2
	x=int(scn_offset_x+pos)
	return x
def cordy(pos):
	scn_offset_y = HEIGHT/2
	y=int(-scn_offset_y+HEIGHT-pos)
	return y
def xycoord(coordname,RotAboutG,MovRel2G,coordscale,coordcolor,screen,font_style):
	#RotAboutG: [thx,thy,thz]
	#MovRel2G:  [[x],[y],[z],[1]]
	
	#def axis lines with scaling
	x0 = np.array([[0],[0],[0],[1]])
	x = np.array([[1*coordscale],[0],[0],[1]])
	y0 = np.array([[0],[0],[0],[1]])
	y = np.array([[0],[1*coordscale],[0],[1]])
	
	#rotate then move
	x0_G = Xlator(x0,MovRel2G,RotAboutG[0],RotAboutG[1],RotAboutG[2])
	x_G = Xlator(x,MovRel2G,RotAboutG[0],RotAboutG[1],RotAboutG[2])
	y0_G = Xlator(y0,MovRel2G,RotAboutG[0],RotAboutG[1],RotAboutG[2])
	y_G = Xlator(y,MovRel2G,RotAboutG[0],RotAboutG[1],RotAboutG[2])
	
	#[x,y]
	pos_x0_G = [x0_G[0][0],x0_G[1][0]]
	pos_x_G = [x_G[0][0],x_G[1][0]]
	pos_y0_G = [y0_G[0][0],y0_G[1][0]]
	pos_y_G = [y_G[0][0],y_G[1][0]]
	
	#draw X axis
	pygame.draw.line(screen,coordcolor,cord(pos_x0_G),cord(pos_x_G),2)
	pygame.draw.line(screen,coordcolor,cord(pos_y0_G),cord(pos_y_G),2)
	
	#Axis and Origin labels
	txt2d("x",coordcolor,[pos_x_G[0]+5,pos_x_G[1]+7],screen,font_style)
	txt2d("y",coordcolor,[pos_y_G[0]-2,pos_y_G[1]+15],screen,font_style)
	txt2d(coordname,coordcolor,[pos_x0_G[0]-3,pos_x0_G[1]-3],screen,font_style)

def hexapi_main():
	#Global Variables:
	global psc
	global jsc
	global exit_main
	global controller_connected
	global refresh_rate
	global mods
	global t
	global stride_speed
	global walk_idle
	
	#Global Variable Def
	psc = None
	jsc = None
	exit_main = False
	controller_connected = False
	refresh_rate = 0.01
	mods = module_select()
	stride_speed = 0
	walk_idle = 0
	
	#Define controller_connection thread
	controller_connection_thread = threading.Thread(target=controller_connection,args=())
	controller_connection_thread.setDaemon(True)
	controller_connection_thread.start()
	
	#-----------------------Pygame Setup
	pygame.init()
	screen = pygame.display.set_mode((WIDTH, HEIGHT))
	pygame.display.set_caption("HexaPi")
	font_style = pygame.font.SysFont(None, 20)
	screen.fill(white)
	
	#-----------------------Body Coordinate systems relative to global (Changed by Controller inputs)
	bx_g = 0
	by_g = 0
	bz_g = 0
	b_g = np.array([[bx_g],[by_g],[bz_g],[1]])
	bthx_g = 0
	bthy_g = 0
	bthz_g = 0
	br_g = [bthx_g,bthy_g,bthz_g]

	#-----------------------Leg Coordinate systems relative to Body (CONSTANT)
	BodyR = 100
	sqrt3 = m.sqrt(3)
	L1_b = np.array([[BodyR*sqrt3/2],	[BodyR*1/2],	[0],[1]])
	L2_b = np.array([[BodyR*0],			[BodyR*1],		[0],[1]])
	L3_b = np.array([[BodyR*-sqrt3/2],	[BodyR*1/2],	[0],[1]])
	L4_b = np.array([[BodyR*-sqrt3/2],	[BodyR*-1/2],	[0],[1]])
	L5_b = np.array([[BodyR*0],			[BodyR*-1],		[0],[1]])
	L6_b = np.array([[BodyR*sqrt3/2],	[BodyR*-1/2],	[0],[1]])

	L1r_b = [0,0,-60]
	L2r_b = [0,0,0]
	L3r_b = [0,0,60]
	L4r_b = [0,0,120]
	L5r_b = [0,0,180]
	L6r_b = [0,0,240]

	L_b = np.array([L1_b,L2_b,L3_b,L4_b,L5_b,L6_b])
	Lr_b = np.array([L1r_b,L2r_b,L3r_b,L4r_b,L5r_b,L6r_b])

	#-----------------------Leg Coordinate systems relative to Global
	LNull_g = np.array([[0],[0],[0],[1]])
	L_g = np.array([LNull_g,LNull_g,LNull_g,LNull_g,LNull_g,LNull_g])
	for leg in range(6):
		L_g[leg] = Xlator(L_b[leg],b_g,br_g[0],br_g[1],br_g[2])
	
	#Main Loop
	while not exit_main:
		#Monitor Loop
		while controller_connected:
			time.sleep(refresh_rate)
			tic = time.perf_counter()
			
			#Try to update controller input and status values
			prev_psc = psc
			try:
				psc = psc_update(prev_psc, refresh_rate, jsc)
				
			except:
				psc = prev_psc
			
			#run primary functions
			main_input_monitor()
			
			#-----------------------Pygame Stuff
			screen.fill(white)
			#Global Coords:
			xycoord("g",[0,0,0],[[0],[0],[0],[1]],50,red,screen,font_style)
			
			#Body Coordinate systems relative to global (Changed by Controller inputs)
			bx_g = bx_g + 3*psc.j_Lx.val
			by_g = by_g - 3*psc.j_Ly.val
			bz_g = 0
			b_g = np.array([[bx_g],[by_g],[bz_g],[1]])
			bthx_g = 0
			bthy_g = 0
			bthz_g = bthz_g - 3*psc.j_Rx.val
			br_g = [bthx_g,bthy_g,bthz_g]
			
			#Body Coords:
			xycoord("b",br_g,b_g,35,seagreen,screen,font_style)
			
			#Leg Coords
			for leg in range(6):
				axName = "L"+str(leg+1)
				L_g[leg] = Xlator(L_b[leg],b_g,br_g[0],br_g[1],br_g[2])
				xycoord(axName,Lr_b[leg],L_g[leg],20,blue,screen,font_style)
				
			#pygame.draw.rect(screen,blue,[cordx(x),cordy(y),5,5])
			toc = time.perf_counter()
			if (toc - tic) > refresh_rate:
				txt("Lagging!",red,48,4,screen,font_style)
				#print(f"Main loop is lagging and took : {toc - tic:0.5f} seconds to complete.")
			txt("Refresh rate: "+ str(refresh_rate),black,47,0,screen,font_style)	
			txt(f"Main loop took : {toc - tic:0.6f} seconds to complete.",black,48,0,screen,font_style)
			#Print off controller button/joy valus:
			for i in range(jsc.ax_num):
				txt(psc.ax[i].name,black,i,3,screen,font_style)
				txt(str(psc.ax[i].val),black,i,5,screen,font_style)
			pygame.display.update()
		time.sleep(0.5)
	print("End of hexapi_main code.")

def Sim_Walk_module():
	global walk_idle
	global gait_selected
	global gait_select_size
	global t
	global foot_vectors
	global stride_speed
	
	#if controller inputs change then caluclate stride speed
	if (psc.j_Rx.val_changed == 1) or (psc.j_Lx.val_changed == 1) or (psc.j_Ly.val_changed == 1):
		stride_speed = abs(round(psc.j_Rx.val + psc.j_Lx.val + psc.j_Ly.val,0))
	
	#check how long inputs have been idle
	if (psc.j_Rx.val == 0) and (psc.j_Lx.val == 0) and (psc.j_Ly.val == 0):
		walk_idle = walk_idle + 1
	else:
		walk_idle = 0
	walk_idle_time = round(walk_idle* refresh_rate,2)
	
	#if idle time exceeds x seconds, then stand still
	if walk_idle_time > 8:
		msg = "\rIdle walk seconds: " + str(walk_idle_time)
		#print to sim
	else:
		if stride_speed != 0:
			#walk
			pass

def main_input_monitor():
	global exit_main
	global controller_connected
	
	#Print button pessed name
	for i in range(jsc.btn_num):
		if psc.btn[i].pressed == 1:
			print(psc.btn[i].name)
	
	#hexapi_main Exit
	if psc.b_ps.pds == 1:
		print("PS Logo button pressed for 1 seconds. Exiting Monitor Loop...")
		controller_connected = False
		exit_main = True
	
	#Triangle
	#Sim_Walk_module Run or Not
	if psc.b_tri.pressed == 1:
		mods.start_stop("Sim_Walk_module")
	if mods.Sim_Walk_module == "RUN":
		Sim_Walk_module()

#Start program
hexapi_main()
