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
HEIGHT = 1000
WIDTH = 1000
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
	S = np.array([[Scale,0,0,0],
					 [0,Scale,0,0],
					 [0,0,Scale,0],
					 [0,0,0,1]])
	return S
def RotAxis(axis,theta):
	theta = theta * m.pi/180
	if axis == "x":
		RTa = np.array([[1,0,0,0],
					 [0,m.cos(theta),-m.sin(theta),0],
					 [0,m.sin(theta),m.cos(theta),0],
					 [0,0,0,1]])
	if axis == "y":
		RTa = np.array([[m.cos(theta),0,m.sin(theta),0],
					 [0,1,0,0],
					 [-m.sin(theta),0,m.cos(theta),0],
					 [0,0,0,1]])
	if axis == "z":
		RTa = np.array([[m.cos(theta),-m.sin(theta),0,0],
					 [m.sin(theta),m.cos(theta),0,0],
					 [0,0,1,0],
					 [0,0,0,1]])
	return RTa
def RotAll(R):
	RTx = RotAxis("x",R[0])
	RTy = RotAxis("y",R[1])
	RTz = RotAxis("z",R[2])
	RT = multi_dot([RTx,RTy,RTz])
	return RT
def MovA(M):
	MT = np.array([[1,0,0,M[0][0]],
				   [0,1,0,M[1][0]],
				   [0,0,1,M[2][0]],
				   [0,0,0,1]])
	return MT
def Xlator(P1,M,R):
	#P2 = R*P1+MT
	P2 = multi_dot([MovA(M),RotAll(R),P1])
	return P2
def Xlator2x(P_L, M_gtob, R_gtob, M_btoL, R_btoL):
	#P_g = RT_gtob * RT_btoL * P_L + RT_btog * M_btoL + M_gtob
	P_g = np.add(np.add(multi_dot([RotAll(R_gtob),RotAll(R_btoL),P_L]),np.dot(RotAll(R_gtob),M_btoL)),M_gtob)
	return P_g
def Xlator3x(P_D, M_gtob, R_gtob, M_btoL, R_btoL,M_LtoD, R_LtoD):
	RT0 = RotAll(R_gtob)
	RT1 = RotAll(R_btoL)
	RT2 = RotAll(R_LtoD)
	M0 = M_gtob
	M1 = M_btoL
	M2 = M_LtoD
	#P_g = (RT0*RT1*RT2*P_D)+(RT0*RT1*M2)+(RT0*M1)+M0
	P_g = np.add(np.add(np.add(multi_dot([RT0,RT1,RT2,P_D]),multi_dot([RT0,RT1,M2])),multi_dot([RT0,M1])),M0)
	return P_g

#-----------------------Pygame Functions
def txt(msg,color,row,col,screen,font_style):
	pos = cord([-(WIDTH/2)+col*100,(HEIGHT/2)-row*15])
	mesg = font_style.render(msg, True, color)
	screen.blit(mesg, pos)
def txt2d(msg,color,pos,screen,font_style):
	mesg = font_style.render(msg, True, color)
	screen.blit(mesg, pos)
def cord(pos):
	scn_offset_x = WIDTH/2
	scn_offset_y = HEIGHT/2
	x=int(scn_offset_x+pos[0])
	y=int(-scn_offset_y+HEIGHT-pos[1])
	return [x,y]

def DrawxyzCoord(coordname,origin,xaxis,yaxis,zaxis,coordcolor,screen,font_style):
	#Translate coordinate to screen coordinates
	orgn= cord([origin[0][0],origin[1][0]])
	xax= cord([xaxis[0][0],xaxis[1][0]])
	yax= cord([yaxis[0][0],yaxis[1][0]])
	zax= cord([zaxis[0][0],zaxis[1][0]])
	
	#draw axes
	pygame.draw.line(screen,coordcolor,orgn,xax,2)
	pygame.draw.line(screen,coordcolor,orgn,yax,2)
	pygame.draw.line(screen,coordcolor,orgn,zax,2)
	
	#Axes and Origin labels
	txt2d("x",coordcolor,xax,screen,font_style)
	txt2d("y",coordcolor,yax,screen,font_style)
	txt2d("",coordcolor,zax,screen,font_style)
	txt2d(coordname,coordcolor,orgn,screen,font_style)
	
def DrawFoot(leg,pos,screen,footcolor,font_style):
	posxy = cord([pos[0][0],pos[1][0]])
	pygame.draw.rect(screen,footcolor,[posxy[0],posxy[1],8,8])
	txt2d("F"+str(leg),footcolor,posxy,screen,font_style)
	
def DrawLimitSphere(Radius,centerxyz,screen,limcolor):
	centerxy = cord([centerxyz[0][0],centerxyz[1][0]])
	pygame.draw.circle(screen,limcolor,centerxy,Radius,1)


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
	refresh_rate = 0.1
	proctime = 0
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
	
	#-----------------------Define Axes
	origin = np.array([[0],[0],[0],[1]])
	xaxis  = np.array([[1],[0],[0],[1]])
	yaxis  = np.array([[0],[1],[0],[1]])
	zaxis  = np.array([[0],[0],[1],[1]])
	
	#-----------------------Body Origin relative to global (Changed by Controller inputs)
	walkspeed = 2
	M_gtob = np.array([[50],[50],[0],[1]])
	R_gtob = [0,0,0]

	#-----------------------Leg Origins relative to Body (CONSTANT)
	BodyR = 100
	sqrt3 = m.sqrt(3)
	M_btoL1 = np.array([[BodyR*sqrt3/2],	[BodyR*1/2],	[0],[1]])
	M_btoL2 = np.array([[BodyR*0],			[BodyR*1],		[0],[1]])
	M_btoL3 = np.array([[BodyR*-sqrt3/2],	[BodyR*1/2],	[0],[1]])
	M_btoL4 = np.array([[BodyR*-sqrt3/2],	[BodyR*-1/2],	[0],[1]])
	M_btoL5 = np.array([[BodyR*0],			[BodyR*-1],		[0],[1]])
	M_btoL6 = np.array([[BodyR*sqrt3/2],	[BodyR*-1/2],	[0],[1]])

	R_btoL1 = [0,0,-60]
	R_btoL2 = [0,0,0]
	R_btoL3 = [0,0,60]
	R_btoL4 = [0,0,120]
	R_btoL5 = [0,0,180]
	R_btoL6 = [0,0,240]

	M_btoL = np.array([M_btoL1,M_btoL2,M_btoL3,M_btoL4,M_btoL5,M_btoL6])
	R_btoL = np.array([R_btoL1,R_btoL2,R_btoL3,R_btoL4,R_btoL5,R_btoL6])
	
	#-----------------------Foot Datum relative to Leg (CONSTANT)
	limcolor = black
	FootRangeRadius = 45
	FootDatumRadius = 40 #relative to leg origin
	M_LtoD = np.array([[0],[FootDatumRadius*1],[0],[1]])
	R_LtoD = [0,0,0]
	P_D = M_LtoD
	
	#-----------------------Draw feet
	zeros = np.array([[0],[0],[0],[0]])
	FootPos = np.array([zeros,zeros,zeros,zeros,zeros,zeros])
	for leg in range(6):
		FootPos[leg] = Xlator3x(P_D, M_gtob, R_gtob, M_btoL[leg], R_btoL[leg], M_LtoD, R_LtoD)
	
	#Main Loop
	while not exit_main:
		#Monitor Loop
		while controller_connected:
			if proctime > refresh_rate:
				proctime = refresh_rate
			time.sleep(refresh_rate-proctime)
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
			#Draw Global Coords:
			g_coordscale = 75
			g_xaxis = np.dot(MatScale(g_coordscale),xaxis)
			g_yaxis = np.dot(MatScale(g_coordscale),yaxis)
			g_zaxis = np.dot(MatScale(g_coordscale),zaxis)
			DrawxyzCoord("g",origin,g_xaxis,g_yaxis,g_zaxis,red,screen,font_style)
			
			
			#Update Body coord translators with controller inputs:
			M_gtob[0][0] = M_gtob[0][0] + int(walkspeed*psc.j_Lx.val)
			M_gtob[1][0] = M_gtob[1][0] - int(walkspeed*psc.j_Ly.val)
			R_gtob[1] = R_gtob[1] - int(walkspeed*psc.j_Ry.val)
			R_gtob[2] = R_gtob[2] - int(walkspeed*psc.j_Rx.val)
			txt("M_gtob: " + str(M_gtob),black,0,0,screen,font_style)
			txt("R_gtob: " + str(R_gtob),black,1,0,screen,font_style)
			
			#Draw Body Coords:
			b_coordscale = 50
			b_origin = Xlator(origin,M_gtob,R_gtob)
			b_xaxis = Xlator(np.dot(MatScale(b_coordscale),xaxis),M_gtob,R_gtob)
			b_yaxis = Xlator(np.dot(MatScale(b_coordscale),yaxis),M_gtob,R_gtob)
			b_zaxis = Xlator(np.dot(MatScale(b_coordscale),zaxis),M_gtob,R_gtob)
			DrawxyzCoord("b",b_origin,b_xaxis,b_yaxis,b_zaxis,seagreen,screen,font_style)
			
			#Leg Coords
			L_coordscale = 35
			for leg in range(6):
				axName = "L"+str(leg+1)
				L_origin= Xlator2x(origin,M_gtob,R_gtob,M_btoL[leg],R_btoL[leg])
				L_xaxis = Xlator2x(np.dot(MatScale(L_coordscale),xaxis),M_gtob,R_gtob,M_btoL[leg],R_btoL[leg])
				L_yaxis = Xlator2x(np.dot(MatScale(L_coordscale),yaxis),M_gtob,R_gtob,M_btoL[leg],R_btoL[leg])
				L_zaxis = Xlator2x(np.dot(MatScale(L_coordscale),zaxis),M_gtob,R_gtob,M_btoL[leg],R_btoL[leg])
				DrawxyzCoord(axName,L_origin,L_xaxis,L_yaxis,L_zaxis,blue,screen,font_style)
				#DrawLimits:
				DatumPos = Xlator3x(P_D, M_gtob, R_gtob, M_btoL[leg], R_btoL[leg], M_LtoD, R_LtoD)
				DrawLimitSphere(FootRangeRadius,DatumPos,screen,limcolor)
				#Draw Feet:
				FTdistVec = np.subtract(FootPos[leg],DatumPos) #foot distance from datum Vector
				FTdist = m.sqrt(((FTdistVec[0][0])**2)+((FTdistVec[1][0])**2)+((FTdistVec[2][0])**2)) #Scalar
				if FTdist > FootRangeRadius:
					FootPos[leg] = DatumPos
				DrawFoot(leg,FootPos[leg],screen,purple,font_style)
				
				
			
			
			toc = time.perf_counter()
			proctime = toc - tic
			if proctime > refresh_rate:
				txt("Lagging!",red,48,4,screen,font_style)
				#print(f"Main loop is lagging and took : {toc - tic:0.5f} seconds to complete.")
			txt("Refresh rate: "+ str(refresh_rate),black,47,0,screen,font_style)	
			txt(f"Main loop took : {proctime:0.6f} seconds to complete.",black,48,0,screen,font_style)
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
