import pygame, time, threading, sys, csv
from pygame.locals import *
from pygame import event
import numpy as np
from numpy.linalg import multi_dot
import math as m
from adafruit_servokit import ServoKit

#Pygame global color vars:
global purple, blue, red, white, black, seagreen, green, pink
purple=(100,0,255)
blue=(0,0,255)
red=(255,0,0)
white=(255,255,255)
black=(0,0,0)
seagreen=(32,178,170)
green=(0,128,0)
pink=(255,105,180)

#numpy print format
np.set_printoptions(formatter={'float': '{: 0.2f}'.format})

class pygame_screens:
	def __init__(self):
		#Pygame setup
		#self.HEIGHT = 1080
		#self.WIDTH = 1920
		self.HEIGHT = int(1080)
		self.WIDTH = int(1920/2)
		self.txtrow = int(self.HEIGHT/15)
		self.txtcol = int(self.WIDTH/100)
		self.lifescale = 1 #1 pixel = 1mm
		pygame.init()
		#self.screen_flags = pygame.FULLSCREEN
		self.screen_flags = pygame.RESIZABLE
		self.screen = pygame.display.set_mode((self.WIDTH, self.HEIGHT),self.screen_flags)
		pygame.display.set_caption("HexaPi")
		self.screen.fill(white)
		
		self.font_style = np.empty(20, dtype=object) 
		for font_size in range(20):
			self.font_style[font_size] = pygame.font.SysFont(None, font_size+20)
			
		self.pgtxt = np.empty([self.txtrow,self.txtcol], dtype=object)
		#set defaults
		for r in range(self.txtrow):
			for c in range(self.txtcol):
				self.pgtxt[r,c] = self.pgtxt_init()
		self.screenrunning = True
		
	def closescreen(self):
		self.screenrunning = False
		time.sleep(0.1)
		pygame.display.quit()
		pygame.init()
		
	def cord(self,pos):
		scn_offset_x = self.WIDTH/2
		scn_offset_y = self.HEIGHT/2
		x=int(scn_offset_x+self.lifescale*pos[0])
		y=int(-scn_offset_y+self.HEIGHT-self.lifescale*pos[1])
		return [x,y]
		
	def txt(self,msg,color,row,col,font_size):
		pos = [int(col*100),int(row*15)]
		mesg = self.font_style[font_size-20].render(msg, True, color)
		self.screen.blit(mesg, pos)
		
	class pgtxt_init:
		def __init__(self):
			self.msg = ""
			self.color = black
			self.font_size = 20
	
	def drawgptxt(self):
		for r in range(self.txtrow):
			for c in range(self.txtcol):
				pass
				if not self.pgtxt[r,c].msg == "":
					self.txt(self.pgtxt[r,c].msg,self.pgtxt[r,c].color,r,c,self.pgtxt[r,c].font_size)
		
	def txt2d(self,msg,color,pos,font_size):
		mesg = self.font_style[font_size-20].render(msg, True, color)
		self.screen.blit(mesg, pos)
		
	def DrawxyzCoord(self,coordname,origin,xaxis,yaxis,zaxis,coordcolor,font_size):
		#Translate coordinate to screen coordinates
		orgn= self.cord([origin[0][0],origin[1][0]])
		xax= self.cord([xaxis[0][0],xaxis[1][0]])
		yax= self.cord([yaxis[0][0],yaxis[1][0]])
		zax= self.cord([zaxis[0][0],zaxis[1][0]])
		
		#draw axes
		pygame.draw.line(self.screen,coordcolor,orgn,xax,2)
		pygame.draw.line(self.screen,coordcolor,orgn,yax,2)
		pygame.draw.line(self.screen,coordcolor,orgn,zax,2)
		
		#Axes and Origin labels
		self.txt2d("x",coordcolor,xax,font_size)
		self.txt2d("y",coordcolor,yax,font_size)
		self.txt2d("",coordcolor,zax,font_size)
		self.txt2d(coordname,coordcolor,orgn,font_size)
	
	def DrawFoot(self,leg,pos,footcolor,font_size):
		posxy = self.cord([pos[0][0],pos[1][0]])
		pygame.draw.rect(self.screen,footcolor,[posxy[0],posxy[1],8,8])
		self.txt2d("F"+str(leg),footcolor,posxy,font_size)
	
	def DrawLimitSphere(self,Radius,centerxyz,limcolor):
		centerxy = self.cord([centerxyz[0][0],centerxyz[1][0]])
		pygame.draw.circle(self.screen,limcolor,centerxy,self.lifescale*Radius,1)

def pygame_screen_loop():
	#Local Variables
	screen_FPS = 30
	screen_refresh_rate = (1/screen_FPS)
	proctime = 0
	hxsim = hexapi_simulator()
	
	#global Variable
	global pgs, exit_screen
	pgs = pygame_screens()
	
	PGrow = 56 #Pygame loop status
	pgs.pgtxt[PGrow,0].msg = "Starting pygame_screen_loop thread"
	pgs.pgtxt[PGrow+1,0].msg = "Pygame loop FPS: " + str(screen_FPS)
	
	while True:
		pgs.screen.fill(white)	
		pgs.pgtxt[PGrow,0].msg = "Loading...."
		pgs.pgtxt[PGrow+1,0].msg = "Waiting for Main Loop"
		pygame.display.update()
		time.sleep(0.25)
		while not exit_main:
			#Re-initialize screen if not running but should be:
			if (exit_screen == False) and (pgs.screenrunning == False):
				pgs = pygame_screens()
				
			#Quit screen if running but shouldn't be:
			if (exit_screen == True) and (pgs.screenrunning == True):
				pgs.closescreen()
				
			#Wait for controller to tell screen to start:
			if (exit_screen == True) and (pgs.screenrunning == False):
				time.sleep(0.25)
			
			while (exit_screen == False) and (pgs.screenrunning == True):
				if proctime > screen_refresh_rate:
					proctime = screen_refresh_rate
				time.sleep(screen_refresh_rate-proctime)
				tic = time.perf_counter()
				pgs.screen.fill(white)
				
				#draw all the pgtxt's
				pgs.drawgptxt()
				
				if mods.Sim_Walk_module == "RUN":
					pgs.txt("Press the Triangle to exit Walk Simulator",green,0,0,30)
					hxsim.refresh_sim()
				
				#All Stopped
				if mods.allstopped:
					pgs.txt("Press the PS button for 1 second to start Exit Program",black,0,0,30)
					pgs.txt("Press the Triangle button to start Walk Simulator",green,2,0,30)
					pgs.txt("Press the Circle button to start x",red,4,0,30)
					pgs.txt("Press the Cross button to start x",blue,6,0,30)
					pgs.txt("Press the Square button to start x",pink,8,0,30)
				
				toc = time.perf_counter()
				proctime = toc - tic
				procmsg = f"pygame_screen_loop loop took : {proctime:0.6f} seconds to complete."
				pgs.pgtxt[PGrow,0].msg = procmsg
				if proctime > screen_refresh_rate:
					pgs.txt("Lagging!",red,PGrow,4,30)
				
				pygame.display.update()
	print("End of pygame_screen_loop thread")
#-----------------------Pygame Classes
class hexapi_simulator:
	def __init__(self):
		#-----------------------Define Axes for pygame
		self.origin = np.array([[0],[0],[0],[1]])
		self.xaxis  = np.array([[1],[0],[0],[1]])
		self.yaxis  = np.array([[0],[1],[0],[1]])
		self.zaxis  = np.array([[0],[0],[1],[1]])

		#Define Global Coords
		self.g_coordscale = 75 #mm
		self.g_xaxis = np.dot(MatScale(self.g_coordscale),self.xaxis)
		self.g_yaxis = np.dot(MatScale(self.g_coordscale),self.yaxis)
		self.g_zaxis = np.dot(MatScale(self.g_coordscale),self.zaxis)

		#Draw Body Coords:
		self.b_coordscale = 50 #mm

		#Leg Coords
		self.L_coordscale = 35 #mm
		
	def refresh_sim(self):
		#Draw Global
		pgs.DrawxyzCoord("g",self.origin,self.g_xaxis,self.g_yaxis,self.g_zaxis,red,20)
		
		#Draw Body Coords:
		b_origin = Xlator(self.origin,hx.M_gtob,hx.R_gtob)
		b_xaxis = Xlator(np.dot(MatScale(self.b_coordscale),self.xaxis),hx.M_gtob,hx.R_gtob)
		b_yaxis = Xlator(np.dot(MatScale(self.b_coordscale),self.yaxis),hx.M_gtob,hx.R_gtob)
		b_zaxis = Xlator(np.dot(MatScale(self.b_coordscale),self.zaxis),hx.M_gtob,hx.R_gtob)
		pgs.DrawxyzCoord("b",b_origin,b_xaxis,b_yaxis,b_zaxis,seagreen,20)
		
		for leg in range(6):
			#Draw Leg Coords
			axName = "L"+str(leg+1)
			L_origin= Xlator2x(self.origin,hx.M_gtob,hx.R_gtob,hx.M_btoL[leg],hx.R_btoL[leg])
			L_xaxis = Xlator2x(np.dot(MatScale(self.L_coordscale),self.xaxis),hx.M_gtob,hx.R_gtob,hx.M_btoL[leg],hx.R_btoL[leg])
			L_yaxis = Xlator2x(np.dot(MatScale(self.L_coordscale),self.yaxis),hx.M_gtob,hx.R_gtob,hx.M_btoL[leg],hx.R_btoL[leg])
			L_zaxis = Xlator2x(np.dot(MatScale(self.L_coordscale),self.zaxis),hx.M_gtob,hx.R_gtob,hx.M_btoL[leg],hx.R_btoL[leg])
			pgs.DrawxyzCoord(axName,L_origin,L_xaxis,L_yaxis,L_zaxis,blue,20)
			
			#Draw Limit sphere about datum
			D_origin = Xlator3x(self.origin, hx.M_gtob, hx.R_gtob, hx.M_btoL[leg], hx.R_btoL[leg], hx.M_LtoD, hx.R_LtoD)
			pgs.DrawLimitSphere(hx.FootRangeRadius,D_origin,black)
			
			#Draw Feet
			pgs.DrawFoot(leg,hx.F_g[leg],purple,20)

#-----------------------Robot classes
class hexapi_robot:
	def __init__(self):
		self.Lc = 152 #mm		length_claw
		self.La = 87.2 #mm		length_arm
		self.Ls = 57.5 #mm		length_shoulder
		
		#Control perspective
		self.POV = "Global"
		self.read_in_gaits()
		self.t = 0
		
		#-----------------------Body origin relative to global origin (Changed by Controller inputs)
		self.strideboost= 1
		self.spinboost = 1
		self.M_gtob = np.array([[50],[50],[0],[1]]) #mm
		self.R_gtob = [0,0,0] #degrees
		
		#-----------------------Leg origins relative to Body origin (CONSTANT)
		self.BodyR = 101 #mm
		sqrt3 = m.sqrt(3)
		M_btoL1 = np.array([[self.BodyR*sqrt3/2],	[self.BodyR*1/2],	[0],[1]])
		M_btoL2 = np.array([[self.BodyR*0],			[self.BodyR*1],		[0],[1]])
		M_btoL3 = np.array([[self.BodyR*-sqrt3/2],	[self.BodyR*1/2],	[0],[1]])
		M_btoL4 = np.array([[self.BodyR*-sqrt3/2],	[self.BodyR*-1/2],	[0],[1]])
		M_btoL5 = np.array([[self.BodyR*0],			[self.BodyR*-1],		[0],[1]])
		M_btoL6 = np.array([[self.BodyR*sqrt3/2],	[self.BodyR*-1/2],	[0],[1]])
		R_btoL1 = [0,0,-60]
		R_btoL2 = [0,0,0]
		R_btoL3 = [0,0,60]
		R_btoL4 = [0,0,120]
		R_btoL5 = [0,0,180]
		R_btoL6 = [0,0,240]
		self.M_btoL = np.array([M_btoL1,M_btoL2,M_btoL3,M_btoL4,M_btoL5,M_btoL6])  #mm
		self.R_btoL = np.array([R_btoL1,R_btoL2,R_btoL3,R_btoL4,R_btoL5,R_btoL6])  #degrees
		
		#-----------------------Foot Datum origins relative to Leg origins (CONSTANT)
		self.FootRangeRadius = 60 #mm
		self.FootDatumRadius = 110 #mm
		self.M_LtoD = np.array([[0],[self.FootDatumRadius],[0],[1]])  #mm
		self.R_LtoD = [0,0,0] #degrees
		
		#-----------------------Foot positions relative to global origin (initial same as datum origin)
		zeros = np.array([[0],[0],[0],[1]])
		self.F_g = np.array([zeros,zeros,zeros,zeros,zeros,zeros])
		for leg in range(6):
			self.F_g[leg] = Xlator3x(zeros, self.M_gtob, self.R_gtob, self.M_btoL[leg], self.R_btoL[leg], self.M_LtoD, self.R_LtoD)  #mm
	
	def read_in_gaits(self):
		print("Importing gait...")
		gaitpath = "gaits/GaitArray.csv"
		global gait_array
		with open(gaitpath, newline='') as csvfile:
			data = list(csv.reader(csvfile))
		data.pop(0)
		self.gait_array = np.array(data)
		self.gait_array_shape = self.gait_array.shape
		self.gait_array_rows = self.gait_array_shape[0]-1
		
	def gait_next(self,rate):
		if (self.t + int(rate)) > self.gait_array_rows:
			self.t = 0
		else:
			self.t = self.t + int(rate)
		
	#functions
	def move(self,ctr_move,ctr_rot):
		if self.POV == "HexaPi":
			ctr_move = np.dot(RotAll(self.R_gtob),ctr_move)
		
		#update body origin
		

		

		
		#update leg origins
		for leg in range(6):
			if self.gait_array[self.t][leg] != 0:
				self.F_g[leg] = Xlator(self.F_g[leg],np.dot(MatScale(2),ctr_move),np.dot(2,ctr_rot))
		
		#update body origin
		self.M_gtob = np.dot(MovA(ctr_move),self.M_gtob)
		self.R_gtob = np.add(self.R_gtob,ctr_rot)
class hexapi_servos:
	def __init__(self):
		#print("HexaPi Robot object initializing")
		self.pwm = [0]*2
		self.pwm[0] = ServoKit(channels=16, address=0x40)
		self.pwm[1] = ServoKit(channels=16, address=0x41)
		self.pwm[0].frequency = 50
		self.pwm[1].frequency = 50
		self.config_filename = "new_servo_config.csv"
		self.prop_names = [ "#0:leg (1 to 6)",
							"#1:part (1=shoulder; 2=arm; 3=claw)",
							"#2:hat",
							"#3:channel",
							"#4:lower limit pwm",
							"#5:upper limit pwm",
							"#6:offset",
							"#7:lower limit physical",
							"#8:upper limit physical"]
		with open(self.config_filename,newline='') as csvfile:
			arr = list(csv.reader(csvfile))
		self.motor = [[0 for x in range(4)] for y in range(7)]
		for r in range(1,19):
			#motor[leg][part]
			self.motor[int(arr[r][0])][int(arr[r][1])] = self.motor_class(r, arr, self.pwm)
	class motor_class():
		def __init__(self,r,arr,pwm):
			self.pwm = pwm
			self.leg = int(arr[r][0])
			self.part = int(arr[r][1])
			self.hat = int(arr[r][2])
			self.chan = int(arr[r][3])
			self.LLpwm = int(arr[r][4])
			self.ULpwm = int(arr[r][5])
			self.offset = int(arr[r][6])
			self.LLphyang = int(arr[r][7])
			self.ULphyang = int(arr[r][8])
			self.pwm[self.hat].servo[self.chan].set_pulse_width_range(self.LLpwm,self.ULpwm)
			
		def move(self, angle):
			angle = angle + self.offset
			if angle >= self.LLphyang and angle <= self.ULphyang:
				self.pwm[self.hat].servo[self.chan].angle = angle
			else:
				pass
				#print("angle " + str(angle) + " out of bounds")
	def config_edit(self, prop, leg, part, val):
		csvfile = pandas.read_csv(self.config_filename)
		row = (leg-1)*3+part-1
		csvfile.loc[row,prop] = csvfile.loc[row,prop] + val
		csvfile.to_csv(self.config_filename,index=False)
		print("Prop " + str(prop) + " set to: " + str(csvfile.loc[row,prop]))
		self.__init__()

#-----------------------Controller Classes
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
		functions = ["Sim_Walk_module", "Walk_module"]
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

#-----------------------Controller Functions
def controller_connection():
	global controller_connected
	global jsc
	global psc
	global exit_main

	pygame.init()
	pygame.display.init()
	idle_wait = 20
	statrow = 59 #pygame text row to print controller status on
	pgs.pgtxt[statrow,0].msg = "Starting controller_connection thread"
	
	while True:
		time.sleep(2)
		if not exit_main:
			attempts = 0
			controller_count = 0
			prev_psc = None
			pgs.pgtxt[statrow,0].msg = "Checking if controller_connected True or False"
			while not controller_connected:
				pygame.joystick.init()
				attempts = attempts + 1
				controller_count = pygame.joystick.get_count()
				contmsg = "\rController count: " + str(controller_count) + " Scanning for Controllers... Attempt " + str(attempts)
				pgs.pgtxt[statrow,0].msg = contmsg
				#sys.stdout.write(msg)
				#sys.stdout.flush()
				if controller_count > 0:
					for i in range(controller_count):
						pygame.joystick.init()
						jsc = controller_class(i)
						contmsg = "\nController found: " + jsc.ctrl_name
						pgs.pgtxt[statrow,0].msg = contmsg
						#print(contmsg)
						if jsc.supported == True:
							contmsg = "Controller connected: " + jsc.ctrl_name
							pgs.pgtxt[statrow,0].msg = contmsg
							#print(contmsg)
							psc = psc_update(prev_psc, refresh_rate, jsc)
							jid_connected = pygame.joystick.Joystick(i).get_id()
							controller_connected = True
							break
						else:
							controller_connected = False
							contmsg = "Controller not supported: " + jsc.ctrl_name
							pgs.pgtxt[statrow,0].msg = contmsg
							#print(contmsg)
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
					contmsg = "\rController idle for " + str(idle_time) + ". Checking connection status"
					pgs.pgtxt[statrow,0].msg = contmsg
					#sys.stdout.write(contmsg)
					#sys.stdout.flush()
					pygame.joystick.quit()
					pygame.joystick.init()
					try:
						jid = pygame.joystick.Joystick(jid_connected).get_id()
						jsc = controller_class(jid)
						contmsg = "Controller idle for " + str(idle_time) + "+ seconds but still connected: " + jsc.ctrl_name
						pgs.pgtxt[statrow,0].msg = contmsg
						#print(contmsg)
						time.sleep(idle_wait)
					except:
						contmsg = "\nController lost connection: " + jsc.ctrl_name
						pgs.pgtxt[statrow,0].msg = contmsg
						#print(contmsg)
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

#-----------------------Main Program
def hexapi_main():
	#Global Controller Variables:
	global psc, jsc, controller_connected, mods, walk_idle
	psc = None
	jsc = None
	controller_connected = False
	mods = module_select()
	walk_idle = 0
	
	#Main Loop Variables:
	global exit_main, refresh_rate
	exit_main = False
	refresh_rate = 0.05
	proctime = 0
	
	#Robot Setup
	global hs, hx
	hs = hexapi_servos()
	hx = hexapi_robot()
	
	#Define pygame_screen_loop thread
	global pgs, exit_screen
	pgs = None
	exit_screen = False
	pygame_screen_thread = threading.Thread(target=pygame_screen_loop,args=())
	pygame_screen_thread.setDaemon(True)
	pygame_screen_thread.start()
	time.sleep(1) #give thread a seconds to init global object pgs
	
	#Define controller_connection thread
	controller_connection_thread = threading.Thread(target=controller_connection,args=())
	controller_connection_thread.setDaemon(True)
	controller_connection_thread.start()
	
	#Pygame status defaults
	global MLrow, Modrow,Ctrrow
	MLrow = 50 #main loop status
	pgs.pgtxt[MLrow+1,0].msg = "Main loop refresh rate limiter: " + str(refresh_rate)
	Modrow = 40 #Module status
	pgs.pgtxt[Modrow,0].msg = "All modules in STOP mode"
	Ctrrow = 30 #Control info
	pgs.pgtxt[Ctrrow,0].msg = "Control Info"
	pgs.pgtxt[Ctrrow+1,0].msg = "Control POV: " + hx.POV
	
	#Main Loop
	while not exit_main:
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
			toc = time.perf_counter()
			proctime = toc - tic
			procmsg = f"Main loop took : {toc - tic:0.5f} seconds to complete."
			pgs.pgtxt[MLrow,0].msg = procmsg
		pgs.pgtxt[MLrow,0].msg = "Main loop is waiting for a controller to be connected"
		time.sleep(0.1)
	print("End of hexapi_main code.")

def main_input_monitor():
	global exit_main, controller_connected, exit_screen
	global POV_msg, POV_Toggle, POV_XY, POV_Rot
	
	#hexapi_main Exit
	if psc.b_ps.pds == 1:
		modmsg = "PS Logo button pressed for 1 seconds. Exiting Monitor Loop..."
		pgs.pgtxt[Modrow,0].msg = modmsg
		print(modmsg)
		controller_connected = False
		exit_main = True
		exit_screen = True
	
	#Open/Close Pygame Screen
	if psc.b_share.pds == 1:
		print("Share button pressed for 1 seconds.")
		if pgs.screenrunning == True:
			modmsg = "Exiting Pygame Screen..."
			pgs.pgtxt[Modrow,0].msg = modmsg
			print(modmsg)
			exit_screen = True
		if pgs.screenrunning == False:
			modmsg = "Starting Pygame Screen..."
			print(modmsg)
			exit_screen = False
	
	#Down button
	#Change POV
	if psc.b_down.pressed == 1:
		if hx.POV == "Global":
			hx.POV = "HexaPi"
		else:
			hx.POV = "Global"
		pgs.pgtxt[Ctrrow+1,0].msg = "Control POV: " + hx.POV
	
	#Cross
	#Walk_module Run or Not
	if psc.b_cross.pressed == 1:
		mods.start_stop("Walk_module")
	if mods.Walk_module == "RUN":
		pgs.pgtxt[Modrow,0].msg = "Walk_module set to RUN"
		update_hexapi_math()
	
	#Triangle
	#Sim_Walk_module Run or Not
	if psc.b_tri.pressed == 1:
		mods.start_stop("Sim_Walk_module")
	if mods.Sim_Walk_module == "RUN":
		pgs.pgtxt[Modrow,0].msg = "Sim_Walk_module set to RUN"
		update_hexapi_math()

def update_hexapi_math():
	global walk_idle, t
	#check how long inputs have been idle
	if (psc.j_Rx.val == 0) and (psc.j_Lx.val == 0) and (psc.j_Ly.val == 0):
		walk_idle = walk_idle + 1
		ctr_move = np.array([[0],[0],[0],[1]])
		ctr_rot = [0,0,0]
	else:
		walk_idle = 0
		#Update Body coord translators with controller inputs:
		#Controller Vals
		ctr_move = np.dot(MatScale(hx.strideboost),np.array([[psc.j_Lx.val],[-psc.j_Ly.val],[0],[1]]))
		ctr_rot = [0,0,hx.spinboost*-psc.j_Rx.val]
		
		#Update hx object
		hx.move(ctr_move,ctr_rot)
		hx.gait_next(1)
		
		pgs.pgtxt[Ctrrow+4,0].msg = "Walking..."
	
	#if idle time exceeds x seconds, then stand still
	walk_idle_time = round(walk_idle*refresh_rate,2)
	if walk_idle_time > 6:
		idlemsg = "Robot has been idle for: " + str(walk_idle_time)
		pgs.pgtxt[Ctrrow+4,0].msg = idlemsg
	pgs.pgtxt[Ctrrow+2,0].msg = "last ctr_move: " + str(ctr_move)
	pgs.pgtxt[Ctrrow+3,0].msg = "last ctr_rot: " + str(ctr_rot)

#Start program
hexapi_main()
