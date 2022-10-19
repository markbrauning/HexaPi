import pygame, time, math, threading, sys, os, csv, pandas
from pygame.locals import *
from pygame import event
from adafruit_servokit import ServoKit

global exit_main
global exit_Servo_Config
global controller_connected
global hx
global psc
global jsc
global exit_main
global controller_connected
global refresh_rate
global exit_Servo_Config
global leg, part, prop
global controller_connected
global jsc
global psc

#Global Variables
hx = None
psc = None
Servo_Config_Module_thread = None

def read_in_gaits():
	print("Importing gaits...")
	listofgates = [ "gaits/gait_0.csv",
					"gaits/gait_1.csv",
					"gaits/gait_2.csv",
					"gaits/gait_3.csv"]
	global gait_array
	global gait_array_size
	#                 leg1    leg2    leg3    leg4    leg5    leg6
	#                tz,txy  tz,txy  tz,txy  tz,txy  tz,txy  tz,txy 
	gait_array =   [[[], [], [], [], [], [], [], [], [], [], [], []], #gait option 0
					[[], [], [], [], [], [], [], [], [], [], [], []], #gait option 1
					[[], [], [], [], [], [], [], [], [], [], [], []], #gait option 2
					[[], [], [], [], [], [], [], [], [], [], [], []]] #gait option 3
	gait_array_size = [0]*4
	
	for i in range(4):
		with open(listofgates[i]) as csv_file:
			csv_reader = csv.reader(csv_file, delimiter=',')
			row_count = 0
			for row in csv_reader:
				col_count = 0
				gait_array_col = 0
				for col_entry in row:
					if (row_count != 0) and ((col_count%3 != 0)):
						gait_array[i][gait_array_col].append(col_entry)
						gait_array_col += 1
					col_count += 1  
				row_count += 1
		print(listofgates[i] + " : Processed %s rows." % (row_count))
		gait_array_size[i] = row_count - 1

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

class hexapi_robot:
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
		self.l1 = 7.75 #old length_arm
		self.l2 = 14.5 #old length_claw
		self.l3 = 5.75 #old length_shoulder
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

class module_select:
	def __init__(self):
		functions = ["walk_module","stand","servo_config","single_leg"]
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

def hexapi_main():
	#Global Variables:
	global hx
	global psc
	global jsc
	global exit_main
	global controller_connected
	global refresh_rate
	global leg, part, prop
	global test_angle
	global mods
	global foot_vectors
	global x_datum, y_datum, z_datum
	global x, y, z
	global foot_coordinates
	global stride_scale
	global stride_speed
	global walk_idle
	global gait_selected
	global t
	
	#Global Variable Def
	hx = hexapi_robot()
	psc = None
	jsc = None
	exit_main = False
	controller_connected = False
	refresh_rate = 0.01
	leg = 1
	part = 3
	prop = 6
	test_angle = 90
	mods = module_select()
	foot_vectors = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
	x_datum = 0
	y_datum = 5
	z_datum = -19
	x = x_datum
	y = y_datum
	z = z_datum
	#                      leg1     2       3       4       5       6
	#                    [x,y,z] [x,y,z] [x,y,z] [x,y,z] [x,y,z] [x,y,z]
	foot_coordinates  = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
	stride_scale = 2
	stride_speed = 0
	walk_idle = 0
	gait_selected = 0
	t = 0
	
	#Read in Gait Array
	read_in_gaits()
	
	#Define controller_connection thread
	controller_connection_thread = threading.Thread(target=controller_connection,args=())
	controller_connection_thread.setDaemon(True)
	controller_connection_thread.start()
	
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
			
			toc = time.perf_counter()
			if (toc - tic) > refresh_rate:
				pass
				#print(f"Main loop is lagging and took : {toc - tic:0.5f} seconds to complete.")
		time.sleep(0.5)
	print("End of hexapi_main code.")

def calc_foot_vectors(Qx,Qy,Qr):
	gfv=[[0,0],[0,0],[0,0],[0,0],[0,0],[0,0]]
	xlation_angle=[5.2360,0,1.0472,0.2122,3.1416,4.1888]

	g_datum = [[-20.14,11.63],[0,23.25],[20.14,11.63],[20.14,-11.63],[0,-23.25],[-20.14,-11.63]]
	Qxy_mag = (((g_datum[0][0])**2)+((g_datum[0][1])**2))**(0.5)

	body_rotation_val = (Qr)/(2*Qxy_mag)
	#fix acos domain error
	if (body_rotation_val<=-1):
		body_rotation_val = -0.999999
	if (body_rotation_val>=1):
		body_rotation_val = 0.999999
	body_rotation = 2*math.asin(body_rotation_val)

	for leg in range(6):
		Qrx = g_datum[leg][0]*math.cos(body_rotation) - g_datum[leg][1]*math.sin(body_rotation)
		Qry = g_datum[leg][0]*math.sin(body_rotation) + g_datum[leg][1]*math.cos(body_rotation)
		gfv[leg][0]= (Qx + Qrx)
		gfv[leg][1]= (Qy + Qry)
		foot_vectors[leg][0] = round((gfv[leg][0]-g_datum[leg][0])*math.cos(xlation_angle[leg]) - (gfv[leg][1]-g_datum[leg][1])*math.sin(xlation_angle[leg]),2)
		foot_vectors[leg][1] = round((gfv[leg][0]-g_datum[leg][0])*math.sin(xlation_angle[leg]) + (gfv[leg][1]-g_datum[leg][1])*math.cos(xlation_angle[leg]),2)
		foot_vectors[leg][2] = round(((foot_vectors[leg][0]**2)+(foot_vectors[leg][1]**2))**(0.5),2)
	return foot_vectors

def calc_foot_xyz(i, foot_vectors):
	for leg in range(6):
		#access z column of gait csv and multiply it by the magnitude of the foot vector for that leg
		foot_coordinates[leg][2] = float(gait_array[gait_selected][leg*2][i])*foot_vectors[leg][2]*stride_scale + z_datum

		#access the xy column of the gait csv and multiply it by the x component of the foot vector
		foot_coordinates[leg][0] = float(gait_array[gait_selected][leg*2+1][i])*foot_vectors[leg][0]*stride_scale + x_datum

		#access the xy column of the gait csv and multiply it by the y component of the foot vector
		foot_coordinates[leg][1] = float(gait_array[gait_selected][leg*2+1][i])*foot_vectors[leg][1]*stride_scale + y_datum
	return foot_coordinates

def fixAcos(val):
	if (val<=-1):
		val = -1
	if (val>=1):
		val = 1
	return val

def xyz2angles(coords):
	#Takes in the desired XYZ components for each leg and outputs the end angles
	#[beta,theta,alpha] ie shoulder,arm,claw
	x = coords[0]
	y = coords[1]
	z = coords[2]
	
	servo_angles = [0,0,0]
	if(x == 0):
		x = 0.00001
	xy = ((x**2)+(y**2))**(0.5)
	c = (((xy-hx.l3)**2)+(z**2))**(0.5)
	if c == 0:
		c = 0.00001

	#Calculate alpha
	alpha_acos_piece = fixAcos(((hx.l1**2)+(hx.l2**2)-(c**2))/(2*hx.l1*hx.l2))
	alpha = math.acos(alpha_acos_piece) - (math.pi/2)

	#Calculate theta
	theta_acos_piece = fixAcos(((hx.l1**2)+(c**2)-(hx.l2**2))/(2*hx.l1*c))
	theta_acos_piece1 = fixAcos(z/c)
	if(xy<=hx.l3):
		theta_piece1 = - math.acos(theta_acos_piece1)
	else:
		theta_piece1 = math.acos(theta_acos_piece1)
	theta = theta_piece1 - math.acos(theta_acos_piece)

	#Calculate beta
	beta_acos_piece = fixAcos(x/xy)
	beta = math.pi - math.acos(beta_acos_piece)

	if (beta > math.pi):
		beta = math.pi
	if (alpha > math.pi):
		alpha = math.pi
	if (theta > math.pi):
		theta = math.pi

	if (beta < 0):
		beta = 0
	if (alpha < 0):
		alpha = 0
	if (theta < 0):
		theta = 0
		
	servo_angles = [0,round(beta*(180/math.pi),0),round(theta*(180/math.pi),0),round(alpha*(180/math.pi),0)]
	return servo_angles

def walk_module():
	global walk_idle
	global gait_selected
	global gait_select_size
	global t
	global foot_vectors
	global stride_scale
	global stride_speed
	
	#Check for gait selection change
	if (psc.b_L.pressed == 1) or (psc.b_R.pressed == 1):
			if psc.b_R.pressed == 1:
				if gait_selected == 3:
					gait_selected = 0
				else:
					gait_selected = gait_selected + 1
			if psc.b_L.pressed == 1:
				if gait_selected == 0:
					gait_selected = 3
				else:
					gait_selected = gait_selected - 1
			print("Gait Selection changed to: " + str(gait_selected))
	
	#if controller inputs change then caluclate foot vectors
	if (psc.j_Rx.val_changed == 1) or (psc.j_Lx.val_changed == 1) or (psc.j_Ly.val_changed == 1):
		foot_vectors = calc_foot_vectors(psc.j_Lx.val,psc.j_Ly.val,psc.j_Rx.val)
		stride_speed = abs(round(psc.j_Rx.val + psc.j_Lx.val + psc.j_Ly.val,0))
		#msg = "\r	Foot Vector: " + str(Vectors)
		#sys.stdout.write(msg)
		#sys.stdout.flush()
	
	#check how long inputs have been idle
	if (psc.j_Rx.val == 0) and (psc.j_Lx.val == 0) and (psc.j_Ly.val == 0):
		walk_idle = walk_idle + 1
	else:
		walk_idle = 0
	walk_idle_time = round(walk_idle* refresh_rate,2)
	
	#if idle time exceeds x seconds, then stand still
	if walk_idle_time > 8:
		msg = "\rIdle walk seconds: " + str(walk_idle_time)
		sys.stdout.write(msg)
		sys.stdout.flush()
		#write code to return to standing stance?
		pass
	else:
		if stride_speed != 0:
			t = int(t + stride_speed)
			if t == gait_array_size[gait_selected]:
				t = 0
			foot_coordinates = calc_foot_xyz(t, foot_vectors)
			for leg in range(6):
				servo_angles = xyz2angles(foot_coordinates[leg])
				for pt in range(1,4):
					hx.motor[leg+1][pt].move(servo_angles[pt])

def single_leg_module():
	global leg, x, y, z
	if psc.b_R.pressed == 1:
		if leg == 6:
			leg = 1
		else:
			leg = leg + 1
		print("Leg selected: " + str(leg))
	if (psc.j_Ry.val != 0) or (psc.j_Lx.val != 0) or (psc.j_Ly.val != 0):
		x = round(x + 0.1*psc.j_Lx.val,1)
		y = round(y + 0.1*psc.j_Ly.val,1)
		z = round(z + 0.1*psc.j_Ry.val,1)
		leg_angles = xyz2angles([x,y,z])
		msgxyz = [x,y,z]
		msg = "\r    leg for x,y,z: " + str(msgxyz) + " angles set to: " + str(leg_angles)
		sys.stdout.write(msg)
		sys.stdout.flush()
		for pt in range(1,4):
			hx.motor[leg][pt].move(leg_angles[pt])

def servo_config_module():
	global leg, part, prop, test_angle
	if psc.b_R.pressed == 1:
		if leg == 6:
			leg = 1
		else:
			leg = leg + 1
		print("Leg selected: " + str(leg))
	if psc.b_up.pressed == 1:
		if part == 3:
			part = 1
		else:
			part = part + 1
		print("part selected: " + str(part))
	if psc.b_down.pressed == 1:
		if prop == 8:
			prop = 4
		else:
			prop = prop + 1
		print("Config edit property set to: " + hx.prop_names[prop])
	
	if prop == 4 or prop == 5:
		val = 5
	else:
		val = 1	
	if psc.b_L1.pressed == 1:
		hx.config_edit(hx.prop_names[prop], leg, part, -val)
		hx.motor[leg][part].move(test_angle)
	if psc.b_R1.pressed == 1:
		hx.config_edit(hx.prop_names[prop], leg, part, val)
		hx.motor[leg][part].move(test_angle)
	
	if psc.j_Lx.val_changed == 1:
		test_angle = 90 + int(float(psc.j_Lx.val) * 90)
		test_angle_str = str(test_angle).zfill(3)
		msg = "\r    Test angle set to: " + test_angle_str
		sys.stdout.write(msg)
		sys.stdout.flush()
		hx.motor[leg][part].move(test_angle)

def main_input_monitor():
	global exit_main
	global controller_connected
	global x, y, z
	global mods
	
	#Print button pessed name
	#for i in range(jsc.btn_num):
	#	if psc.btn[i].pressed == 1:
	#		print(psc.btn[i].name)
	
	#hexapi_main Exit
	if psc.b_ps.pds == 1:
		print("PS Logo button pressed for 1 seconds. Exiting Monitor Loop...")
		controller_connected = False
		exit_main = True
	
	#Stand
	#
	
	#Lay Down
	#
	
	#Walk
	#walk_module if no special modules set to RUN
	if mods.allstopped == True:
		walk_module()
	
	#Triangle
	#Servo_Config_Module Run or Not
	if psc.b_tri.pressed == 1:
		mods.start_stop("servo_config")
	if mods.servo_config == "RUN":
		servo_config_module()
	
	#Circle
	#single_leg_module Run or Not
	if psc.b_circle.pressed == 1:
		mods.start_stop("single_leg")
	if mods.single_leg == "RUN":
		single_leg_module()
	
#Start program
hexapi_main()












