import pygame, time, math, threading, sys, os, csv
from pygame.locals import *
from pygame import event
from adafruit_servokit import ServoKit

class psc_update:
	def __init__(self, controller, prev_psc, refresh_rate):
		events = pygame.event.get()
		
		#Buttons:
		self.btn = [0]*controller.get_numbuttons()
		for i in range(controller.get_numbuttons()):
			self.btn[i] = self.button(i, controller, prev_psc, refresh_rate)
		self.b_cross = self.btn[0]	#0 Cross
		self.b_circle = self.btn[1]	#1 Circle
		self.b_tri = self.btn[2]	#2 Triangle
		self.b_sq= self.btn[3]		#3 Square
		self.b_L1 = self.btn[4]		#4 L1
		self.b_R1 = self.btn[5]		#5 R1
		self.b_L2 = self.btn[6]		#6 L2
		self.b_R2 = self.btn[7]		#7 R2
		self.b_select = self.btn[8]	#8 Select
		self.b_start = self.btn[9]	#9 Start
		self.b_ps = self.btn[10]	#10 Logo
		self.b_Lj = self.btn[11]	#11 Left joystick
		self.b_Rj = self.btn[12]	#12 Right joystick
		self.b_up = self.btn[13]	#13 d-pad up
		self.b_down = self.btn[14]	#14 d-pad down
		self.b_L = self.btn[15]		#15 d-pad left
		self.b_R = self.btn[16]		#16 d-pad right
		
		#Axes:
		accuracy = 2
		self.ax = [0]*controller.get_numaxes()
		for i in range(controller.get_numaxes()):
			self.ax[i] = self.axis(i, accuracy, controller)
		self.j_Lx = self.ax[0]		#0 Left Joystick x-axis
		self.j_Ly = self.ax[1]		#1 Left Joystick y-axis
		self.j_L2 = self.ax[2]		#2 L2 axis
		self.j_Rx = self.ax[3]		#3 Right Joystick x-axis
		self.j_Ry = self.ax[4]		#4 Right Joystick y-axis
		self.j_R2 = self.ax[5]		#5 R2 axis
		
	class button:
		def __init__(self, index, controller, prev_psc, refresh_rate):
			self.state = controller.get_button(index)
			self.index = index
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
				self.state_changed = 0
	
	class axis:
		def __init__(self, index, accuracy, controller):
			self.val = round(controller.get_axis(index),accuracy)
			self.index = index
			#Value changed?
			try:
				if self.val != prev_psc.ax[index].val:
					self.val_changed = 1
				else:
					self.val_changed = 0
			except:
				self.val_changed = 0

class hexapi_robot:
	def __init__(self):
		self.pwm = [0]*2
		self.pwm[0] = ServoKit(channels=16, address=0x40)
		self.pwm[1] = ServoKit(channels=16, address=0x41)
		self.pwm[0].frequency = 50
		self.pwm[1].frequency = 50
		with open('new_servo_config.csv',newline='') as csvfile:
			#Column Summary
			#0:leg (1 to 6)
			#1:part (1=shoulder; 2=arm; 3=claw)
			#2:hat
			#3:channel
			#4:lower limit pwm
			#5:upper limit pwm
			#6:offset
			#7:upper limit physical
			#8:upper limit physical
			arr = list(csv.reader(csvfile))
		self.motor = [[0 for x in range(4)] for y in range(7)]
		for r in range(1,18):
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
			
		def move(self, angle):
			if angle > self.LLphyang and angle < self.ULphyang:
				self.pwm[self.hat].servo[self.chan].angle = angle
			else:
				print("angle " + str(angle) + " out of bounds")


def Servo_Config_Module():
	print("Servo_Config_Module has started.")
	leg = 1
	part = 3

	while True:
		time.sleep(2)
		while not exit_Servo_Config:
			if psc.b_R.pds >= 0.5:
				if leg == 6:
					leg = 1
				else:
					leg = leg + 1
				print("Leg selected: " + str(leg))
				time.sleep(0.75)
			if psc.b_up.pds >= 0.5:
				if part == 3:
					part = 1
				else:
					part = part + 1
				print("part selected: " + str(part))
				time.sleep(0.75)
			
			test_angle = 90 + int(float(psc.j_Rx.val) * 88)
			if not len(str(test_angle))>2:
				msgx = "0" + str(test_angle)
			else:
				msgx = str(test_angle)
			msg = "\r" + "Test angle set to: " + msgx
			sys.stdout.write(msg)
			sys.stdout.flush()
			try:
				hx.motor[leg][part].move(test_angle)
			except:
				print("SOMETHING MESSED UP hx")
			time.sleep(0.02)
		if exit_Servo_Config:
			print("Servo_Config_Module loop inactive")
	print("Servo_Config_Module ended. This should never happen.")

#Desc.
def hexapi_main():
	
	#Controller Connection
	#global controller
	controller_found = False
	controller_count = 0
	attempts = 0
	while not controller_found:
		attempts = attempts + 1
		sys.stdout.write("\rScanning for Controllers... Attempt %s. " % (attempts))
		sys.stdout.flush()
		pygame.init()
		controller_count = pygame.joystick.get_count()
		if controller_count > 0:
			controller = pygame.joystick.Joystick(0)
			controller.init()
			print("Successfully connected to : " + controller.get_name())
			time.sleep(1)
			controller_found = True
		else:
			time.sleep(2)
			pygame.joystick.quit()
			
	#Define other threads
	Servo_Config_Module_thread = threading.Thread(target=Servo_Config_Module,args=())
	Servo_Config_Module_thread.setDaemon(True)		
	
	refresh_rate = 0.05
	global hx
	hx = hexapi_robot()
	
	global psc
	prev_psc = None
	psc = psc_update(controller, prev_psc, refresh_rate)
	
	global exit_Servo_Config
	exit_Servo_Config = True
	
	#Main loop
	exit_main = False
	while not exit_main:
		time.sleep(refresh_rate)
		prev_psc = psc
		psc = psc_update(controller, prev_psc, refresh_rate)

		#exit code inputs
		if (psc.b_start.pds >= 1) and (psc.b_select.pds >= 1):
			print("Start+Select buttons pressed for 1 seconds. Exiting controller_main...")
			exit_main = True
			
		if psc.b_tri.pds >= 2 and exit_Servo_Config == True:
			print("Servo_Config_Module set to start")
			exit_Servo_Config = False
			try:				
				Servo_Config_Module_thread.start()
				pass
			except:
				pass
			time.sleep(2)
		elif psc.b_tri.pds >= 2 and exit_Servo_Config == False:
			print("Servo_Config_Module set to exit")
			exit_Servo_Config = True
			time.sleep(2)
	print("End of hexapi_main code.")

#Start program
hexapi_main()











