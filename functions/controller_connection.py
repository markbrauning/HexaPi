import pygame, time, threading, sys, os, csv, pandas
from pygame.locals import *
from pygame import event
from adafruit_servokit import ServoKit
import numpy as np
import math as m



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