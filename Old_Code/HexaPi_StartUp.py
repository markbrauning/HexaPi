from V4_Code import *

import pygame, time, threading
from pygame.locals import *
from pygame import event

def Controller_Connect():
	#Check if there are any controllers connected
	pygame.init()
	controller_count = 0
	attempts = 0
	while pygame.joystick.get_count() = 0:
		attempts = attempts + 1
		sys.stdout.write("\rScanning for Controllers... Attempt %s. " % (attempts))
		sys.stdout.flush()
		controller_count = pygame.joystick.get_count()
		if controller_count > 0:
			print("Controller count is: " + str(controller_count))
			controller = pygame.joystick.Joystick(0)
			controller.init()
			print("Controller name: " + controller.get_name())
			print("Controller connection successful.")
		else:
			time.sleep(1)




def HexaPi_Main():
	time.sleep(20)
	contr_thread = threading.Thread(target=Controller_Connect,args=())
	contr_thread.setDaemon(True)
	while True:
		time.sleep(10)
		if not contr_thread.is_alive():	
			contr_thread.start()

#Start Code
HexaPi_Main()
