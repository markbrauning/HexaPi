import pygame, time, math, threading, sys, os, csv
from pygame.locals import *
from pygame import event
from adafruit_servokit import ServoKit

pwm0 = ServoKit(channels=16, address=0x40)
pwm1 = ServoKit(channels=16, address=0x41)
global gait_selected
gait_selected = 0

global chan
global hat
chan = 1
hat = 1

global vector_thread_kill
vector_thread_kill = False

global foot_vectors
foot_vectors=[[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

global foot_coordinates
#                      leg1     2       3       4       5       6
#                    [x,y,z] [x,y,z] [x,y,z] [x,y,z] [x,y,z] [x,y,z]
foot_coordinates  = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

global calculated_angles
#                     leg1      2       3       4       5       6
#                    [beta,theta,alpha] ie [shoulder,arm,claw]
calculated_angles = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

#Desc.
def read_in_servo_config():
        print("Importing servo configs...")
        global servo_config
                       #[[(shoulder), (arm), (claw)],[]...]
        servo_config =  [[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[],[]]
        
        with open('servo_config.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                row_count = 0
                for row in csv_reader:
                    col_count = 0
                    array_col = 0
                    for col_entry in row:
                        servo_config[array_col].append(col_entry)
                        array_col += 1
                        col_count += 1  
                    row_count += 1
                print('Processed %s columns.' % (col_count))
                print('Processed %s rows.' % (row_count))
                print("servo config im port complete")

#Desc.
def read_in_gaits():
        print("Importing gaits...")
        global gait_array

        #                leg1    leg2    leg3    leg4    leg5    leg6
        #               tz,txy  tz,txy  tz,txy  tz,txy  tz,txy  tz,txy 
        gait_array =  [[[], [], [], [], [], [], [], [], [], [], [], []], #gait option 0
                       [[], [], [], [], [], [], [], [], [], [], [], []], #gait option 1
                       [[], [], [], [], [], [], [], [], [], [], [], []], #gait option 2
                       [[], [], [], [], [], [], [], [], [], [], [], []]] #gait option 3


        with open('gaits/gait_0.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                row_count = 0
                for row in csv_reader:
                    col_count = 0
                    gait_array_col = 0
                    for col_entry in row:
                        if (row_count != 0) and ((col_count%3 != 0)):
                            gait_array[0][gait_array_col].append(col_entry)
                            gait_array_col += 1
                        col_count += 1  
                    row_count += 1
                print('Gait 0. Processed %s rows.' % (row_count))

        with open('gaits/gait_1.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                row_count = 0
                for row in csv_reader:
                    col_count = 0
                    gait_array_col = 0
                    for col_entry in row:
                        if (row_count != 0) and ((col_count%3 != 0)):
                            gait_array[1][gait_array_col].append(col_entry)
                            gait_array_col += 1
                        col_count += 1  
                    row_count += 1
                print('Gait 1. Processed %s rows.' % (row_count))

        with open('gaits/gait_2.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                row_count = 0
                for row in csv_reader:
                    col_count = 0
                    gait_array_col = 0
                    for col_entry in row:
                        if (row_count != 0) and ((col_count%3 != 0)):
                            gait_array[2][gait_array_col].append(col_entry)
                            gait_array_col += 1
                        col_count += 1  
                    row_count += 1
                print('Gait 2. Processed %s rows.' % (row_count))

        with open('gaits/gait_3.csv') as csv_file:
                csv_reader = csv.reader(csv_file, delimiter=',')
                row_count = 0
                for row in csv_reader:
                    col_count = 0
                    gait_array_col = 0
                    for col_entry in row:
                        if (row_count != 0) and ((col_count%3 != 0)):
                            gait_array[3][gait_array_col].append(col_entry)
                            gait_array_col += 1
                        col_count += 1  
                    row_count += 1
                print('Gait 3. Processed %s rows.' % (row_count))

#Desc.
def foot_vector_main(vector_event):
	print("foot_vector_main thread has started.")
	prev_a_js_left_x = 0
	prev_a_js_left_y = 0
	prev_a_js_right_x = 0
	
	#start Gait thread
	gait_thread = threading.Thread(target=gait_main,args=())
	gait_thread.setDaemon(True)
	gait_thread.start()
	
	exit_vector = False
	while not exit_vector:
		if not vector_event.is_set():
			print("exit_vector set to True")
			exit_vector = True
		time.sleep(0.05)
		
		a_js_left_x = psc.j_lx
		a_js_left_y = psc.j_ly
		a_js_right_x = psc.j_rx
		if (a_js_left_x != prev_a_js_left_x) or (a_js_left_y != prev_a_js_left_y) or(a_js_right_x != prev_a_js_right_x):
			foot_vectors = calc_foot_vectors(a_js_left_x,a_js_left_y,a_js_right_x)
			prev_a_js_left_x = a_js_left_x
			prev_a_js_left_y = a_js_left_y 
			prev_a_js_right_x = a_js_right_x
			report_string = "\rFoot Vectors... x=" + str(round(a_js_left_x,1)) + " y=" + str(round(a_js_left_y,1)) + " r=" + str(round(a_js_right_x,1))
			sys.stdout.write(report_string)
			sys.stdout.flush()
	print("foot_vector_main has ended")

#Desc.
def calc_foot_vectors(Qx,Qy,Qr):
	#print("Calculating Foot Vecotrs... ")
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

#Take in calucluated angles for a motor and moves it using PWM
def move_motor(leg, part, servo_angle):
    use_col = 3*leg + part
    use_i2c = int(servo_config[use_col][183])
    use_channel = int(servo_config[use_col][184])
    use_angle = int(servo_angle)

    if use_i2c == 0:
        pwm0.servo[use_channel].angle = use_angle
    else:
        pwm1.servo[use_channel].angle = use_angle
                
#Takes in the current angles and moves each leg with the "move_motor" function
def move_legs(servo_angles):
    for leg in range(6):
        for part in range(3):
            move_motor(leg, part, servo_angles[leg][part])
            
#Takes in the desired XYZ components for each leg and outputs the end angles
def calc_angles(foot_coordinates):
    l1 = 7.75
    l2 = 14.5
    l3 = 5.75
    
    #                leg1      2       3       4       5       6
    #           [beta,theta,alpha] ie shoulder,arm,claw
    servo_angles = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]
																																																						

    for leg in range(6):
        x = foot_coordinates[leg][0]
        y = foot_coordinates[leg][1]
        z = foot_coordinates[leg][2]
        xy = ((x**2)+(y**2))**(0.5)
        c = (((xy-l3)**2)+(z**2))**(0.5)

        #Calculate alpha
        #fix acos domain error
        alpha_acos_piece = ((l1**2)+(l2**2)-(c**2))/(2*l1*l2)

        if (alpha_acos_piece<=-1):
                alpha_acos_piece = -1
        if (alpha_acos_piece>=1):
                alpha_acos_piece = 1

        alpha = math.acos(alpha_acos_piece) - (math.pi/2)
                
        #Calculate theta
        #fix acos domain error
        theta_acos_piece = ((l1**2)+(c**2)-(l2**2))/(2*l1*c)
        if (theta_acos_piece<=-1):
                theta_acos_piece = -1
        if (theta_acos_piece>=1):
                theta_acos_piece = 1
        
        theta_acos_piece1 = z/c
        if (theta_acos_piece1<=-1):
                theta_acos_piece1 = -1
        if (theta_acos_piece1>=1):
                theta_acos_piece1 = 1
        
        if(c == 0):
                c = 0.00000000001

        if(xy<=l3):
                theta_piece1 = - math.acos(theta_acos_piece1)
        else:
                theta_piece1 = math.acos(theta_acos_piece1)
        
        theta = theta_piece1 - math.acos(theta_acos_piece)
        
        #Calculate beta
        beta_acos_piece = x/xy
        if (beta_acos_piece<=-1):
                beta_acos_piece = -1
        if (beta_acos_piece>=1):
                beta_acos_piece = 1
        if(x == 0):
                x = 0.00000000001
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
                theta = 00

        servo_angles[leg] = [beta*(180/math.pi),theta*(180/math.pi),alpha*(180/math.pi)]
        
    return servo_angles

#Desc.
def calc_foot_xyz(i):
    x_datum = 0
    y_datum = 5
    z_datum = -19
    scale = 2
    #                      leg1     2       3       4       5       6
    #                    [x,y,z] [x,y,z] [x,y,z] [x,y,z] [x,y,z] [x,y,z]
    foot_coordinates  = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

    for leg in range(6):
            #access z column of gait csv and multiply it by the magnitude of the foot vector for that leg
            foot_coordinates[leg][2] = float(gait_array[gait_selected][leg*2][i])*foot_vectors[leg][2]*scale + z_datum
            
            #access the xy column of the gait csv and multiply it by the x component of the foot vector
            foot_coordinates[leg][0] = float(gait_array[gait_selected][leg*2+1][i])*foot_vectors[leg][0]*scale + x_datum
            
            #access the xy column of the gait csv and multiply it by the y component of the foot vector
            foot_coordinates[leg][1] = float(gait_array[gait_selected][leg*2+1][i])*foot_vectors[leg][1]*scale + y_datum
    return foot_coordinates

#Desc.
def gait_main():
	print("gait_main thread has started.")
	while True:
		for i in range(len(gait_array[gait_selected][0])): 
			time.sleep(0.01)                               
			foot_coordinates = calc_foot_xyz(i)
			calculated_angles = calc_angles(foot_coordinates)
			move_legs(calculated_angles)
	print("gait_main has ended")

#Desc.
def controller_main(controller):
	print("controller_main thread has started.")
	global psc
	while True:
		refresh_rate = 0.01
		time.sleep(refresh_rate)
		events = pygame.event.get()
		psc = controller_inputs(controller, refresh_rate)
	print("End of controller_main code.")

#Desc.
class controller_inputs():
	def __init__(self,controller, refresh_rate):
		#Axes current values:
		accuracy = 3
		self.j_Lx = round(controller.get_axis(0),accuracy)
		self.j_Ly = round(controller.get_axis(1),accuracy)
		self.j_L2 = round(controller.get_axis(2),accuracy)
		self.j_Rx = round(controller.get_axis(3),accuracy)
		self.j_Ry = round(controller.get_axis(4),accuracy)
		self.j_R2 = round(controller.get_axis(5),accuracy)
		
		#Buttons current values:
		self.b_cross = controller.get_button(0)		#0 Cross
		self.b_circle = controller.get_button(1)	#1 Circle
		self.b_tri = controller.get_button(2)		#2 Triangle
		self.b_sq= controller.get_button(3)		#3 Square
		self.L1 = controller.getbutton(4)		#4 L1
		self.R1 = controller.getbutton(5)		#5 R1
		self.L2 = controller.getbutton(6)		#6 L2
		self.R2 = controller.getbutton(7)		#7 R2
		self.b_select = controller.get_button(8)	#8 Select
		self.b_start = controller.get_button(9)		#9 Start
		self.b_ps = controller.get_button(10)		#10 Logo
		self.Lj = controller.getbutton(11)		#11 Left joystick
		self.Rj = controller.getbutton(12)		#12 Right joystick
		self.b_up = controller.get_button(13)		#13 d-pad up
		self.b_down = controller.get_button(14)		#14 d-pad down
		self.b_left = controller.get_button(15)		#15 d-pad left
		self.b_right = controller.get_button(16)	#16 d-pad right
		
#Desc.
def oneleg_main(oneleg_event):
	print("oneleg_main has started.")
	exit_oneleg = False
	while not exit_oneleg:
		time.sleep(0.005)
		if not oneleg_event.is_set():
			exit_oneleg = True
		pwm0.servo[chan].angle = 90 + int(float(psc.j_ry) * 90)
	print("oneleg_main ended.")

#Desc.
def hexapi_main():
	#Get Gait and servo configs
	read_in_servo_config()
	read_in_gaits()
	
	#Check if there are any controllers connected
	controller_found = False
	controller_count = 0
	attempts = 0
	while not controller_found:
		attempts = attempts + 1
		sys.stdout.write("\rScanning for Controllers... Attempt %s. " % (attempts))
		sys.stdout.flush()
		pygame.init()
		pygame.joystick.init()
		controller_count = pygame.joystick.get_count()
		if controller_count > 0:
			print("Controller count is: " + str(controller_count))
			global controller
			controller = pygame.joystick.Joystick(0)
			controller.init()
			print("Controller name: " + controller.get_name())
			print("Controller connection successful.")
			controller_found = True
		else:
			time.sleep(2)
			pygame.joystick.quit()
	
	#start controller input monitoring thread
	contr_thread = threading.Thread(target=controller_main,args=(controller, ))
	contr_thread.setDaemon(True)
	contr_thread.start()
	time.sleep(2)
	
	#Define other threads
	vector_event = threading.Event()
	vector_thread = threading.Thread(target=foot_vector_main,args=(vector_event, ))
	vector_thread.setDaemon(True)
	

	oneleg_event = threading.Event()
	oneleg_thread = threading.Thread(target=oneleg_main,args=(oneleg_event, ))
	oneleg_thread.setDaemon(True)
	
	#Main
	refresh_rate = 0.01
	gait_selected = 0
	exit_main = False
	b_start_hold = 0
	b_circle_hold = 0
	b_dl_hold = 0
	b_dr_hold = 0
	b_du_hold = 0
	b_dd_hold = 0
	chan = 4
	hat = 1

	while not exit_main:
		time.sleep(refresh_rate)
		#exit code inputs
		if (psc.b_start == 1) and (psc.b_select == 1):
			print("Start+Select buttons pressed. Exiting controller_main...")
			exit_main = True
		
		#Start/End Vector & Gait threads
		if psc.b_start == 1:
			b_start_hold = b_start_hold + 1
			if b_start_hold == 300:
				print("start pressed for 3 seconds")
				if vector_thread.is_alive():
					print("main command to end vector_thread")
					vector_event.clear()
					time.sleep(0.5)
				else:
					#Start foot vector thread
					vector_event.set()
					time.sleep(0.5)
					vector_thread.start()
		else:
			b_start_hold = 0

				
		#Start/End oneleg thread
		if psc.b_circle == 1:
			b_circle_hold = b_circle_hold + 1
			if b_circle_hold == 100:
				print("Circle pressed for 1 seconds")
				if oneleg_thread.is_alive():
					print("main command to end oneleg_thread")
					oneleg_event.clear()
					time.sleep(1)
				else:
					#Start foot vector thread
					oneleg_event.set()
					time.sleep(0.5)
					oneleg_thread.start()
		else:
			b_circle_hold = 0

		#leg select down
		if psc.b_dl == 1:
			b_dl_hold = b_dl_hold + 1
			if b_dl_hold == 10:
				if chan == 1:
					chan = 16
				else:
					chan = chan - 1
				print("Channel selected: " + str(chan))
		else:
			b_dl_hold = 0
		
		#leg select up
		if psc.b_dr == 1:
			b_dr_hold = b_dr_hold + 1
			if b_dr_hold == 10:
				if chan == 16:
					chan = 1
				else:
					chan = chan + 1
				print("Channel selected: " + str(chan))
		else:
			b_dr_hold = 0
		
		#joint select up
		if psc.b_du == 1:
			b_du_hold = b_du_hold + 1
			if b_du_hold == 10:
				if hat == 2:
					hat = 1
				else:
					hat = hat + 1
				print("Joint selected: " + str(hat))
		else:
			b_du_hold = 0
		
		

	print("End of hexapi_main code.")



#Start program
hexapi_main()
