
global foot_coordinates
#                      leg1     2       3       4       5       6
#                    [x,y,z] [x,y,z] [x,y,z] [x,y,z] [x,y,z] [x,y,z]
foot_coordinates  = [[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

            
#Takes in the desired XYZ components for each leg and outputs the end angles


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

#Start program
hexapi_main()
