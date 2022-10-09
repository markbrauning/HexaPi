# Playstation controller Code


# For controller pairing:
# In the terminal place the following code:

# setup
#sudo apt-get install bluetooth libbluetooth3 libusb-dev
#sudo systemctl enable bluetooth.service
#sudo usermod -G bluetooth -a pi

# pairing
#wget http://www.pabr.org/sixlinux/sixpair.c
#gcc -o sixpair sixpair.c -lusb
	# plug in controller with cord
#sudo ./sixpair
	# should see the following message:
	# Current Bluetooth master: 5c:f3:70:66:5c:e2
	# Setting master bd_addr to 5c:f3:70:66:5c:e2



# confirm it is connected with:
#ls /dev/input	# device should be called something like 'js0'


# Handeler///

from triangula.input import SixAxis, SixAxisResource

# Button handler, will be bound to the square button later
def handler(button):
  print 'Button {} pressed'.format(button)

# Get a joystick, this will fail unless the SixAxis controller is paired and active
# The bind_defaults argument specifies that we should bind actions to the SELECT and START buttons to
# centre the controller and reset the calibration respectively.
with SixAxisResource(bind_defaults=True) as joystick:
    # Register a button handler for the square button
    joystick.register_button_handler(handler, SixAxis.BUTTON_SQUARE)
    while 1:
        # Read the x and y axes of the left hand stick, the right hand stick has axes 2 and 3
        x = joystick.axes[0].corrected_value()
        y = joystick.axes[1].corrected_value()
        print(x,y)
