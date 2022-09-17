# HexaPi


Need to set up PS3 controller:

https://www.hackster.io/sgmoorthy/ps3-joystick-controlled-raspberry-pi-robot-454769

Or just run these while connected via USB:

sudo apt-get -y install libusb-dev joystick python-pygame
cd ~
wget http://www.pabr.org/sixlinux/sixpair.c
gcc -o sixpair sixpair.c -lusb

reboot then run this:
sudo ~/sixpair

I kept getting usb.h missing error so i ran the following:
sudo apt-get install libusb-dev

Test get 9-17-22
