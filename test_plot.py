import pygame
from pygame.locals import *
import sys
import time
import math as m
toc = 0

pygame.init()


purple = (100,0,255)
blue=(0,0,255)
red=(255,0,0)
white=(255,255,255)

HEIGHT = 750
WIDTH = 750
FPS = 60
 
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("HexaPi")
screen.fill(white)

font_style = pygame.font.SysFont(None, 20)
def txt(msg,color,row,col):
	pos = cord([-(WIDTH/2)+col*100,(HEIGHT/2)-row*15])
	mesg = font_style.render(msg, True, color)
	screen.blit(mesg, pos)

def cord(pos):
	x=int((WIDTH/2)+pos[0])
	y=int(-(HEIGHT/2)+HEIGHT-pos[1])
	return [x,y]
def cordx(pos):
	x=int((WIDTH/2)+pos)
	return x
def cordy(pos):
	y=int(-(HEIGHT/2)+HEIGHT-pos)
	return y


x = 0
y = 0

FPS = pygame.time.Clock()

while True:
	time.sleep(0.1)
	
	tic = time.perf_counter()
	screen.fill(white)
	x = x+1
	y = y+1
	
	pygame.draw.rect(screen,blue,[cordx(x),cordy(y),5,5])
	pygame.draw.line(screen,red,cord([0,0]),cord([100,0]),2)
	pygame.draw.line(screen,red,cord([0,0]),cord([0,100]),2)
	txt("x: "+str(x),purple,0,0)
	txt("y: "+str(y),purple,1,0)
	toc = time.perf_counter()
	txt(f"Main loop took : {toc - tic:0.6f} seconds to complete.",purple,48,0)
	pygame.display.update()

	
	
