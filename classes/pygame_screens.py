class pygame_screens:
	def __init__(self):
		#Pygame setup
		#self.HEIGHT = 1080
		#self.WIDTH = 1920
		self.HEIGHT = int(1080)
		self.WIDTH = int(1920/1.5)
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
	
	def DrawFoot(self,leg,pos,footcolor,font_size,reff):
		posxy = self.cord([pos[0][0],pos[1][0]])
		pygame.draw.rect(self.screen,footcolor,[posxy[0],posxy[1],8,8])
		self.txt2d("F"+str(leg)+"_"+reff,footcolor,posxy,font_size)
	
	def DrawFootVector(self,leg,datum,vec,vectorcolor,font_size):
		xyvec = self.cord([vec[0][0],vec[1][0]])
		xydatum = self.cord([datum[0][0],datum[1][0]])
		pygame.draw.line(self.screen,vectorcolor,xydatum,xyvec,2)
		self.txt2d("V"+str(leg),vectorcolor,xyvec,font_size)
	
	def DrawLimitSphere(self,Radius,centerxyz,limcolor):
		centerxy = self.cord([centerxyz[0][0],centerxyz[1][0]])
		pygame.draw.circle(self.screen,limcolor,centerxy,self.lifescale*Radius,1)