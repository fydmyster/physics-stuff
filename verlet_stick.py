import pygame,math,sys,Vector2,sat,gen_collision
from pygame.locals import*

pygame.init()

WIDTH=320
HEIGHT=240

BLACK=(0,0,0)
WHITE=(255,255,255)
GREEN=(0,255,0)
BLUE=(0,0,255)

# particle class that implements verlet Integration

def vmin(vec):
	pass

def vmax(vec):
	pass

class ParticleSystem:
	def __init__(self):
		self.currentParticlePos=[]
		self.oldParticlePos=[]
		self.accParticleForces=[]
		
		self.colliding_points = []
		
		# init particles
		self.part1=Vector2.Vector(10.0,10.0)
		self.part2=Vector2.Vector(45.0,15.0)
		# init forces
		self.accforce1=Vector2.Vector()
		self.accforce2=Vector2.Vector()
		# append new particles
		self.currentParticlePos.append(self.part1)
		self.currentParticlePos.append(self.part2)
		# append old particles
		self.oldParticlePos.append(self.part1)
		self.oldParticlePos.append(self.part2)
		# append forces
		self.accParticleForces.append(self.accforce1)
		self.accParticleForces.append(self.accforce2)
		
		self.collider = sat.Line((self.part1.x,self.part1.y),(self.part2.x,self.part2.y))
		self.cur_center = Vector2.Vector(self.collider.center.x,self.collider.center.y)
		self.old_center = Vector2.Vector(self.collider.center.x,self.collider.center.y)
		
		self.in_contact = False
		self.friction_ratios = [0,0]
		
		self.gravity = Vector2.Vector(0.0,0.13)
		self.timeStep = 0.0
		self.NUM_PARTICLES=len(self.currentParticlePos)
		
		
	def Verlet(self):
		for i in range(self.NUM_PARTICLES):
			x=self.currentParticlePos[i]
			temp=x
			oldx=self.oldParticlePos[i]
			a=self.accParticleForces[i]
			vel = x-oldx
			
			if self.in_contact:
				vel *= 1 - (0.10 * ( self.friction_ratios[i]))
			
			x += vel + a 
			oldx=temp
			
			# update lists
			self.currentParticlePos[i]=x
			self.oldParticlePos[i]=oldx
			
	def AccumulateForces(self):
		for i in range(self.NUM_PARTICLES):
			self.accParticleForces[i]=self.gravity
			
	def SatisfyConstraints(self):
		
		# collide with geometry
		result = sat.collidePolygon(self.collider,polyA)
		
		if result[0]:
			self.in_contact = True
			#print "collision guy"
			#sat.resolveCollision(result,self.collider,polyA,1)
			#self.collider.update()
			
			
			overlap = (abs(result[1])) *0.98 # removed the coefficient since 1 works fine	#is the penetration amount
			dir = (result[2])                           #is the penetration direction
			d = self.collider.center - polyA.center
	
			if d.dotProduct(dir) < 0:
				dir = -dir
		
			dir *= overlap
			
			self.collider.x += dir.x
			self.collider.y += dir.y
			self.collider.update()
			
			scp = None
			dcp = None
			for line in polyA_lines:
			
				stick = gen_collision.Line((self.collider.pos.x,self.collider.pos.y),(self.collider.p1.x,self.collider.p1.y))
				line_seg = gen_collision.Line(line[0],line[1])
				
				#dcp = gen_collision.calculateIntersectPoint((self.collider.pos.x,self.collider.pos.y),(self.collider.p1.x,self.collider.p1.y),
				#										line[0],line[1])
				
				res = stick.collideLine(line_seg)
				if res[0]:
					dcp = res[1]
				
				scp = dcp
				
				if dcp is not None:
					rx = dcp[0] - dir.x
					ry = dcp[1] - dir.y
					dcp = [rx,ry]
					break
			
			
			# do line seg check for contact point
			#scp = None
			#for line in polyA_lines:
			#	scp = gen_collision.calculateIntersectPoint((self.collider.pos.x,self.collider.pos.y),(self.collider.p1.x,self.collider.p1.y),
			#											line[0],line[1])
			#	if scp is not None:
			#		break
			
			
			self.collider.x -= dir.x
			self.collider.y -= dir.y	
			self.collider.update()
			
			if scp is not None:
				
				if scp[0] == "parallel":
					print "line is parallel"
				
				pygame.draw.circle(screen,(0,255,0),scp,5)
				pygame.draw.circle(screen,(255,0,0),dcp,5)
				
				d_vec = self.collider.p1 - self.collider.pos 
				d_mag = d_vec.get_magnitude()
				
				p_vec = Vector2.Vector(scp[0],scp[1])
				p_vec = p_vec - self.collider.pos
				p_mag = p_vec.get_magnitude()
				
				ratio = p_mag/float(d_mag)
				#print ratio
				
				c2 = ratio				# I swapped these around
				c1 = 1 - ratio			# I swapped these around
				scp_v = Vector2.Vector(scp[0],scp[1])
				dcp_v = Vector2.Vector(dcp[0],dcp[1])
				
				self.friction_ratios = [c1,c2]
				
				'''
				dis_vec = scp_v - dcp_v
				dis_vec_n = scp_v - dcp_v
				dis_vec_n2 = scp_v - dcp_v
				dis_vec_n.normalize()
				dis_vec_n2.normalize()
				
				
				numerator = dis_vec.dotProduct(dis_vec_n) 
				denom =  dis_vec_n.dotProduct(dis_vec_n) * (c1 * c1 + c2 * c2)		# used a normal here 
				mys = numerator/float(denom)
				
				x1 = self.collider.pointlist[0] + (dis_vec_n *(c1*mys))		# used a normal here too 
				x2 = self.collider.pointlist[1] + (dis_vec_n *(c2*mys)) 
				'''
				
				dis_vec = scp_v - dcp_v
				dis_vec_n = scp_v - dcp_v
				dis_vec_n2 = scp_v - dcp_v
				dis_vec_n.normalize()
				dis_vec_n2.normalize()
				
				
				numerator = dis_vec.dotProduct(dis_vec_n) 
				denom =  dis_vec_n.dotProduct(dis_vec_n) * (c1 * c1 + c2 * c2)		# used a normal here 
				mys = numerator/float(denom)
				
				x1 = self.collider.pointlist[0] + (dis_vec_n *(c1 * mys))		# used a normal here too 
				x2 = self.collider.pointlist[1] + (dis_vec_n *(c2 * mys)) 
				
				
				self.currentParticlePos[0] = x1
				self.currentParticlePos[1] = x2
				
			
			
			# adjust particle positions
			#self.currentParticlePos[0].x = self.collider.pointlist[0].x
			#self.currentParticlePos[0].y = self.collider.pointlist[0].y
			#self.currentParticlePos[1].x = self.collider.pointlist[1].x
			#self.currentParticlePos[1].y = self.collider.pointlist[1].y
			
				
			
			#self.oldParticlePos[0].x = self.collider.pointlist[0].x
			#self.oldParticlePos[0].y = self.collider.pointlist[0].y
			#self.oldParticlePos[1].x = self.collider.pointlist[1].x
			#self.oldParticlePos[1].y = self.collider.pointlist[1].y
		
		else:
			self.in_contact = False
		
		#cube constrain
		
		for point in self.currentParticlePos:
			if point.x < 0:
				point.x = 0
			elif point.x > WIDTH:
				point.x = WIDTH
			
			if point.y < 0:
				point.y = 0
			elif point.y > HEIGHT:
				point.y = HEIGHT
		
		
		restlength = 60
		restlength2 = restlength*restlength
		
		delta = self.currentParticlePos[1]-self.currentParticlePos[0]
		delta2 = delta.dotProduct(delta)
		
		x1=self.currentParticlePos[0]
		x2=self.currentParticlePos[1]
		
		#deltalength = math.sqrt(delta.dotProduct(delta))
		#diff = (60-deltalength)/deltalength 	# 60 is the constraint length (had to hack and swap deltalength and 60 around)
		
		diff = restlength2 /(delta2 + restlength2) - 0.5
		#diff *= -2.0
		
		delta *= diff/1.0
		
		x1 -= delta * (0.5) * 0.4 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
		x2 += delta * (0.5) * 0.4	#* diff
		
		#print x1
		self.currentParticlePos[0] = x1
		self.currentParticlePos[1] = x2
		
	def Update(self,timepassed):
		self.timeStep=timepassed
	
	def TimeStep(self):
		self.AccumulateForces()
		self.Verlet()
		
		self.old_center = self.cur_center
		
		# redefine collider
		point1 = self.currentParticlePos[0].x, self.currentParticlePos[0].y
		point2 = self.currentParticlePos[1].x, self.currentParticlePos[1].y
		
		self.collider = sat.Line(point1,point2)
		
		self.cur_center = Vector2.Vector(self.collider.center.x,self.collider.center.y)
		
		
		self.SatisfyConstraints()
		
		print self.in_contact
		
	def draw(self):
		for i in range(self.NUM_PARTICLES-1):
			pygame.draw.line(screen,WHITE,(int(self.currentParticlePos[i].x),int(self.currentParticlePos[i].y)),(int(self.currentParticlePos[i+1].x),int(self.currentParticlePos[i+1].y)))
			#pygame.draw.circle(screen,WHITE,(self.currentParticlePos[i].x,self.currentParticlePos[i].y),5)
			
	
clock=pygame.time.Clock()
Timepassed=0

pygame.display.set_caption("Verlet Integration")
screen=pygame.display.set_mode((WIDTH,HEIGHT))

# collidables
polyA = sat.RectSAT([190,40,(250,0,0)],30,160)
#polyA = sat.DEG45RU([190,190,(250,0,0)],30,40)

t_line = [(polyA.position.x,polyA.position.y),(polyA.p1.x,polyA.p1.y)]
r_line = [(polyA.p1.x,polyA.p1.y),(polyA.p2.x,polyA.p2.y)]
b_line = [(polyA.p2.x,polyA.p2.y),(polyA.p3.x,polyA.p3.y)]
#b_line = [(polyA.p2.x,polyA.p2.y),(polyA.position.x,polyA.position.y)]
#l_line = [(polyA.p3.x,polyA.p3.y),(polyA.position.x,polyA.position.y)]
l_line = [(polyA.p3.x,polyA.p3.y),(polyA.position.x,polyA.position.y)]

polyA_lines = [t_line,r_line,b_line,l_line]



# create test system
stick=ParticleSystem()

while True:
	for event in pygame.event.get():
		if event.type==QUIT:
			pygame.quit()
			sys.exit()
			
		if event.type==KEYDOWN:
			if event.key==K_ESCAPE:
				pygame.quit()
				sys.exit()
			
			if event.key == K_LEFT:
				stick.currentParticlePos[0].x -= 3
			if event.key == K_RIGHT:
				stick.currentParticlePos[0].x += 3
			if event.key == K_UP:
				stick.currentParticlePos[0].y -= 3
			if event.key == K_DOWN:
				stick.currentParticlePos[0].y += 3
				
		if event.type==KEYUP:
			pass
			
		
	screen.fill(BLACK)
	
	stick.TimeStep()
		
	stick.draw()
	polyA.draw(screen)
	pygame.display.update()
		
	Timepassed=clock.tick(60)