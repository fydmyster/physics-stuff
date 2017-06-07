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
	def __init__(self,x,y,w,h):
		self.currentParticlePos=[]
		self.oldParticlePos=[]
		self.accParticleForces=[]
		
		self.colliding_points = []
		
		# init particles
		self.p1 = Vector2.Vector(x,y)
		self.p2 = Vector2.Vector(x + w, y)
		self.p3 = Vector2.Vector(x + w, y + h)
		self.p4 = Vector2.Vector(x, y + h)
		
		# init forces
		self.accforce1=Vector2.Vector()
		self.accforce2=Vector2.Vector()
		self.accforce3=Vector2.Vector()
		self.accforce4=Vector2.Vector()
		
		# append new particles
		self.currentParticlePos.append(self.p1)
		self.currentParticlePos.append(self.p2)
		self.currentParticlePos.append(self.p3)
		self.currentParticlePos.append(self.p4)
		
		# append old particles
		self.oldParticlePos.append(self.p1)
		self.oldParticlePos.append(self.p2)
		self.oldParticlePos.append(self.p3)
		self.oldParticlePos.append(self.p4)
		
		# append forces
		self.accParticleForces.append(self.accforce1)
		self.accParticleForces.append(self.accforce2)
		self.accParticleForces.append(self.accforce3)
		self.accParticleForces.append(self.accforce4)
		
		self.constraint_len = 30
		self.cross_len = math.sqrt((self.constraint_len * self.constraint_len) 
									+ (self.constraint_len * self.constraint_len))
		
		
		self.updateColliders()
		self.NUM_PARTICLES = len(self.currentParticlePos)
	
		self.norm_friction = 0.0
		self.friction = 0.6
		self.offending_index = None
		self.in_contact = False
		self.friction_ratios = [self.norm_friction] * self.NUM_PARTICLES	# 1.0 means all point get friction
		
		self.gravity = Vector2.Vector(0.0,0.13)
		self.timeStep = 0.0
		
	def updateSegColliders(self):
		pass
		
	def updateColliders(self):
		
		t_collider = sat.Line((self.currentParticlePos[0].x, self.currentParticlePos[0].y),
								(self.currentParticlePos[1].x, self.currentParticlePos[1].y))
								
		r_collider = sat.Line((self.currentParticlePos[1].x, self.currentParticlePos[1].y),
								(self.currentParticlePos[2].x, self.currentParticlePos[2].y))
								
		b_collider = sat.Line((self.currentParticlePos[2].x, self.currentParticlePos[2].y),
								(self.currentParticlePos[3].x, self.currentParticlePos[3].y))
								
		l_collider = sat.Line((self.currentParticlePos[3].x, self.currentParticlePos[3].y),
								(self.currentParticlePos[0].x, self.currentParticlePos[0].y))
		
		
		#av_positions = []
		# average line ypositions and use the to sort line_colliders
		#lines_dict = {}
		
		# store line seg info along with the indices to currentParticlePos they reference
		#self.line_colliders = [(t_collider,(0,1)), (r_collider,(1,2)), (b_collider,(2,3)), (l_collider,(3,0))]
		self.line_colliders = [(t_collider,(0,1)), (r_collider,(1,2)), (b_collider,(2,3)), (l_collider,(3,0))]
		
		
		'''
		epsilon = 0.1
		for line,index in self.line_colliders:
			line_av = (line.pos.y + line.p1.y) / 2.0
			line_av += epsilon
			
			lines_dict[line_av] = (line,index)
			epsilon += 0.1
			
		d_keys = lines_dict.keys()
		d_keys.sort()
		
		self.line_colliders = []
		for key in d_keys:
			line = lines_dict[key]
			self.line_colliders.append(line)
		'''
		
		points = []
		for v in self.currentParticlePos:
			point = v.to_point()
			points.append(point)
			
		self.shape_collider = sat.Polygon(points)
		
		# calculate box center
		# get vector from point 0 to point 2
		crossVector = self.currentParticlePos[2] - self.currentParticlePos[0]
		crossVector /=2
		self.center = self.currentParticlePos[0] + crossVector
			
		
	def Verlet(self):
		for i in range(self.NUM_PARTICLES):
			x = self.currentParticlePos[i]
			temp = x
			oldx = self.oldParticlePos[i]
			a = self.accParticleForces[i]
			vel = x - oldx
			
			if self.in_contact:
				vel *= 1.0 - (self.friction * ( self.friction_ratios[i]))
				#print self.friction_ratios[i]
				
			x += vel + a 
			oldx = temp
			
			# update lists
			self.currentParticlePos[i]	= x
			self.oldParticlePos[i]		= oldx
			
	def AccumulateForces(self):
		for i in range(self.NUM_PARTICLES):
			self.accParticleForces[i] = self.gravity
			
	def SatisfyConstraints(self):
		
		# collide with geometry
		result = sat.collidePolygon(self.shape_collider,polyA)
		
		if result[0]:
			self.in_contact = True
			#print "collision guy"
			#sat.resolveCollision(result,self.shape_collider,polyA,1)
			#self.collider.update()
			
			
			overlap = (abs(result[1])) * 0.95				#is the penetration amount
			dir = (result[2])                           #is the penetration direction
			d = self.shape_collider.center - polyA.center
	
			if d.dotProduct(dir) < 0:
				dir = -dir
		
			dir *= overlap
			
			for line_collider,index in self.line_colliders:
				line_collider.x += dir.x
				line_collider.y += dir.y
				line_collider.update()
			
			scp = None
			dcp = None
			
			offending_lines_list = []
			ret = False
			self.offending_index = None
			offending_collider = None
			for line_collider,index in self.line_colliders:
				
				if ret:
					break
				
				for line in polyA_lines:
			
					stick = gen_collision.Line((float(line_collider.pos.x),float(line_collider.pos.y)),
												(float(line_collider.p1.x),float(line_collider.p1.y)))
					line_seg = gen_collision.Line(line[0],line[1])
				
					#dcp = gen_collision.calculateIntersectPoint((self.collider.pos.x,self.collider.pos.y),(self.collider.p1.x,self.collider.p1.y),
					#										line[0],line[1])
						
					res = stick.collideLine(line_seg)
					if res[0]:
						#print "this is happening"
						dcp = res[1]
					
						
					scp = dcp
				
					if dcp is not None:
						rx = dcp[0] - dir.x
						ry = dcp[1] - dir.y
						dcp = [rx,ry]
						self.offending_index = index
						offending_collider = line_collider
						#offending_lines_list.append([line_collider,scp,dcp,index])
						
						ret = True
						break
			'''			
			if len(offending_lines_list) > 0:
				gradient = 100
				flattest_line = offending_lines_list[0][0] 
				flattest_scp = offending_lines_list[0][1]
				flattest_dcp = offending_lines_list[0][2]
				flattest_index = offending_lines_list[0][3]
				
				for line,static_point,dyn_point,o_index in offending_lines_list: 
					
					try:
						rise = line.p1.y - line.pos.y 
						run = line.p1.x - line.pos.x
						t_gradient = rise/float(run)
						
						if t_gradient < gradient:
							gradient = t_gradient
							flattest_line = line
							flattest_scp = static_point
							flattest_dcp = dyn_point
							flattest_index = o_index
							
					except ZeroDivisionError:
						gradient = 0
						flattest_line = line
						flattest_scp = static_point
						flattest_dcp = dyn_point
						flattest_index = o_index
							
						
				offending_collider = flattest_line
				scp = flattest_scp
				dcp = flattest_dcp
				self.offending_index = flattest_index
			'''
			
			# do line seg check for contact point
			#scp = None
			#for line in polyA_lines:
			#	scp = gen_collision.calculateIntersectPoint((self.collider.pos.x,self.collider.pos.y),(self.collider.p1.x,self.collider.p1.y),
			#											line[0],line[1])
			#	if scp is not None:
			#		break
			
			
			for line_collider,index in self.line_colliders:
				line_collider.x -= dir.x
				line_collider.y -= dir.y
				line_collider.update()
			
			#offending_collider.x -= dir.x
			#offending_collider.y -= dir.y
			
			
			if scp is not None:
				
				#if scp[0] == "parallel":
				#	print "line is parallel"
				
				pygame.draw.circle(screen,(0,255,0),(int(scp[0]),int(scp[1])),5)
				pygame.draw.circle(screen,(255,0,0),(int(dcp[0]),int(dcp[1])),5)
				
				d_vec = offending_collider.p1 - offending_collider.pos 
				d_mag = d_vec.get_magnitude()
				
				p_vec = Vector2.Vector(scp[0],scp[1])
				p_vec = p_vec - offending_collider.pos
				p_mag = p_vec.get_magnitude()
				
				ratio = p_mag/float(d_mag)
				#print ratio
				
				c2 = ratio				# I swapped these around
				c1 = 1 - ratio			# I swapped these around
				scp_v = Vector2.Vector(scp[0],scp[1])
				dcp_v = Vector2.Vector(dcp[0],dcp[1])
				
				#self.friction_ratios = [c1,c2]
				self.friction_ratios = [self.norm_friction] * self.NUM_PARTICLES
				self.friction_ratios[self.offending_index[0]] = c1
				self.friction_ratios[self.offending_index[1]] = c2
				
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
				
				x1 = offending_collider.pointlist[0] + (dis_vec_n *(c1 * mys))		# used a normal here too 
				x2 = offending_collider.pointlist[1] + (dis_vec_n *(c2 * mys)) 
				
				
				self.currentParticlePos[self.offending_index[0]] = x1
				self.currentParticlePos[self.offending_index[1]] = x2
				
			
			
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
		
		
		for i in range(3):
			#cube constrain
			for point in self.currentParticlePos:
				if point.x<0:
					point.x=0
				elif point.x>WIDTH:
					point.x=WIDTH
				
				if point.y<0:
					point.y=0
				elif point.y>HEIGHT:
					point.y=HEIGHT
			
			
			restlength = self.constraint_len
			restlength2 = restlength*restlength
			
			crosslength = self.cross_len
			crosslength2 = crosslength*crosslength
			
			
			delta = self.currentParticlePos[1]-self.currentParticlePos[0]
			delta2 = delta.dotProduct(delta)
			
			x1=self.currentParticlePos[0]
			x2=self.currentParticlePos[1]
			
			#deltalength = math.sqrt(delta.dotProduct(delta))
			#diff = (60-deltalength)/deltalength 	# 60 is the constraint length (had to hack and swap deltalength and 60 around)
			
			diff = restlength2 /(delta2 + restlength2) - 0.5
			#diff *= -2.0
			
			delta *= diff/1.0
			
			x1 -= delta * (0.5) * 1.0 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
			x2 += delta * (0.5) * 1.0	#* diff
			
			#print x1
			self.currentParticlePos[0] = x1
			self.currentParticlePos[1] = x2
			
			#############################################################################
			delta = self.currentParticlePos[2]-self.currentParticlePos[1]
			delta2 = delta.dotProduct(delta)
			
			x1=self.currentParticlePos[1]
			x2=self.currentParticlePos[2]
			
			#deltalength = math.sqrt(delta.dotProduct(delta))
			#diff = (60-deltalength)/deltalength 	# 60 is the constraint length (had to hack and swap deltalength and 60 around)
			
			diff = restlength2 /(delta2 + restlength2) - 0.5
			#diff *= -2.0
			
			delta *= diff/1.0
			
			x1 -= delta * (0.5) * 1.0 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
			x2 += delta * (0.5) * 1.0	#* diff
			
			#print x1
			self.currentParticlePos[1] = x1
			self.currentParticlePos[2] = x2
			
			#############################################################################
			delta = self.currentParticlePos[3]-self.currentParticlePos[2]
			delta2 = delta.dotProduct(delta)
			
			x1=self.currentParticlePos[2]
			x2=self.currentParticlePos[3]
			
			#deltalength = math.sqrt(delta.dotProduct(delta))
			#diff = (60-deltalength)/deltalength 	# 60 is the constraint length (had to hack and swap deltalength and 60 around)
			
			diff = restlength2 /(delta2 + restlength2) - 0.5
			#diff *= -2.0
			
			delta *= diff/1.0
			
			x1 -= delta * (0.5) * 1.0 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
			x2 += delta * (0.5) * 1.0	#* diff
			
			#print x1
			self.currentParticlePos[2] = x1
			self.currentParticlePos[3] = x2
			
			#############################################################################
			delta = self.currentParticlePos[0] - self.currentParticlePos[3]
			delta2 = delta.dotProduct(delta)
			
			x1=self.currentParticlePos[3]
			x2=self.currentParticlePos[0]
			
			#deltalength = math.sqrt(delta.dotProduct(delta))
			#diff = (60-deltalength)/deltalength 	# 60 is the constraint length (had to hack and swap deltalength and 60 around)
			
			diff = restlength2 /(delta2 + restlength2) - 0.5
			#diff *= -2.0
			
			delta *= diff/1.0
			
			x1 -= delta * (0.5) * 1.0 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
			x2 += delta * (0.5) * 1.0	#* diff
			
			#print x1
			self.currentParticlePos[3] = x1
			self.currentParticlePos[0] = x2
			
			#############################################################################
			#	cross constraint to prevent collapse
			#############################################################################
			delta = self.currentParticlePos[0] - self.currentParticlePos[2]
			delta2 = delta.dotProduct(delta)
			
			x1=self.currentParticlePos[2]
			x2=self.currentParticlePos[0]
			
			#deltalength = math.sqrt(delta.dotProduct(delta))
			#diff = (60-deltalength)/deltalength 	# 60 is the constraint length (had to hack and swap deltalength and 60 around)
			
			diff = crosslength2 /(delta2 + crosslength2) - 0.5
			#diff *= -2.0
			
			delta *= diff/1.0
			
			x1 -= delta * (0.5) * 1.0 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
			x2 += delta * (0.5) * 1.0	#* diff
			
			#print x1
			self.currentParticlePos[2] = x1
			self.currentParticlePos[0] = x2
			
	def Update(self,timepassed):
		self.timeStep=timepassed
	
	def TimeStep(self):
		self.AccumulateForces()
		self.Verlet()
		
		self.updateColliders()
		
		#for i in range(5):
		self.SatisfyConstraints()
		
		#print self.in_contact
		
	def draw(self):
		for i in range(self.NUM_PARTICLES):
			pygame.draw.line(screen,BLUE,(int(self.line_colliders[i][0].pos.x),int(self.line_colliders[i][0].pos.y)),
							(int(self.line_colliders[i][0].p1.x),int(self.line_colliders[i][0].p1.y)))
			#pygame.draw.circle(screen,WHITE,(self.currentParticlePos[i].x,self.currentParticlePos[i].y),5)
			if i < self.NUM_PARTICLES - 1:
				pygame.draw.line(screen,(0,255,40),(int(self.currentParticlePos[i].x),int(self.currentParticlePos[i].y)),(int(self.currentParticlePos[i+1].x),int(self.currentParticlePos[i+1].y)))
			
	
clock=pygame.time.Clock()
Timepassed=0

pygame.display.set_caption("Verlet Integration")
screen=pygame.display.set_mode((WIDTH,HEIGHT))

# collidables
#polyA = sat.RectSAT([190,100,(250,0,0)],30,160)
polyA = sat.DEG45RD([150,150,(250,0,0)],30,60)


t_line = [(float(polyA.position.x),float(polyA.position.y)),(float(polyA.p1.x),float(polyA.p1.y))]
r_line = [(float(polyA.p1.x),float(polyA.p1.y)),(float(polyA.p2.x),float(polyA.p2.y))]
#b_line = [(float(polyA.p2.x),float(polyA.p2.y)),(float(polyA.p3.x),float(polyA.p3.y))]
b_line = [(polyA.p2.x,polyA.p2.y),(polyA.position.x,polyA.position.y)]
#l_line = [(polyA.p3.x,polyA.p3.y),(polyA.position.x,polyA.position.y)]
#l_line = [(float(polyA.p3.x),float(polyA.p3.y)),(float(polyA.position.x),float(polyA.position.y))]

print t_line
#polyA_lines = [t_line,r_line,b_line,l_line]
polyA_lines = [t_line,r_line,b_line]



# create test system
stick=ParticleSystem(49,29,50,50)

while True:
	for event in pygame.event.get():
		if event.type==QUIT:
			pygame.quit()
			sys.exit()
			
		if event.type==KEYDOWN:
			if event.key==K_ESCAPE:
				pygame.quit()
				sys.exit()
			'''
			if event.key == K_LEFT:
				stick.currentParticlePos[0].x -= 3
			if event.key == K_RIGHT:
				stick.currentParticlePos[0].x += 3
			if event.key == K_UP:
				stick.currentParticlePos[0].y -= 3
			if event.key == K_DOWN:
				stick.currentParticlePos[0].y += 3
			'''	
		if event.type==KEYUP:
			pass
	
	states = pygame.key.get_pressed()
	if states[K_LEFT]:
		stick.currentParticlePos[0].x -= 1
	if states[K_RIGHT]:
		stick.currentParticlePos[0].x += 1
	if states[K_UP]:
		stick.currentParticlePos[0].y -= 1
	if states[K_DOWN]:
		stick.currentParticlePos[0].y += 1
	
	
	screen.fill(BLACK)
	
	stick.TimeStep()
		
	stick.draw()
	polyA.draw(screen)
	pygame.display.update()
		
	Timepassed=clock.tick(60)