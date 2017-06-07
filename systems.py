import pygame,math,sys,Vector2,sat,gen_collision

class PolySystem(object):
	
	def __init__(self, owner, vertices, mass, cross_constraints, stiffness , c_stiffness):
		
		# cross_constraints is a list containing tuples relating to the indices we are linking
		# e.g [(0,3),(1,4)]
		
		self.owner = owner
		self.mass = mass
		self.stiffness = stiffness				# the stiffness of constraints giving the body its shape
		self.c_stiffness = c_stiffness			# the stiffness of the cross constraints inner to the body
		
		# NOTE: low stiffness coefficients can lead to the shape collapsing into an invalid state
		
		if mass > 100000:
			self.immovable = True
		else:
			self.immovable = False
		
		self.timestep = 1
		self.num_iterations = 1
		self.inv_masses = [0.5] * len(vertices)
		self.vertices = vertices
		self.cross_constraints = cross_constraints
		
		self.minx,self.maxx = (0,320)
		self.miny,self.maxy = (0,240)
		
		self.constraint_lengths = []
		self.cross_lengths = []
		self.currentParticlePos = []
		self.oldParticlePos = []
		
		if not self.immovable:
			self.__calcConstraints()
		
		for v in vertices:
			vec = Vector2.Vector(v.x, v.y)
			veco = Vector2.Vector(v.x, v.y)
			self.currentParticlePos.append(vec)
			self.oldParticlePos.append(veco)
			
		self.accParticleForces = []
		
		for i in range(len(self.vertices)):
			v = Vector2.Vector()
			self.accParticleForces.append(v)
		
		self.NUM_PARTICLES = len(self.currentParticlePos)
		self.norm_friction = 0.0
		self.friction = 0.3
		self.drag = 0.0
		self.offending_index = None
		self.num_collisions = 0
		self.in_contact = False
		self.friction_ratios = [self.norm_friction] * self.NUM_PARTICLES	# 1.0 means all point get friction
		
		self.gravity = Vector2.Vector(0.0,0.11)
		self.wind = Vector2.Vector(0.0,0.0)
		
	def __calcConstraints(self):
		
		j = 1
		for i in range(len(self.vertices)):
			
			if i == len(self.vertices)-1:
				j = 0
				
			v1 = self.vertices[i]
			v2 = self.vertices[j]
			
			cvec = v2 - v1
			clen = cvec.get_magnitude()
			self.constraint_lengths.append(clen)
			
			j += 1
		
		if self.cross_constraints is not None:
			for i in range(len(self.cross_constraints)):
			
				indices = self.cross_constraints[i]
			
				v1 = self.vertices[indices[0]]
				v2 = self.vertices[indices[1]]
			
				cvec = v2 - v1
				clen = cvec.get_magnitude()
				self.cross_lengths.append(clen)
			
	def Verlet(self):
		
		if not self.immovable:
			for i in range(self.NUM_PARTICLES):		
				if self.inv_masses[i] > 0:	
					x = Vector2.Vector()
					x.x = self.currentParticlePos[i].x
					x.y = self.currentParticlePos[i].y
					
					temp = Vector2.Vector()
					temp.x = x.x
					temp.y = x.y
					
					oldx = Vector2.Vector()
					oldx.x = self.oldParticlePos[i].x
					oldx.y = self.oldParticlePos[i].y
					
					
					a = self.accParticleForces[i]
					vel = x - oldx
					
					if self.in_contact:
						
						f = 1 - (self.friction * ( self.friction_ratios[i])) 
						vel *= f
						
					x += vel + a * self.timestep * self.timestep
					oldx = temp
					
					# update positions
					self.currentParticlePos[i].x	= x.x
					self.currentParticlePos[i].y	= x.y
					self.oldParticlePos[i].x		= oldx.x
					self.oldParticlePos[i].y		= oldx.y
					
	def AccumulateForces(self):
		
		if not self.immovable:
			for i in range(self.NUM_PARTICLES):
				if self.inv_masses[i] > 0:
					self.accParticleForces[i] = self.gravity + self.wind
	
	def CollideBounds(self):
		
		if not self.immovable:
			for i in range(self.NUM_PARTICLES):
				point = self.currentParticlePos[i]
				
				if point.x < self.minx:
					point.x = self.minx
					self.friction_ratios[i] = 0.3
					self.num_collisions += 1
				
				elif point.x > self.maxx:
					point.x = self.maxx
					self.friction_ratios[i] = 0.3
					self.num_collisions += 1
					
				if point.y < self.miny:
					point.y = self.miny
					self.friction_ratios[i] = 0.3
					self.num_collisions += 1
					
				elif point.y > self.maxy:
					point.y = self.maxy
					self.friction_ratios[i] = 0.3
					self.num_collisions += 1
		
	def SatisfyConstraints(self):
		
		if not self.immovable:
		
			for i in range(self.num_iterations):	
				
				j = 1
				for i in range(len(self.currentParticlePos)):
					
					if i == len(self.currentParticlePos)-1:
						j = 0
					
					restlength = self.constraint_lengths[i]
					restlength2 = restlength*restlength
				
					delta = self.currentParticlePos[j]-self.currentParticlePos[i]
					delta2 = delta.dotProduct(delta)
					
					x1 = Vector2.Vector()
					x2 = Vector2.Vector()
						
					x1.x = self.currentParticlePos[i].x
					x1.y = self.currentParticlePos[i].y
					x2.x = self.currentParticlePos[j].x
					x2.y = self.currentParticlePos[j].y
				
					diff = restlength2 /(delta2 + restlength2) - 0.5
					
					inv_mass1 = self.inv_masses[i]
					inv_mass2 = self.inv_masses[j]
					
					delta_invmass = inv_mass1 + inv_mass2
					
					delta *= diff/ delta_invmass		# 1.0 here is the combined particle masses
				
					x1 -= delta * inv_mass1 * self.stiffness 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
					x2 += delta * inv_mass2 * self.stiffness	#* diff
				
					self.currentParticlePos[i].x = x1.x
					self.currentParticlePos[i].y = x1.y
					self.currentParticlePos[j].x = x2.x
					self.currentParticlePos[j].y = x2.y
					
					j += 1
					
				######################################
				# cross_constraints relaxation
				######################################
				if self.cross_constraints is not None:
				
					for i in range(len(self.cross_constraints)):
						
						i1,i2 = self.cross_constraints[i]
						restlength = self.cross_lengths[i] 	
						restlength2 = restlength*restlength
					
						delta = self.currentParticlePos[i2]-self.currentParticlePos[i1]
						delta2 = delta.dotProduct(delta)
						
						x1 = Vector2.Vector()
						x2 = Vector2.Vector()
						x1.x = self.currentParticlePos[i1].x
						x1.y = self.currentParticlePos[i1].y
						
						x2.x = self.currentParticlePos[i2].x
						x2.y = self.currentParticlePos[i2].y
					
						
						diff = restlength2 /(delta2 + restlength2) - 0.5
						
						inv_mass1 = self.inv_masses[i1]
						inv_mass2 = self.inv_masses[i2]
						
						delta_invmass = inv_mass1 + inv_mass2
						
						delta *= diff / delta_invmass 		# 1.0 here is the combined particle masses
					
						x1 -= delta * inv_mass1 * self.c_stiffness 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
						x2 += delta * inv_mass2 * self.c_stiffness	#* diff
					
						self.currentParticlePos[i1].x = x1.x
						self.currentParticlePos[i1].y = x1.y
						
						self.currentParticlePos[i2].x = x2.x
						self.currentParticlePos[i2].y = x2.y
			
	def draw(self,Surface):
		
		j = 1
		for i in range(self.NUM_PARTICLES):
			if i == self.NUM_PARTICLES - 1:
				j = 0
				
			pygame.draw.line(Surface,(250,0,0),(int(self.currentParticlePos[i].x),int(self.currentParticlePos[i].y)),
							(int(self.currentParticlePos[j].x),int(self.currentParticlePos[j].y)))
			j += 1
			
			#pygame.draw.circle(Surface,(240,0,0),(self.currentParticlePos[i].x,self.currentParticlePos[i].y),5)
			#if i < self.NUM_PARTICLES - 1:
			#	pygame.draw.line(screen,WHITE,(int(self.currentParticlePos[i].x),int(self.currentParticlePos[i].y)),(int(self.currentParticlePos[i+1].x),int(self.currentParticlePos[i+1].y)))
		
		#pygame.draw.rect(Surface,(0,0,255),self.owner.rect,1)
		if self.cross_constraints is not None:
			for i,j in self.cross_constraints:
				linepos1 = self.currentParticlePos[i].to_point()
				linepos2 = self.currentParticlePos[j].to_point()
			
				pygame.draw.line(Surface,(0,255,0),linepos1,linepos2)
		
class CircleSystem(object):
	
	def __init__(self, owner, pos , radius, mass):
		
		# cross_constraints is a list containing tuples relating to the indices we are linking
		# e.g [(0,3),(1,4)]
		
		self.owner = owner
		self.mass = mass
		self.radius = radius
		
		if mass > 100000:
			self.immovable = True
		else:
			self.immovable = False
		
		
		self.inv_mass = 1
		
		self.timestep = 1.0
		self.minx,self.maxx = (0,320)
		self.miny,self.maxy = (0,240)
		
		self.currentParticlePos = []
		self.oldParticlePos = []
		
		
		vec = Vector2.Vector(pos.x, pos.y)
		veco = Vector2.Vector(pos.x, pos.y)
		self.currentParticlePos.append(vec)
		self.oldParticlePos.append(veco)
		
		self.accParticleForces = []
		v = Vector2.Vector()
		self.accParticleForces.append(v)
		
		self.NUM_PARTICLES = len(self.currentParticlePos)
		self.norm_friction = 0.0
		self.friction = 0.2
		self.drag = 0.0
		self.offending_index = None
		self.num_collisions = 0
		self.in_contact = False
		self.friction_ratios = [self.norm_friction] * self.NUM_PARTICLES	# 1.0 means all point get friction
		
		self.gravity = Vector2.Vector(0.0,0.08)
		self.wind = Vector2.Vector(0.0,0.0)
		
	def __calcConstraints(self):
		pass
			
	def Verlet(self):
		
		if not self.immovable:
		
			for i in range(self.NUM_PARTICLES):
				x = self.currentParticlePos[i]
				temp = x
				oldx = self.oldParticlePos[i]
				a = self.accParticleForces[i]
				vel = x - oldx
				
				if self.in_contact:	
					f = 1 - (self.friction * ( self.friction_ratios[i])) 
					vel *= f
					
				x += vel + a * self.timestep * self.timestep 
				oldx = temp
				
				# update positions
				self.currentParticlePos[i]	= x
				self.oldParticlePos[i]		= oldx
				
	def AccumulateForces(self):
		
		if not self.immovable:
			for i in range(self.NUM_PARTICLES):
				self.accParticleForces[i] = self.gravity + self.wind
	
	def CollideBounds(self):
		
		if not self.immovable:
			for i in range(self.NUM_PARTICLES):
				point = self.currentParticlePos[i]
				
				if point.x < self.minx + self.radius:
					point.x = self.minx + self.radius
					self.friction_ratios[i] = 1.0
					self.num_collisions += 1
				
				elif point.x > self.maxx - self.radius:
					point.x = self.maxx - self.radius
					self.friction_ratios[i] = 1.0
					self.num_collisions += 1
					
				if point.y < self.miny + self.radius:
					point.y = self.miny + self.radius
					self.friction_ratios[i] = 1.0
					self.num_collisions += 1
					
				elif point.y > self.maxy - self.radius:
					point.y = self.maxy - self.radius
					self.friction_ratios[i] = 1.0
					self.num_collisions += 1
		
	def SatisfyConstraints(self):
		pass
			
	def draw(self,Surface):
		
		pygame.draw.circle(Surface,(240,0,0),(self.currentParticlePos[0].x,self.currentParticlePos[0].y),self.radius,1)
		
		#pygame.draw.rect(Surface,(0,0,255),self.owner.rect,1)

class FreeSystem(object):
	
	def __init__(self, owner, particle_list, constraint_list):
		
		
		self.owner = owner
		
		self.timestep = 1
		self.num_iterations = 1
		self.inv_masses = []
		
		self.minx,self.maxx = (0,320)
		self.miny,self.maxy = (0,240)
		
		self.constraint_list = constraint_list
		
		self.currentParticlePos = []
		self.oldParticlePos = []
		
		
		for point,mass in particle_list:
			vec = Vector2.Vector(point[0],point[1])	# create new particle
			veco = Vector2.Vector(point[0],point[1])
			self.currentParticlePos.append(vec)
			self.oldParticlePos.append(veco)
			
			# initialise inv_masses
			self.inv_masses.append(mass)
		
		self.accParticleForces = []
		
		for i in range(len(self.currentParticlePos)):
			v = Vector2.Vector()
			self.accParticleForces.append(v)
			
		self.NUM_PARTICLES = len(self.currentParticlePos)
		self.norm_friction = 0.0
		self.friction = 0.3
		self.drag = 0.0
		self.offending_index = None
		self.num_collisions = 0
		self.in_contact = False
		self.friction_ratios = [self.norm_friction] * self.NUM_PARTICLES	# 1.0 means all point get friction
		
		self.gravity = Vector2.Vector(0.0,0.11)
		self.wind = Vector2.Vector(0.0,0.0)
	
	def Verlet(self):
		
		
		for i in range(self.NUM_PARTICLES):
			
			if self.inv_masses[i] > 0:
			
				x = Vector2.Vector()
				x.x = self.currentParticlePos[i].x
				x.y = self.currentParticlePos[i].y
				
				temp = Vector2.Vector()
				temp.x = x.x
				temp.y = x.y
				
				oldx = Vector2.Vector()
				oldx.x = self.oldParticlePos[i].x
				oldx.y = self.oldParticlePos[i].y
				
				
				a = self.accParticleForces[i]
				vel = x - oldx
				
				if self.in_contact:
					
					f = 1 - (self.friction * ( self.friction_ratios[i])) 
					vel *= f
					
				x += vel + a * self.timestep * self.timestep
				oldx = temp
				
				# update positions
				self.currentParticlePos[i].x	= x.x
				self.currentParticlePos[i].y	= x.y
				self.oldParticlePos[i].x		= oldx.x
				self.oldParticlePos[i].y		= oldx.y
		
	def AccumulateForces(self):
		
		for i in range(self.NUM_PARTICLES):
			if self.inv_masses[i] > 0:
				self.accParticleForces[i] = self.gravity + self.wind
	
	def SatisfyConstraints(self):
		
		for i in range(self.num_iterations):	
			
			for i in range(len(self.constraint_list)):
				
				length,indices,stiffness = self.constraint_list[i]
				start,end = indices
				
				restlength = length
				restlength2 = restlength*restlength
			
				delta = self.currentParticlePos[end]-self.currentParticlePos[start]
				delta2 = delta.dotProduct(delta)
				
				x1 = Vector2.Vector()
				x2 = Vector2.Vector()
					
				x1.x = self.currentParticlePos[start].x
				x1.y = self.currentParticlePos[start].y
				x2.x = self.currentParticlePos[end].x
				x2.y = self.currentParticlePos[end].y
			
				diff = restlength2 /(delta2 + restlength2) - 0.5
				
				inv_mass1 = self.inv_masses[start]
				inv_mass2 = self.inv_masses[end]
				
				delta_invmass = inv_mass1 + inv_mass2
				
				delta *= diff/ delta_invmass		# 1.0 here is the combined particle masses
			
				x1 -= delta * inv_mass1 * stiffness 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
				x2 += delta * inv_mass2 * stiffness		#* diff
			
				self.currentParticlePos[start].x = x1.x
				self.currentParticlePos[start].y = x1.y
				self.currentParticlePos[end].x = x2.x
				self.currentParticlePos[end].y = x2.y
	
	def CollideBounds(self):
		
		self.num_collisions = 0
		for i in range(self.NUM_PARTICLES):
			point = self.currentParticlePos[i]
			
			if point.x < self.minx:
				point.x = self.minx
				self.friction_ratios[i] = 0.3
				self.num_collisions += 1
			
			elif point.x > self.maxx:
				point.x = self.maxx
				self.friction_ratios[i] = 0.3
				self.num_collisions += 1
				
			if point.y < self.miny:
				point.y = self.miny
				self.friction_ratios[i] = 0.3
				self.num_collisions += 1
				
			elif point.y > self.maxy:
				point.y = self.maxy
				self.friction_ratios[i] = 0.3
				self.num_collisions += 1
	
		if self.num_collisions > 0:
			self.in_contact = True
		else:
			self.in_contact = False
	
	def draw(self,Surface):
		
		for i in range(self.NUM_PARTICLES):
			#pygame.draw.circle(Surface,self.color,(self.current_particle_pos[i].x,self.current_particle_pos[i].y),2)
			pass
			
		# will see if this is necessary later
		for constraint in self.constraint_list:
			s_i = constraint[1][0]
			e_i = constraint[1][1]
				
				
			pos1x = self.currentParticlePos[s_i].x
			pos1y = self.currentParticlePos[s_i].y
				
				
			#print "this is thre pos",pos1x,pos1y
			pos2x = self.currentParticlePos[e_i].x
			pos2y = self.currentParticlePos[e_i].y
				
			pygame.draw.line(Surface,(255,0,0),(pos1x,pos1y),(pos2x,pos2y))
