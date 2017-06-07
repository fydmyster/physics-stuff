import pygame,math,sys,Vector2,sat,gen_collision
from pygame.locals import*

# todo : implement restitution
#		: sleeping objects if velocity drops beneath a certain threshold
#		:  grid based spatial partitions
#		:  calculate masses properly

pygame.init()

WIDTH=320
HEIGHT=240

BLACK=(0,0,0)
WHITE=(255,255,255)
GREEN=(0,255,0)
BLUE=(0,0,255)

# particle class that implements verlet Integration

class ShapeProperties(object):
	
	def __init__(self,shape_collider = None,line_colliders = None):
		
		self.parent = None
		self.shape_collider = shape_collider
		self.line_colliders = line_colliders
	
	def setParent(self,parent):
		self.parent = parent
	
	def setShapeCollider(self, shape_collider):
		self.shape_collider = shape_collider
	
	def setLineColliders(self, line_colliders):
		self.line_colliders = line_colliders
	
def vmin(vec):
	pass

def vmax(vec):
	pass

class ParticleSystem:
	def __init__(self,x,y,w,h):
		
		self.type = "dynamic"
		self.mass = 1
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
		
		self.constraint_len = 20
		self.cross_len = math.sqrt((self.constraint_len * self.constraint_len) 
									+ (self.constraint_len * self.constraint_len))
		
		self.properties = ShapeProperties()
		self.properties.setParent(self)
		self.updateColliders()
		self.NUM_PARTICLES = len(self.currentParticlePos)
	
		self.norm_friction = 0.0
		self.friction = 0.3
		self.drag = 0.0
		self.offending_index = None
		self.num_collisions = 0
		self.in_contact = False
		self.friction_ratios = [self.norm_friction] * self.NUM_PARTICLES	# 1.0 means all point get friction
		
		self.gravity = Vector2.Vector(0.0,0.05)
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
		line_colliders = [(t_collider,(0,1)), (r_collider,(1,2)), (b_collider,(2,3)), (l_collider,(3,0))]
		
		self.properties.setLineColliders(line_colliders)
		
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
			
		shape_collider = sat.Polygon(points)
		shape_collider.type = "dynamic"
		
		self.properties.setShapeCollider(shape_collider)
		
		
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
			
			# movement hack to prevent objects from shifting slighty infinitely
			# comment in out if you feel you dont need it
			# its a faster hack than implementing sleeping
			
			#if abs(vel.x) < 0.05 and abs(vel.y) < 0.05:
			#	vel.x = 0.0
			#	vel.y = 0.0
			
			
			# trying to model true friction does not got down well with particle based stuff
			# so to get stability I unfortunately have to apply friction to all particles 
			# to prevent none colliding ones from shifting. This therefore sacrifices some torque
			# but the results are passable (as long as friction aint too strong)
			# I'll look into sorting this out in the future
			if self.in_contact:
				
				f = 1 - (self.friction * ( self.friction_ratios[i])) 
					#print f
				vel *= f
				#print self.friction_ratios[i]
			
			# optional drag
			#else:
			#	f = 1 - (self.drag * ( self.friction_ratios[i])) 
				#print f
			#	vel *= f
			
				
			x += vel + a 
			oldx = temp
			
			# update lists
			self.currentParticlePos[i]	= x
			self.oldParticlePos[i]		= oldx
			
	def AccumulateForces(self):
		for i in range(self.NUM_PARTICLES):
			self.accParticleForces[i] = self.gravity
	
	def CollideBodies(self):
		
		bodies = []
		for a_properties in world_objects:
			
			if a_properties.shape_collider.type == "dynamic":
				bodies.append(a_properties)
		
		'''
		bodies_dict = {}
		smallest_dist = 1000
		#line_to_test = None
		
		epsilon = 0.035
		for body in bodies:
			shape = body.shape_collider
			c_vec = (shape.center - self.properties.shape_collider.center)
			dist = c_vec.get_magnitude()
			
			
			if bodies_dict.has_key(dist):
				dist += epsilon
			
			bodies_dict[dist] = body
			
		d_keys = bodies_dict.keys()
		d_keys.sort()
		
		bodies = []
		for key in d_keys:
			body = bodies_dict[key]
			bodies.append(body)
		'''
		
		for body in bodies:
			polyA = body.shape_collider
			polyA_lines = body.line_colliders
			parent = body.parent
			
			if self.properties.shape_collider == polyA:
				continue
		
			result = sat.collidePolygon(self.properties.shape_collider,polyA)
			
					
			if result[0]:
				
				self.num_collisions += 1
				
				m = 0.5
				
				overlap = (abs(result[1])) * 0.999 	# hacky var here: 0.8 seems more stable than 0.95
				
				dir = (result[2])                           #is the penetration direction
				dir_other = Vector2.Vector(result[2].x,result[2].y)
				
				d = self.properties.shape_collider.center - polyA.center
				d_other = polyA.center - self.properties.shape_collider
				
				if d.dotProduct(dir) < 0:
					dir = -dir
			
				if d_other.dotProduct(dir_other) < 0:
					dir_other = -dir_other
			
				
				dir *= overlap
				dir_other *= overlap
				#dir *= m
				#dir_other *= m
				
				for line_collider,index in self.properties.line_colliders:
					line_collider.x += dir.x
					line_collider.y += dir.y
					line_collider.update()
				
				scp = None
				dcp = None
				scp_other = None
				dcp_other = None
				
				######################################################################
				# sort the lines
				######################################################################
				
				lines_dict = {}
				smallest_dist = 1000
				#line_to_test = None
				
				#epsilon = 0.01
				for line,index in self.properties.line_colliders:
					c_vec = (line.p1 - line.pos)
					mag = c_vec.get_magnitude()
					c_vec.normalize()
					true_centroid = line.pos + (c_vec * (mag/2.0))
					
					# dist from poly center
					d_vec = true_centroid - polyA.center
					dist = d_vec.get_magnitude()
					
					#if dist < smallest_dist:
					#	smallest_dist = dist
						#line_to_test = (line,index)
					
					lines_dict[dist] = (line,index)
					#epsilon += 0.01
				
				d_keys = lines_dict.keys()
				d_keys.sort()
			
				self.properties.line_colliders = []
				for key in d_keys:
					line = lines_dict[key]
					self.properties.line_colliders.append(line)
			
			
				#################
				# todo : test ALL colliding line segs and store their indices to compute friction
				
				offending_lines_list = []
				ret = False
				self.offending_index = None
				offending_collider = None
				offending_collider_other = None
				offending_index_other = None
				
				for line_collider,index in self.properties.line_colliders:
					
					if ret:
						break
					
					for line,ind in polyA_lines:
				
						stick = gen_collision.Line(((line_collider.pos.x),(line_collider.pos.y)),
													((line_collider.p1.x),(line_collider.p1.y)))
						line_seg = gen_collision.Line(((line.pos.x),(line.pos.y)),
													((line.p1.x),(line.p1.y)))
					
						#dcp = gen_collision.calculateIntersectPoint((self.collider.pos.x,self.collider.pos.y),(self.collider.p1.x,self.collider.p1.y),
						#										line[0],line[1])
							
						res = stick.collideLine(line_seg)
						if res[0]:
							#print "this is happening"
							dcp = res[1]
						
							
						scp = dcp
					
						if dcp is not None:
							rx = dcp[0] - (dir.x )
							ry = dcp[1] - (dir.y )
							dcp = [rx,ry]
							self.offending_index = index
							offending_collider = line_collider
							
							ret = True
							break
				
				
				for line_collider,index in self.properties.line_colliders:
					line_collider.x -= dir.x
					line_collider.y -= dir.y
					line_collider.update()
				
				#offending_collider.x -= dir.x
				#offending_collider.y -= dir.y
				
				
				
				for line_collider,index in parent.properties.line_colliders:
					line_collider.x += dir_other.x
					line_collider.y += dir_other.y
					line_collider.update()
				
			
				scp_other = None
				dcp_other = None
				
				######################################################################
				# sort the lines
				######################################################################
				
				lines_dict = {}
				smallest_dist = 1000
				#line_to_test = None
				
				#epsilon = 0.01
				for line,index in parent.properties.line_colliders:
					c_vec = (line.p1 - line.pos)
					mag = c_vec.get_magnitude()
					c_vec.normalize()
					true_centroid = line.pos + (c_vec * (mag/2.0))
					
					# dist from poly center
					d_vec = true_centroid - self.properties.shape_collider.center
					dist = d_vec.get_magnitude()
					
					lines_dict[dist] = (line,index)
						
				d_keys = lines_dict.keys()
				d_keys.sort()
			
				parent.properties.line_colliders = []
					
				for key in d_keys:
					line = lines_dict[key]
					parent.properties.line_colliders.append(line)
			
				
				#################
				# todo : test ALL colliding line segs and store their indices to compute friction
				
				ret = False
				offending_collider_other = None
				offending_index_other = None
				
				for line_collider,index in parent.properties.line_colliders:
						
					if ret:
						break
						
					for line,ind in self.properties.line_colliders:
					
						stick = gen_collision.Line(((line_collider.pos.x),(line_collider.pos.y)),
													((line_collider.p1.x),(line_collider.p1.y)))
						line_seg = gen_collision.Line(((line.pos.x),(line.pos.y)),
													((line.p1.x),(line.p1.y)))
						
							#dcp = gen_collision.calculateIntersectPoint((self.collider.pos.x,self.collider.pos.y),(self.collider.p1.x,self.collider.p1.y),
							#										line[0],line[1])
								
						res = stick.collideLine(line_seg)
						if res[0]:
							#print "this is happening"
							dcp_other = res[1]
							
								
						scp_other = dcp_other
						
						if dcp_other is not None:
							rx = dcp_other[0] - (dir_other.x )
							ry = dcp_other[1] - (dir_other.y )
							dcp_other = [rx,ry]
							offending_index_other = index
							offending_collider_other = line_collider
								
							ret = True
							break
					
				
				for line_collider,index in parent.properties.line_colliders:
					line_collider.x -= dir_other.x
					line_collider.y -= dir_other.y
					line_collider.update()
				
				
				
				if scp is not None:
					
					#if scp[0] == "parallel":
					#	print "line is parallel"
					
					#pygame.draw.circle(screen,(0,255,0),(int(scp[0]),int(scp[1])),5)
					#pygame.draw.circle(screen,(255,0,0),(int(dcp[0]),int(dcp[1])),5)
					
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
					dis_vec_n.normalize()
					
					
					numerator = dis_vec.dotProduct(dis_vec_n) 
					denom =  dis_vec_n.dotProduct(dis_vec_n) * (c1 * c1 + c2 * c2)		# used a normal here 
					mys = numerator/float(denom)
					
					x1 = offending_collider.pointlist[0] + ((dis_vec_n *(c1 * mys * m)))		# used a normal here too 
					x2 = offending_collider.pointlist[1] + ((dis_vec_n *(c2 * mys * m))) 
					
					
					self.currentParticlePos[self.offending_index[0]] = x1
					self.currentParticlePos[self.offending_index[1]] = x2
					
					#print "why why"
				
				
				if  scp_other is not None :
					

					### todo
					# attempt to move the other lineseg involved in collision in the opposite dir
					# of the other one and share the movements by half
					
					d_vec = offending_collider_other.p1 - offending_collider_other.pos 
					d_mag = d_vec.get_magnitude()
					
					p_vec = Vector2.Vector(scp_other[0],scp_other[1])
					p_vec = p_vec - offending_collider_other.pos
					p_mag = p_vec.get_magnitude()
					
					ratio = p_mag/float(d_mag)
					#print ratio
					
					c2 = ratio				# I swapped these around
					c1 = 1 - ratio			# I swapped these around
					scp_v = Vector2.Vector(scp_other[0],scp_other[1])
					dcp_v = Vector2.Vector(dcp_other[0],dcp_other[1])
					
					#self.friction_ratios = [c1,c2]
					#parent.friction_ratios = [self.norm_friction] * self.NUM_PARTICLES
					parent.friction_ratios[offending_index_other[0]] = c1
					parent.friction_ratios[offending_index_other[1]] = c2
					
					
					dis_vec = scp_v - dcp_v
					dis_vec_n = scp_v - dcp_v
					dis_vec_n.normalize()
					
					
					numerator = dis_vec.dotProduct(dis_vec_n) 
					denom =  dis_vec_n.dotProduct(dis_vec_n) * (c1 * c1 + c2 * c2)		# used a normal here 
					mys = numerator/float(denom)
					
					x1 = offending_collider_other.pointlist[0] + ((dis_vec_n *(c1 * mys * m)))		# used a normal here too 
					x2 = offending_collider_other.pointlist[1] + ((dis_vec_n *(c2 * mys * m))) 
					
					
					parent.currentParticlePos[offending_index_other[0]] = x1
					parent.currentParticlePos[offending_index_other[1]] = x2
						
	
	def CollideGeometry(self):
		
		polygons = []
		for a_properties in world_objects:
			
			if a_properties.shape_collider.type == "static":
				polygons.append(a_properties)
		
		for poly in polygons:
			polyA = poly.shape_collider
			polyA_lines = poly.line_colliders
			parent = poly.parent
			
			if self.properties.shape_collider == polyA:
				break
		
			result = sat.collidePolygon(self.properties.shape_collider,polyA)
			
					
			if result[0]:
				
				self.num_collisions += 1
				
				m = 1
				
				overlap = (abs(result[1])) * 0.99 	# hacky var here: 0.8 seems more stable than 0.95
				
				dir = (result[2])                           #is the penetration direction
				
				d = self.properties.shape_collider.center - polyA.center
				
				if d.dotProduct(dir) < 0:
					dir = -dir
			
				
				dir *= overlap
				
				for line_collider,index in self.properties.line_colliders:
					line_collider.x += dir.x
					line_collider.y += dir.y
					line_collider.update()
				
				scp = None
				dcp = None
				
				######################################################################
				# sort the lines
				######################################################################
				
				lines_dict = {}
				smallest_dist = 1000
				#line_to_test = None
				
				#epsilon = 0.01
				for line,index in self.properties.line_colliders:
					c_vec = (line.p1 - line.pos)
					mag = c_vec.get_magnitude()
					c_vec.normalize()
					true_centroid = line.pos + (c_vec * (mag/2.0))
					
					# dist from poly center
					d_vec = true_centroid - polyA.center
					dist = d_vec.get_magnitude()
					
					#if dist < smallest_dist:
					#	smallest_dist = dist
						#line_to_test = (line,index)
					
					lines_dict[dist] = (line,index)
					#epsilon += 0.01
				
				d_keys = lines_dict.keys()
				d_keys.sort()
			
				self.properties.line_colliders = []
				for key in d_keys:
					line = lines_dict[key]
					self.properties.line_colliders.append(line)
			
			
				#################
				# todo : test ALL colliding line segs and store their indices to compute friction
				
				ret = False
				self.offending_index = None
				offending_collider = None
				
				for line_collider,index in self.properties.line_colliders:
					
					if ret:
						break
					
					for line,ind in polyA_lines:
				
						stick = gen_collision.Line(((line_collider.pos.x),(line_collider.pos.y)),
													((line_collider.p1.x),(line_collider.p1.y)))
						line_seg = gen_collision.Line(((line.pos.x),(line.pos.y)),
													((line.p1.x),(line.p1.y)))
					
						#dcp = gen_collision.calculateIntersectPoint((self.collider.pos.x,self.collider.pos.y),(self.collider.p1.x,self.collider.p1.y),
						#										line[0],line[1])
							
						res = stick.collideLine(line_seg)
						if res[0]:
							#print "this is happening"
							dcp = res[1]
						
							
						scp = dcp
					
						if dcp is not None:
							rx = dcp[0] - (dir.x )
							ry = dcp[1] - (dir.y )
							dcp = [rx,ry]
							self.offending_index = index
							offending_collider = line_collider
							
							ret = True
							break
				
				
				for line_collider,index in self.properties.line_colliders:
					line_collider.x -= dir.x
					line_collider.y -= dir.y
					line_collider.update()
				
				
				if scp is not None:
					
					
					#pygame.draw.circle(screen,(0,255,0),(int(scp[0]),int(scp[1])),5)
					#pygame.draw.circle(screen,(255,0,0),(int(dcp[0]),int(dcp[1])),5)
					
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
					dis_vec_n.normalize()
					
					numerator = dis_vec.dotProduct(dis_vec_n) 
					denom =  dis_vec_n.dotProduct(dis_vec_n) * (c1 * c1 + c2 * c2)		# used a normal here 
					mys = numerator/float(denom)
					
					x1 = offending_collider.pointlist[0] + ((dis_vec_n *(c1 * mys * m)))		# used a normal here too 
					x2 = offending_collider.pointlist[1] + ((dis_vec_n *(c2 * mys * m))) 
					
					
					self.currentParticlePos[self.offending_index[0]] = x1
					self.currentParticlePos[self.offending_index[1]] = x2
					
	def ResolveCollisions(self):
		
		self.num_collisions = 0
		self.friction_ratios = [self.norm_friction] * self.NUM_PARTICLES
		
		self.CollideBodies()
		self.CollideGeometry()
		self.CollideBounds()
		
		if self.num_collisions > 0:
			self.in_contact = True
		else:
			self.in_contact = False
		
	def CollideBounds(self):
		
		for i in range(self.NUM_PARTICLES):
			point = self.currentParticlePos[i]
			
			if point.x < 0:
				point.x = 0
				self.friction_ratios[i] = 0.3
				self.num_collisions += 1
			
			elif point.x > WIDTH:
				point.x = WIDTH
				self.friction_ratios[i] = 0.3
				self.num_collisions += 1
				
			if point.y < 0:
				point.y = 0
				self.friction_ratios[i] = 0.3
				self.num_collisions += 1
				
			elif point.y > HEIGHT:
				point.y = HEIGHT
				self.friction_ratios[i] = 0.3
				self.num_collisions += 1
		
	
	def SatisfyConstraints(self):
		
		#self.CollideBounds()
			
		
		for i in range(1):
			
					
			restlength = self.constraint_len
			restlength2 = restlength*restlength
			
			crosslength = self.cross_len
			crosslength2 = crosslength*crosslength
			
			
			delta = self.currentParticlePos[1]-self.currentParticlePos[0]
			delta2 = delta.dotProduct(delta)
			
			x1= self.currentParticlePos[0]
			x2= self.currentParticlePos[1]
			
			#deltalength = math.sqrt(delta.dotProduct(delta))
			#diff = (60-deltalength)/deltalength 	# 60 is the constraint length (had to hack and swap deltalength and 60 around)
			
			diff = restlength2 /(delta2 + restlength2) - 0.5
			#diff *= -2.0
			
			delta *= diff/1.0		# 1.0 here is the combined particle masses
			
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
			
			#############################################################################
			#	cross constraint to prevent collapse
			#############################################################################
			delta = self.currentParticlePos[1] - self.currentParticlePos[3]
			delta2 = delta.dotProduct(delta)
			
			x1=self.currentParticlePos[3]
			x2=self.currentParticlePos[1]
			
			#deltalength = math.sqrt(delta.dotProduct(delta))
			#diff = (60-deltalength)/deltalength 	# 60 is the constraint length (had to hack and swap deltalength and 60 around)
			
			diff = crosslength2 /(delta2 + crosslength2) - 0.5
			#diff *= -2.0
			
			delta *= diff/1.0
			
			x1 -= delta * (0.5) * 1.0 	#* diff		0.5 is the invmass ; 0.4 is the rigidity
			x2 += delta * (0.5) * 1.0	#* diff
			
			#print x1
			self.currentParticlePos[3] = x1
			self.currentParticlePos[1] = x2
			
			
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
			pygame.draw.line(screen,WHITE,(int(self.properties.line_colliders[i][0].pos.x),int(self.properties.line_colliders[i][0].pos.y)),
							(int(self.properties.line_colliders[i][0].p1.x),int(self.properties.line_colliders[i][0].p1.y)))
			#pygame.draw.circle(screen,WHITE,(self.currentParticlePos[i].x,self.currentParticlePos[i].y),5)
			if i < self.NUM_PARTICLES - 1:
				pygame.draw.line(screen,WHITE,(int(self.currentParticlePos[i].x),int(self.currentParticlePos[i].y)),(int(self.currentParticlePos[i+1].x),int(self.currentParticlePos[i+1].y)))
			
	
clock=pygame.time.Clock()
Timepassed=0

pygame.display.set_caption("Verlet Integration")
screen=pygame.display.set_mode((WIDTH,HEIGHT))

# store the sat shape along with the line colliders
world_objects = []


# collidables
#polyA = sat.RectSAT([190,100,(250,0,0)],30,160)
polyA = sat.DEG22RU([300,50,(250,0,0)],30,100)
polyB = sat.DEG45LU([50,50,(250,0,0)],30,50)

#t_line = [((polyA.position.x),(polyA.position.y)),((polyA.p1.x),(polyA.p1.y))]
#r_line = [((polyA.p1.x),(polyA.p1.y)),(float(polyA.p2.x),(polyA.p2.y))]
#b_line = [((polyA.p2.x),(polyA.p2.y)),((polyA.p3.x),(polyA.p3.y))]
#b_line = [(polyA.p2.x,polyA.p2.y),(polyA.position.x,polyA.position.y)]
#l_line = [(polyA.p3.x,polyA.p3.y),(polyA.position.x,polyA.position.y)]
#l_line = [((polyA.p3.x),(polyA.p3.y)),((polyA.position.x),(polyA.position.y))]

t_line = sat.Line((polyA.position.x, polyA.position.y),
					(polyA.p1.x, polyA.p1.y))
					
r_line = sat.Line((polyA.p1.x, polyA.p1.y),
					(polyA.p2.x, polyA.p2.y))

b_line = sat.Line((polyA.p2.x, polyA.p2.y),
					(polyA.position.x, polyA.position.y))

t_line2 = sat.Line((polyB.position.x, polyB.position.y),
					(polyB.p1.x, polyB.p1.y))
					
r_line2 = sat.Line((polyB.p1.x, polyB.p1.y),
					(polyB.p2.x, polyB.p2.y))

b_line2 = sat.Line((polyB.p2.x, polyB.p2.y),
					(polyB.position.x, polyB.position.y))
					
					
#polyA_lines = [t_line,r_line,b_line,l_line]
polyA_lines = [(t_line,0),(r_line,0),(b_line,0)]
polyA_props = ShapeProperties(polyA,polyA_lines)

polyB_lines = [(t_line2,0),(r_line2,0),(b_line2,0)]
polyB_props = ShapeProperties(polyB,polyB_lines)


world_objects.append(polyA_props)
world_objects.append(polyB_props)

# create test system
stick = ParticleSystem(49,29,50,50)
box2 = ParticleSystem(100,60,50,50)
box3 = ParticleSystem(200,60,70,70)

world_objects.append(box2.properties)
world_objects.append(stick.properties)
world_objects.append(box3.properties)

#print box2.properties
#print stick.properties

while True:
	for event in pygame.event.get():
		if event.type==QUIT:
			pygame.quit()
			sys.exit()
			
		if event.type==KEYDOWN:
			if event.key==K_ESCAPE:
				pygame.quit()
				sys.exit()
			
			if event.key == K_z:
				pass
			
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
		stick.currentParticlePos[0].x -= 0.8
	if states[K_RIGHT]:
		stick.currentParticlePos[0].x += 0.8
	if states[K_UP]:
		stick.currentParticlePos[0].y -= 0.8
	if states[K_DOWN]:
		stick.currentParticlePos[0].y += 0.8
	
	
	screen.fill(BLACK)
	
	#stick.TimeStep()
	#box2.TimeStep()
	
	stick.AccumulateForces()
	box2.AccumulateForces()
	box3.AccumulateForces()
	
	stick.Verlet()
	box2.Verlet()
	box3.Verlet()
	
	stick.updateColliders()
	box2.updateColliders()
	box3.updateColliders()
	
	stick.ResolveCollisions()
	stick.updateColliders()
	box2.updateColliders()
	box3.updateColliders()
	
	box2.ResolveCollisions()
	stick.updateColliders()
	box2.updateColliders()
	box3.updateColliders()
	
	box3.ResolveCollisions()
	#stick.updateColliders()
	#box2.updateColliders()
	#box3.updateColliders()
	
	stick.SatisfyConstraints()
	box2.SatisfyConstraints()
	box3.SatisfyConstraints()
	
	
	stick.draw()
	box2.draw()
	box3.draw()
	polyA.draw(screen)
	polyB.draw(screen)
	pygame.display.update()
		
	Timepassed = clock.tick(60)