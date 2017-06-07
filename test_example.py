import math,pygame,sat,gen_collision,Vector2,collide,systems
import sys

from pygame.locals import*

# todo : implement restitution
#		: sleeping objects if velocity drops beneath a certain threshold
#		:  grid based spatial partitions
#		:  calculate masses properly

# bugs line segs behave funny when colliding alone with static bodies
# circles are the buggiest and will fling things in contact with them 

pygame.init()

WIDTH=320
HEIGHT=240

BLACK=(0,0,0)
WHITE=(255,255,255)
GREEN=(0,255,0)
BLUE=(0,0,255)

class ShapeProperties(object):
	
	def __init__(self,shape_collider = None,line_colliders = None):
		
		self.parent = None						# the body that owns the ShapeProperties instance
		self.shape_collider = shape_collider	# the separating axis theorem definition of the shape
		self.line_colliders = line_colliders	# line segment objects used in finding contact points
	
	def setParent(self,parent):
		self.parent = parent
	
	def setShapeCollider(self, shape_collider):
		self.shape_collider = shape_collider
	
	def setLineColliders(self, line_colliders):
		self.line_colliders = line_colliders

class World(object):
	
	def __init__(self):
		
		self.objects = []
	
	def addBody(self,body):
		
		self.objects.append(body)
	
	def init(self):
		for body in self.objects:
			body.updateShape()
		
	def draw(self,Surface):
		
		for body in self.objects:
			body.draw(Surface)
	
	def update(self):
		
		for body in self.objects:
			body.system.AccumulateForces()
		
		for body in self.objects:
			body.system.Verlet()
		
		for body in self.objects:
			if not body.immovable:
				body.updateShape()
		
		for body in self.objects:
			body.system.friction_ratios = [body.system.norm_friction] * body.system.NUM_PARTICLES
			body.system.num_collisions = 0
				
		for body in self.objects:
			body.handleCollision()
			
			for body in self.objects:
				if not body.immovable:
					body.updateShape()
			
		for body in self.objects:
			body.system.SatisfyConstraints()
		
class Body(object):
	"""A physical object"""
	
	def __init__(self,world):
		
		self.world = world
		self.type = None
		self.shape = ShapeProperties()
		self.system = None
		self.mass = 0
		self.radius = None
		self.immovable = True
		self.rect = pygame.Rect(1000,1000,2,2)
		self.polygons = ["line", "poly"]
		self.collidable = False
		
	def link(self,self_index, other_index, other_body):
		
		self.system.currentParticlePos[self_index] = other_body.system.currentParticlePos[other_index]
		self.system.oldParticlePos[self_index] = other_body.system.oldParticlePos[other_index]
	
	def setTimeStep(self, timestep):
		"""Sets the timestep of the body. Smaller values give slow motion effect"""
		
		# timestep : float between 0 and 1
		self.system.timestep = timestep
	
	def setIterations(self, num_iterations):
		"""Sets the number of relaxation loops on the body per timestep"""
		
		if self.system is None:
			raise Exception("Body system has not yet been defined!")
		
		# the higher num_iterations is, the more rigid the body
		self.system.num_iterations = num_iterations
	
	def setFriction(self, friction):
		"""Sets the friction force acting on a bodies when they collide with other stuff"""
		self.system.friction = friction
	
	def setGravity(self, gravity_vector):
		"""Sets the gravity force acting on a body"""
		
		if self.system is None:
			raise Exception("Body system has not yet been defined!")
		
		# the higher num_iterations is, the more rigid the body
		self.system.gravity = gravity_vector
	
	def setWind(self, wind_vector):
		"""Sets the wind force acting on a body"""
		
		if self.system is None:
			raise Exception("Body system has not yet been defined!")
		
		# the higher num_iterations is, the more rigid the body
		self.system.wind = wind_vector
	
	def setInvMass(self, particle_index, mass):
		"""Sets the inverse mass of the particle at specified index"""
		
		if particle_index >= self.system.NUM_PARTICLES:
			raise Exception("the index you have selected is out of range")
		
		self.system.inv_masses[particle_index] = mass 
	
	def handleCollision(self):
		"""Detects and resolves collisions between colliding bodies"""
		
		if self.collidable:
		
			for body in self.world.objects:
				
				
				if body.collidable:
					
					# do a cheap AAB collision check first
					if self.rect.colliderect(body.rect):
						
						
						# use SAT to detect the collisions
						if self.type in self.polygons and body.type in self.polygons:
							collide.collidePolyPoly(self,body)
						
						elif self.type in self.polygons and body.type == "circle":
							collide.collidePolyCircle(self,body)
						
						elif self.type == "circle" and body.type in self.polygons:
							collide.collidePolyCircle(body,self)
						
						elif self.type == "circle" and body.type == "circle":
							collide.collideCircleCircle(self,body)
									
			self.system.CollideBounds()
			
			if self.system.num_collisions > 0:
				self.system.in_contact = True
			else:
				self.system.in_contact = False
		
		else:
			# only collide with bounds if not collidable
			self.system.CollideBounds()
		
	def updateShape(self):
		"""Updates the body's collision detection data"""
		
		if self.collidable:
			if self.type == "poly":
			
				line_colliders = []
				
				for i in range(len(self.system.currentParticlePos)-1):
					
					collider = sat.Line((self.system.currentParticlePos[i].x, self.system.currentParticlePos[i].y),
										(self.system.currentParticlePos[i+1].x, self.system.currentParticlePos[i+1].y))
					
					
					collider.type = "dynamic"
					line_colliders.append([collider,(i,i+1)])
				
				collider = sat.Line((self.system.currentParticlePos[len(self.system.currentParticlePos)-1].x, self.system.currentParticlePos[len(self.system.currentParticlePos)-1].y),
										(self.system.currentParticlePos[0].x, self.system.currentParticlePos[0].y))
					
					
				line_colliders.append([collider,(len(self.system.currentParticlePos)-1,0)])
				
				self.shape.setLineColliders(line_colliders)
				
				# create collision rect
				minx = 10000
				miny = 10000
				maxx = -10000
				maxy = -10000
				
				points = []
				for v in self.system.currentParticlePos:
					
					if v.x < minx:
						minx = v.x
					if v.x > maxx:
						maxx = v.x
					if v.y < miny:
						miny = v.y
					if v.y > maxy:
						maxy = v.y
					
					
					point = v.to_point()
					points.append(point)
				
				#self.rect = pygame.Rect(minx,miny,abs(maxx-minx),abs(maxy-miny))
				
				self.rect.x = minx -4
				self.rect.y = miny -4
				self.rect.w = abs(maxx-minx) + 10
				self.rect.h = abs(maxy-miny) + 10
				
				#print self.rect
				
				shape_collider = sat.Polygon(points)
				
				self.shape.setShapeCollider(shape_collider)
				
			elif self.type == "circle":
				
				v = self.system.currentParticlePos[0]
				point = v.to_point()
					
				shape_collider = sat.CircleSAT(point, self.radius)
				
				self.shape.setShapeCollider(shape_collider)
				
				#self.rect = pygame.Rect(point[0]-self.radius,point[1]-self.radius,self.radius*2,self.radius*2)
				self.rect.x = (point[0]-self.radius) - 4
				self.rect.y = (point[1]-self.radius) - 4
				self.rect.w = (self.radius*2) + 10
				self.rect.h = self.rect.w
				
				
			elif self.type == "line":
				
				line_colliders = []
				collider = sat.Line((self.system.currentParticlePos[0].x, self.system.currentParticlePos[0].y),
										(self.system.currentParticlePos[1].x, self.system.currentParticlePos[1].y))
				
				collider.type = "dynamic"
				line_colliders.append([collider,(0,1)])
				
				
				# create collision rect
				minx = 10000
				miny = 10000
				maxx = -10000
				maxy = -10000
				
				for v in self.system.currentParticlePos:
					
					if v.x < minx:
						minx = v.x
					if v.x > maxx:
						maxx = v.x
					if v.y < miny:
						miny = v.y
					if v.y > maxy:
						maxy = v.y
				
				#self.rect = pygame.Rect(minx,miny,abs(maxx-minx),abs(maxy-miny))
				self.rect.x = minx -5
				self.rect.y = miny -5
				self.rect.w = abs(maxx-minx) + 10
				self.rect.h = abs(maxy-miny) + 10
				
				
				self.shape.setLineColliders(line_colliders)	
				
				shape_collider = sat.Line((self.system.currentParticlePos[0].x, self.system.currentParticlePos[0].y),
										(self.system.currentParticlePos[1].x, self.system.currentParticlePos[1].y))
				
				self.shape.setShapeCollider(shape_collider)
				
		
	def setAsPoly(self,mass,vertices, c_constraints = None, stiffness=1.0, c_stiffness=1.0):
		"""Initialises the body as a convex polygon type based on arguments provided"""
		
		self.system = systems.PolySystem(self, vertices, mass, c_constraints, stiffness, c_stiffness)
		self.type = "poly"
		self.mass = mass
		self.collidable = True
		
		if self.mass > 100000:
			self.immovable = True
		else:
			self.immovable = False
	
	def setAsLine(self,mass,vertices, stiffness=1.0, c_stiffness=1.0):
		"""Initialises the body as a stick segment type based on arguments provided"""
		
		self.type = "line"
		self.system = systems.PolySystem(self, vertices, mass , None, stiffness, c_stiffness)
		self.mass = mass
		self.collidable = True
		
		if self.mass > 100000:
			self.immovable = True
		else:
			self.immovable = False
	
	def setAsCircle(self, mass, pos, radius):
		"""Initialises the body as a circular type based on arguments provided"""
		
		# Circles are buggy and need serious fixing up
		# Use them if you don't mind experiencing some temperamental collision response :D
		
		self.system = systems.CircleSystem(self, pos, radius, mass)
		self.type = "circle"
		self.radius = radius
		self.mass = mass
		self.collidable = True
		
		if self.mass > 0:
			self.immovable = False
		else:
			self.immovable = True
	
	def setAsFree(self,particle_list, constraint_list):
		"""Initialises the body to a free form type(for ragdolls and such) based on arguments provided"""		
		
		# @param particle_list : a list of lists; with each inner list having the following
		# format : [(x,y),invmass] - an invmass of 0 makes the  particle immovable
		# @param constraint_list : a list containing inner lists of the format
		# : [length,(start_index,end_index),stiffness] - indices here refer to the particles which the constraint
		# forms between
		
		self.system = systems.FreeSystem(self, particle_list, constraint_list)
		self.immovable = False
	
	def draw(self,Surface):
		"""Draws the body to a specified surface"""
		self.system.draw(Surface)
		
		
clock = pygame.time.Clock()
Timepassed=0

pygame.display.set_caption("Verlet Integration")
screen=pygame.display.set_mode((WIDTH,HEIGHT))

w = World()
b1 = Body(w)
b2 = Body(w)
b3 = Body(w)
b4 = Body(w)
b5 = Body(w)

#plist = [[(40,40),0.5],[(70,50),0.5]]
#clist = [[60,(0,1),1.0]]

plist = [[(127, 39), 1.0], [(113, 49), 1.0], [(137, 51), 1.0], [(123, 61), 1.0], [(119, 79), 1.0], [(94, 79), 1.0], [(139, 82), 1.0], [(155, 94), 1.0], [(72, 91), 1.0], [(114, 97), 1.0], [(123, 113), 1.0], [(130, 133), 1.0], [(98, 106), 1.0], [(80, 128), 1.0]]
clist = [[17.204650534085253, (2, 3), 1.0], [17.204650534085253, (0, 1), 1.0], [15.620499351813308, (0, 2), 1.0], [15.620499351813308, (1, 3), 1.0], [18.439088914585774, (3, 4), 1.0], [25.0, (4, 5), 1.0], [20.223748416156685, (4, 6), 1.0], [20.0, (6, 7), 1.0], [25.059928172283335, (5, 8), 1.0], [18.681541692269406, (4, 9), 1.0], [18.357559750685819, (9, 12), 1.0], [28.42534080710379, (12, 13), 1.0], [18.357559750685819, (9, 10), 1.0], [21.189620100417091, (10, 11), 1.0]]

plist2 = [[(36.0, 55.0), 1.0], [(49.0, 69.0), 1.0], [(23.0, 69.0), 1.0], [(35.0, 83.0), 1.0]]
clist2 = [[19.104973174542799, (0, 1), 1.0], [19.798989873223331, (1, 3), 1.0], [18.439088914585774, (3, 2), 1.0], [19.104973174542799, (0, 2), 1.0], [28.0178514522438, (0, 3), 1.0], [26.0, (1, 2), 1.0]]

#plist=[[(53.0, 40.0), 0.0], [(58.0, 50.0), 1.0], [(58.0, 60.0), 1.0], [(58.0, 70.0), 1.0], [(63.0, 80.0), 1.0], [(63.0, 90.0), 1.0], [(63.0, 100.0), 1.0], [(63.0, 110.0), 1.0]]
#clist=[[11.180339887498949, (0, 1), 1.0], [10.0, (1, 2), 1.0], [10.0, (2, 3), 1.0], [11.180339887498949, (3, 4), 1.0], [10.0, (4, 5), 1.0], [10.0, (5, 6), 1.0], [10.0, (6, 7), 1.0]]

b5.setAsFree(plist,clist)
b2.setAsFree(plist2,clist2)

cr1 = [(0,2),(1,3)]
cr = [(0,3),(4,1),(2,5)]
b1.setAsPoly(1,[Vector2.Vector(50,20),Vector2.Vector(70,20),Vector2.Vector(70,40),Vector2.Vector(50,40)],cr1,1,0.5)
#b1.setAsPoly(4,[Vector2.Vector(10,0),Vector2.Vector(20,0),Vector2.Vector(30,10),
#	Vector2.Vector(20,20),Vector2.Vector(10,20),Vector2.Vector(0,10)],cr)

b3.setAsPoly(18,[Vector2.Vector(20,20),Vector2.Vector(40,0),Vector2.Vector(40,20)])
#b4.setAsPoly(1000000,[Vector2.Vector(0,100),Vector2.Vector(190,100),Vector2.Vector(190,150),Vector2.Vector(0,150)])
#b4.setAsPoly(1000000,[Vector2.Vector(0,100),Vector2.Vector(190,120),Vector2.Vector(0,120)])
b4.setAsPoly(1000000,[Vector2.Vector(30,150),Vector2.Vector(190,100),Vector2.Vector(190,150)])
#b2.setAsLine(10,[Vector2.Vector(10,26),Vector2.Vector(50,26)])
#b5.link(7,2,b1)
#b1.link(1,2,b3)
#b1.link(2,0,b5)
#b2.setAsCircle(1000000,Vector2.Vector(150,200),30)
#b1.setIterations(4)
#b5.setIterations(4)
#b1.setGravity(Vector2.Vector(0,0.0012))
#b2.setGravity(Vector2.Vector(0,0.0012))
#b5.setGravity(Vector2.Vector(0,0.0012))
#b1.setInvMass(0,0.0)
b2.setInvMass(2,0.0)
#b2.setInvMass(1,0.0)
b2.link(1,0,b5)
#b5.setWind(Vector2.Vector(0.18,0))
#b1.setFriction(0.8)
#b5.setTimeStep(0.3)
#b3.setTimeStep(0.3)
#linking a particle with an inverse mass of 0 to another object doesnt work well
# unless that other objects' particle also sets its inverse mass to 0

w.addBody(b1)
w.addBody(b2)
w.addBody(b3)
w.addBody(b4)
w.addBody(b5)
w.init()

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
				
		if event.type==KEYUP:
			pass
	
	
	states = pygame.key.get_pressed()
	if states[K_LEFT]:
		b1.system.currentParticlePos[0].x -= 0.8
	if states[K_RIGHT]:
		b1.system.currentParticlePos[0].x += 0.8
	if states[K_UP]:
		b1.system.currentParticlePos[0].y -= 1.8
	if states[K_DOWN]:
		b1.system.currentParticlePos[0].y += 0.8
	
	#print b4.system.num_collisions
	
	w.update()
	
	screen.fill(BLACK)
	
	w.draw(screen)
	
	pygame.display.update()
		
	Timepassed = clock.tick(60)