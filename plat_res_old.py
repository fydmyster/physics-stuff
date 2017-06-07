import pygame,sys,sat,Vector2,gen_collision
from pygame.locals import*

class AAB(object):
	
	def __init__(self,x,y,w,h):
		
		self.w = w
		self.h = h
		self.xw = w/2
		self.xh = h/2
		
		self.collision_poly = sat.RectSAT([w,h,(0,0,255)],x,y)
		self.collision_lines = [] 
		l1 = sat.Line((self.collision_poly.position.x,self.collision_poly.position.y),
					(self.collision_poly.p1.x,self.collision_poly.p1.y))
		
		l2 = sat.Line((self.collision_poly.p1.x,self.collision_poly.p1.y),
					(self.collision_poly.p2.x,self.collision_poly.p2.y))
		
		l3 = sat.Line((self.collision_poly.p2.x,self.collision_poly.p2.y),
					(self.collision_poly.p3.x,self.collision_poly.p3.y))
		
		l4 = sat.Line((self.collision_poly.p3.x,self.collision_poly.p3.y),
					(self.collision_poly.position.x,self.collision_poly.position.y))
		
		self.collision_lines = [l1,l2,l3,l4]
		
		self.position = Vector2.Vector(x,y)
		self.vel = Vector2.Vector(0,0)
		self.gravity = Vector2.Vector(0,0.19)
		self.elasticity = 0.0		# should be between 0 and 1
		
		self.accel = 0.5
		self.friction = 0.93
		self.static_friction = 0.6
		
		self.ml = False
		self.mr = False
		self.md = False
		self.mu = False
		self.in_contact = False
	
	def move(self):
		
		#self.vel *= self.friction
		
		
		if self.ml:
			self.vel.x -= self.accel
		if self.mr:
			self.vel.x += self.accel
		if self.mu:
			self.vel.y -= self.accel
		if self.md:
			self.vel.y += self.accel
	
		self.vel += self.gravity
		self.position += self.vel
		self.vel *= self.friction
		
		
		# collision resolution here
		if self.position.x < 0:
			self.position.x = 0
		elif self.position.x  > WIDTH - self.w:
			self.position.x = WIDTH - self.w
		if self.position.y < 0:
			self.position.y = 0
		elif self.position.y  > HEIGHT - self.h:
			self.position.y = HEIGHT - self.h
		
		self.collision_poly.x = self.position.x
		self.collision_poly.y = self.position.y
		self.collision_poly.update()
		
		self.collision_lines = [] 
		l1 = sat.Line((self.collision_poly.position.x,self.collision_poly.position.y),
					(self.collision_poly.p1.x,self.collision_poly.p1.y))
		
		l2 = sat.Line((self.collision_poly.p1.x,self.collision_poly.p1.y),
					(self.collision_poly.p2.x,self.collision_poly.p2.y))
		
		l3 = sat.Line((self.collision_poly.p2.x,self.collision_poly.p2.y),
					(self.collision_poly.p3.x,self.collision_poly.p3.y))
		
		l4 = sat.Line((self.collision_poly.p3.x,self.collision_poly.p3.y),
					(self.collision_poly.position.x,self.collision_poly.position.y))
		
		self.collision_lines = [l1,l2,l3,l4]
		
		num_collisions = 0
		for poly,line_segs in world_polygons:
			
			result = sat.collidePolygon(self.collision_poly,poly)
			
			if result[0]:
				num_collisions += 1
				overlap = (abs(result[1]))				#is the penetration amount
				dir = (result[2])                           #is the penetration direction
				d = self.collision_poly.center - poly.center
	
				if d.dotProduct(dir) < 0:
					dir = -dir
				
				line_dir =Vector2.Vector(dir.x,dir.y)
				line_dir *= (overlap * 0.99)
				
				dir *= (overlap)
				
				for line_collider in self.collision_lines:
					line_collider.x += line_dir.x
					line_collider.y += line_dir.y
				
				lines_dict = {}
				smallest_dist = 1000
				#line_to_test = None
				
				#epsilon = 0.01
				for line in self.collision_lines:
					c_vec = (line.p1 - line.pos)
					mag = c_vec.get_magnitude()
					c_vec.normalize()
					true_centroid = line.pos + (c_vec * (mag/2.0))
					
					# dist from poly center
					d_vec = true_centroid - poly.center
					dist = d_vec.get_magnitude()
					
					lines_dict[dist] = line
						
				d_keys = lines_dict.keys()
				d_keys.sort()
			
				self.collision_lines = []
				
				for key in d_keys:
					line = lines_dict[key]
					self.collision_lines.append(line)
				
				ret = False
				for line in line_segs:
					
				# test line segs for collision normal
					
					if ret:
						break
					
					for line_collider in self.collision_lines:
				
					
						seg_d = gen_collision.Line(((line_collider.pos.x),(line_collider.pos.y)),
													((line_collider.p1.x),(line_collider.p1.y)))
						seg_s = gen_collision.Line(line[0],line[1])
												
					
						res = seg_d.collideLine(seg_s)
						
						if res[0]:
							
							collision_edge = seg_s
							collision_vec = Vector2.Vector.from_points(line[0],line[1])
							line_dir.normalize()
							td = Vector2.Vector(dir.x,dir.y)
							
							td.normalize()
							collision_vec.normalize()
							collision_normal = Vector2.Vector(td.x,td.y)
							contact_normal = Vector2.Vector(collision_vec.y,-collision_vec.x)
							
							ret = True
							break
							
				#print type(collision_normal)
				self.collision_poly.x += dir.x
				self.collision_poly.y += dir.y
				self.collision_poly.update()
				
				self.position.x = self.collision_poly.x
				self.position.y = self.collision_poly.y
				
				# just elasticity
				#dot2 = self.vel.dotProduct(collision_normal) * (1 + self.elasticity )
				#nx = collision_normal * dot2
				
				
				# elasticity and friction together
				static_friction = 1
				
				vn = contact_normal * self.vel.dotProduct(contact_normal)
				vt = self.vel - vn
				
				print vt.get_magnitude()
				if vt.get_magnitude() < 0.01:
					static_friction = 1.01
					#print "wtf"
					
				self.vel = (vt  * (1 - static_friction)) + (vn * - self.elasticity) 
				
				#self.vel = self.vel - nx
		
		if num_collisions > 0:
			self.in_contact = True
		else:
			self.in_contact = False
		
	def update(self):
		
		self.move()
		
	def draw(self,Surface):
		
		self.collision_poly.draw(Surface)

pygame.init()

WIDTH = 320
HEIGHT = 240

pygame.display.set_caption("Platformer collision resolution")
screen = pygame.display.set_mode((WIDTH,HEIGHT))

timepassed = 0
clock = pygame.time.Clock()

player = AAB(40,40,30,30)

img = pygame.image.load("hair.png").convert()
img.set_colorkey((255,0,255))

world_polygons = []

box = sat.RectSAT([150,100,(0,255,255)],10,120)
box2 = sat.DEG45RU([150,100,(0,255,255)],160,120)

l1 = [(box.position.x,box.position.y),(box.p1.x,box.p1.y)]
l2 = [(box.p1.x,box.p1.y),(box.p2.x,box.p2.y)]
#l3 = [(box.p2.x,box.p2.y),(box.position.x,box.position.y)]
l3 = [(box.p2.x,box.p2.y),(box.p3.x,box.p3.y)]
l4 = [(box.p1.x-10,box.p1.y),(box.p1.x+10,box.p1.y)]

lines = [l1,l2,l3,l4]
world_polygons.append((box,lines))

l1 = [(box2.position.x,box2.position.y),(box2.p1.x,box2.p1.y)]
l2 = [(box2.p1.x,box2.p1.y),(box2.p2.x,box2.p2.y)]
l3 = [(box2.p2.x,box2.p2.y),(box2.position.x,box2.position.y)]
#l3 = [(box2.p2.x,box2.p2.y),(box2.p3.x,box2.p3.y)]
#l4 = [(box2.p3.x,box2.p3.y),(box2.position.x,box2.position.y)]

lines = [l1,l2,l3]
world_polygons.append((box2,lines))


while True:
	
	for event in pygame.event.get():
		if event.type == QUIT:
			pygame.quit()
			sys.exit()
			
		if event.type == KEYDOWN:
			
			if event.key == K_ESCAPE:
				pygame.quit()
				sys.exit()
		
	keystate = pygame.key.get_pressed()
	
	if keystate[K_LEFT]:
		player.ml = True
	else:
		player.ml = False
	
	if keystate[K_RIGHT]:
		player.mr = True
	else:
		player.mr = False
	
	if keystate[K_UP]:
		player.mu = True
	else:
		player.mu = False
	
	if keystate[K_DOWN]:
		player.md = True
	else:
		player.md = False
	
	player.update()
	
	screen.fill((50,150,0))
	
	player.draw(screen)
	screen.blit(img,(0,0))
	
	for poly,segs in world_polygons:
		poly.draw(screen)
	
	pygame.display.flip()
	
	
	clock.tick(60)