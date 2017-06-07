import pygame,sys,sat,Vector2,gen_collision
from pygame.locals import*

class AAB(object):
	
	def __init__(self,x,y,w,h):
		
		self.w = w
		self.h = h
		self.xw = w/2
		self.xh = h/2
		
		self.collision_poly = sat.RectSAT([w,h,(0,0,255)],x,y)
		
		self.position = Vector2.Vector(x,y)
		self.vel = Vector2.Vector(0,0)
		self.gravity = Vector2.Vector(0,0.24)
		self.elasticity = 0.0		# should be between 0 and 1
		
		self.accel = 0.5
		self.friction = 0.97
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
		
		num_collisions = 0
		for poly in world_polygons:
			
			if poly.collision_type == "circle":
				result = sat.collidePolyCircle(poly,self.collision_poly)
			else:
				result = sat.collidePolygon(self.collision_poly,poly)
			
			if result[0]:
				num_collisions += 1
				overlap = (abs(result[1]))				#is the penetration amount
				dir = (result[2])                           #is the penetration direction
				d = self.collision_poly.center - poly.center
	
				if d.dotProduct(dir) < 0:
					dir = -dir
				
				dir *= (overlap)
									
				temp_dir = Vector2.Vector(dir.x,dir.y)	
				temp_dir.normalize()
				collision_normal = Vector2.Vector(temp_dir.x,temp_dir.y)
				
					
				self.collision_poly.x += dir.x
				self.collision_poly.y += dir.y
				self.collision_poly.update()
				
				self.position.x = self.collision_poly.x
				self.position.y = self.collision_poly.y
				
				# just elasticity
				#dot2 = self.vel.dotProduct(collision_normal) * (1 + self.elasticity )
				#nx = collision_normal * dot2
				
				# use collision_normal to detect if we're atop a platform
				#print collision_normal
				
				
				# elasticity and friction together
				static_friction = 0.05
				
				vn = collision_normal * self.vel.dotProduct(collision_normal)
				vt = self.vel - vn
				
				if vt.get_magnitude() < 0.01:
					static_friction = 1.01
					
				self.vel = (vt  * (1 - static_friction)) + (vn * - self.elasticity) 
				
				#self.vel = self.vel - nx
				
				# comment this break statement to allow for proper resolution
				# leaving it uncommented removes the corner catching bug by only testing against a single
				# poly each frame. So if you push into places putting you in contact with 2 polys at once
				# you can slip through
				#break
				
		if num_collisions > 0:
			self.in_contact = True
		else:
			self.in_contact = False
		
	def update(self):
		
		self.move()
		
	def draw(self,Surface):
		
		self.collision_poly.draw(Surface)

class Circle(object):
	
	def __init__(self,x,y,radius):
		
		self.radius = radius
		self.w = radius
		self.h = radius
		self.xw = self.w/2
		self.xh = self.h/2
		
		self.collision_poly = sat.CircleSAT((x,y),self.radius)
		
		self.position = Vector2.Vector(x,y)
		self.vel = Vector2.Vector(0,0)
		self.gravity = Vector2.Vector(0,0.19)
		self.elasticity = 0.9		# should be between 0 and 1
		
		self.accel = 0.5
		self.friction = 0.97
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
		self.vel.x *= self.friction
		
		
		# collision resolution here
		if self.position.x < self.radius:
			self.position.x = self.radius
		elif self.position.x  > WIDTH - self.radius:
			self.position.x = WIDTH - self.radius
		if self.position.y < self.radius:
			self.position.y = self.radius
			self.vel.y = 0
		elif self.position.y  > HEIGHT - self.radius:
			self.position.y = HEIGHT - self.radius
			self.vel.y = 0
			
		self.collision_poly.x = self.position.x
		self.collision_poly.y = self.position.y
		self.collision_poly.update()
		
		num_collisions = 0
		for poly in world_polygons:
			
			if poly.collision_type == "circle":
				#continue
				result = sat.collideCircleCircleV(self.position,self.radius,poly.center,poly.radius)
			else:
				result = sat.collidePolyCircle(self.collision_poly,poly)
			
			if result[0]:
				num_collisions += 1
				overlap = (abs(result[1]))				#is the penetration amount
				dir = (result[2])                           #is the penetration direction
				d = self.collision_poly.center - poly.center
	
				if d.dotProduct(dir) < 0:
					dir = -dir
				
				dir *= (overlap)
									
				temp_dir = Vector2.Vector(dir.x,dir.y)	
				temp_dir.normalize()
				collision_normal = Vector2.Vector(temp_dir.x,temp_dir.y)
				
					
				self.collision_poly.x += dir.x
				self.collision_poly.y += dir.y
				self.collision_poly.update()
				
				self.position.x = self.collision_poly.x
				self.position.y = self.collision_poly.y
				
				# just elasticity
				#dot2 = self.vel.dotProduct(collision_normal) * (1 + self.elasticity )
				#nx = collision_normal * dot2
				
				# use collision_normal to detect if we're atop a platform
				#print collision_normal
				
				
				# elasticity and friction together
				static_friction = 0.05
				
				vn = collision_normal * self.vel.dotProduct(collision_normal)
				vt = self.vel - vn
				
				if vt.get_magnitude() < 0.01:
					static_friction = 1.01
					
				self.vel = (vt  * (1 - static_friction)) + (vn * - self.elasticity) 
				
				#self.vel = self.vel - nx
				
				# comment this break statement to allow for proper resolution
				# leaving it uncommented removes the corner catching bug by only testing against a single
				# poly each frame. So if you push into places putting you in contact with 2 polys at once
				# you can slip through
				#break
				
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

player = Circle(40,40,30)

world_polygons = []

#box2 = sat.RectSAT([150,100,(0,255,255)],10,120)
#box = sat.DEG45LU([150,100,(0,255,255)],10,120)
box2 = sat.DEG67LU([150,100,(0,255,255)],160,120)
box = sat.CircleSAT((150,150),40)

ray_color_t = (0,200,0)
ray_color_f = (200,200,255)

world_polygons.append(box)

clip_poly = []
for i in range(len(box2.pointlist)-1,-1,-1):
	ps = box2.pointlist[i]
	clip_poly.append(ps)

world_polygons.append(box2)

s = Vector2.Vector(WIDTH/2,0)

p = Vector2.Vector(0,0)

mouse = Vector2.Vector(0,0)

while True:
	
	for event in pygame.event.get():
		if event.type == QUIT:
			pygame.quit()
			sys.exit()
			
		if event.type == KEYDOWN:
			
			if event.key == K_ESCAPE:
				pygame.quit()
				sys.exit()
		
		if event.type == MOUSEMOTION:
			mx,my = event.pos
			mouse.x = mx
			mouse.y = my
			
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
	
	if keystate[K_j]:
		p.x -= 2
	if keystate[K_l]:
		p.x += 2
	if keystate[K_i]:
		p.y -= 2
	if keystate[K_k]:
		p.y += 2
	
	
	pin = sat.pointInsidePolygon(p,box2.pointlist)
	#print pin
	
	pts = sat.lineCircleClip(box.center.x,box.center.y,box.radius,s,mouse)
	print pts
	
	player.update()
	
	screen.fill((0,0,0))
	
	player.draw(screen)
	
	for item in pts:
		if item is not None:
			pygame.draw.circle(screen,(0,200,20),item,2)
	
	#clip_points = sat.clipSegmentPoly(clip_poly,s,mouse)
	clip_points = sat.clipSegmentPoly(box2.pointlist,s,mouse)
	#clip_points = sat.clipSegmentCircle(box.center,box.radius,s,mouse)
	#print clip_points
	
	dc = ray_color_f
	if clip_points:
		dir = mouse - s
		dist = dir.get_magnitude()
		dir.normalize()
		
		new_dist = dist * clip_points[0]
		dir *= new_dist
		dm = s + dir 
		dc = ray_color_t
	else:
		
		dm = Vector2.Vector(mouse.x,mouse.y) 
		dc = ray_color_f
	
	pygame.draw.line(screen,dc,(s.x,s.y),(dm.x,dm.y))
	pygame.draw.rect(screen,dc,(p.x,p.y,3,3))
	for poly in world_polygons:
		poly.draw(screen)
	
	pygame.display.flip()
	
	
	clock.tick(60)