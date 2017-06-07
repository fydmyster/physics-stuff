import sys,pygame,math
import Vector2
from pygame.locals import*

def clipSegmentPoly(vertices, start, end , cw = True):
	"""Clips a line segment to a polygon"""
	# param : vertices - vectors representing the points of a convex polygon
	# param : start - line starting position (vector)
	# param : end - line end position (vector)
	# param : cw - the polygon winding ; True for clockwise, False for counter-clockwise 
	
	# returns the relative distances to the far and near polygon planes intersected by the segment
	
	# init 
	tnear = 0.0 
	tfar = 1.0
	
	x_dir = end - start
	
	# test separating axes of vertices
	j = len(vertices) - 1
	for i in range(len(vertices)):
		
		e0 = vertices[j]
		e1 = vertices[i]
		e = e1 - e0
		
		if cw:
			en = Vector2.Vector(e.y, -e.x)	# this is a left hand normal; use rhn if vertice windings are counter-clockwise
		else:
			en = Vector2.Vector(-e.y, e.x)	# this is a right hand normal; use lhn if vertice windings are clockwise

		d = e0 - start
		
		denom = d.dotProduct(en)
		numer = x_dir.dotProduct(en)
		
		if abs(numer) < 1.0e-8:
			if denom < 0.0:
				return False
		
		else:
			
			tclip = denom / float(numer)
			
			# near intersection
			if numer < 0.0:
				
				if tclip > tfar:
					return False
				
				if tclip > tnear:
					tnear = tclip
					nnear = Vector2.Vector(en.x,en.y)
					nnear.normalize()
			
			else:
				if tclip < tnear:
					return False
					
				if tclip < tfar:
					tfar = tclip
					nfar = Vector2.Vector(en.x,en.y)
					nfar.normalize()
		
		j = i
		
	return tnear,tfar

def lineCircleClip(cx, cy, radius, start, end):
	"""Computes the entry and exit point of a line through a circle"""
	dx = end.x - start.x
	dy = end.y - start.y
		
	a = dx * dx + dy * dy	
	b = 2 * (dx * (start.x - cx) + dy * (start.y - cy))
	c = (start.x - cx) * (start.x - cx) + (start.y - cy) * (start.y - cy) - radius * radius
	
	det = b * b - 4 * a * c
	
	if a <= 0.00000001 or det < 0:
		
		return (None,None)
		
	elif det == 0:
		
		# one solution
		t = -b / (2 * a)
		
		p1 = (start.x + t * dx, start.y + t * dy)
		
		return (p1,None)
		
	else:
		# two solutions
		t = float((-b + math.sqrt(det))/(2*a))
		p1 = (start.x + t * dx, start.y + t * dy)
		
		t = float((-b - math.sqrt(det))/(2*a))
		p2 = (start.x + t * dx, start.y + t * dy)
		
		return (p1,p2)
			
def pointInsidePolygon(point, vertices, cw = True):
	"""Checks if a point is contained inside a polygon"""
	# param : vertices - list containing vectors relating to the vertices making up the polygon
	# param : p - the point we are testing. a vector should be passed
	# param : cw - True or False; indicates the winding direction of the vertices provided
	
	j = 2 
	for i in range(len(vertices)):
		
		#if i == len(vertices)-1:
			#j = i
			#i = 0
		#	pass
		
		dir = vertices[i] - vertices[j]
		normal = Vector2.Vector(-dir.y, dir.x)
		d = point - vertices[j]
		
		sign = d.dotProduct(normal)
		
		if not cw:
			if sign > 0.0:
				return False
		else:
			if sign < 0.0:
				return False
		j = i
		
	return True
	
def collideCircleCircleV(c1,r1,c2,r2):
	"""Computes if there is a collision between two circles. Takes vectors as arguments for c1 and c2 """
	# compute distance between points
	dv = c2 - c1
	dist = dv.get_magnitude()
	
	rad_sum = r1 + r2
	
	if dist > rad_sum:
		# no collision return
		return (False,)
	
	else:
		min_overlap = abs(rad_sum - dist)
		dv.normalize()
		pDirection = dv
		
		return (True,min_overlap,pDirection)
		
		
def collidePolyCircle(circle,polygon):
	"""Detects a collision between a convex polygon and circle using SAT"""
	# get axes to test against
	axes_to_test = []
	min_overlap = 100
	
	# get lh normals for all egdes
	# these are the axes we'll project onto
	for edge in polygon.edgelist:
		tempAxisA = Vector2.Vector(-(edge.y),edge.x)
		tempAxisA.normalize()
		axes_to_test.append(tempAxisA)
		
	# find closest vertice to circle in polygon
	distances = []
	for p in polygon.pointlist:
		vec = Vector2.Vector.from_points((p.x,p.y),(circle.x,circle.y))
		dist = vec.get_magnitude()
		axis = [dist,vec]
		distances.append(axis)
		
	min_dist = distances[0]
	for dist,axis in distances:
		if dist < min_dist[0]:
			min_dist = (dist,axis)
		
	tempAxisB = min_dist[1]
	tempAxisB.normalize()
	axes_to_test.append(tempAxisB)
		
	# calculate circles pointlist
	circle_pointslist = [Vector2.Vector(circle.x,circle.y),Vector2.Vector(circle.x,circle.y)]
		
	for axis in axes_to_test:
		
		min1=axis.dotProduct(polygon.pointlist[0])
		max1=min1
		
		min2=axis.dotProduct(circle_pointslist[0])
		max2=min2
		
		for vertex in polygon.pointlist:
			projA=axis.dotProduct(vertex)
			#projDot=Vector2.Vector(axis.x*proj,axis.y*proj)
			
			if projA<min1:
				min1=projA
			if projA>max1:
				max1=projA
			
		for i in range(len(circle_pointslist)):
			if i == 0:
				projB = (axis.dotProduct(circle_pointslist[i])) + circle.radius
					
				if projB<min2:
					min2=projB
				if projB>max2:
					max2=projB
				
			else:
				projB = (axis.dotProduct(circle_pointslist[i])) - circle.radius
					
				if projB<min2:
					min2=projB
				if projB>max2:
					max2=projB
				
		test1=min1-max2
		test2=min2-max1
		
		if test1>0 or test2>0 :
			return (False,)
		
		test1Interval=abs(test1)
		test2Interval=abs(test2)
		
		if test1Interval<min_overlap:
			min_overlap=test1Interval
			pDirection=axis
		
		if test2Interval<min_overlap:
			min_overlap=test2Interval
			pDirection=axis
					
	return (True,min_overlap,pDirection)
	

def collidePolygon(polyA,polyB):
	"""Detects a collision between 2 convex polygons using SAT"""
	
	axes_to_test=[]
	min_overlap = 100
	
	# get lh normals for all egdes
	# these are the axes we'll project onto
	for edge in polyA.edgelist:
		tempAxisA=Vector2.Vector(-(edge.y),edge.x)
		tempAxisA.normalize()
		axes_to_test.append(tempAxisA)
	
	for edge in polyB.edgelist:
		tempAxisB=Vector2.Vector(-(edge.y),edge.x)
		tempAxisB.normalize()
		axes_to_test.append(tempAxisB)
		
	for axis in axes_to_test:
		
		min1 = axis.dotProduct(polyA.pointlist[0])
		max1 = min1
		
		min2 = axis.dotProduct(polyB.pointlist[0])
		max2 = min2
		
		for vertex in polyA.pointlist:
			projA = axis.dotProduct(vertex)
			#projDot=Vector2.Vector(axis.x*proj,axis.y*proj)
			
			if projA<min1:
				min1=projA
			if projA>max1:
				max1=projA
			
		for vertex in polyB.pointlist:
			projB=axis.dotProduct(vertex)
			
			if projB<min2:
				min2=projB
			if projB>max2:
				max2=projB
		
		test1 = min1-max2
		test2 = min2-max1
		
		if test1 > 0 or test2 > 0 :
			return (False,)
		
		test1Interval=abs(test1)
		test2Interval=abs(test2)
		
		if test1Interval<min_overlap:
			min_overlap=test1Interval
			pDirection=axis
		
		if test2Interval<min_overlap:
			min_overlap=test2Interval
			pDirection=axis
					
	return (True,min_overlap,pDirection)

def resolveCollision(result, polyA, polyB, resolve_type = 0):
	"""Resolves collision along axis of least penetration"""
	
	# resolve_type : 0 - projects both polygons out of collision
	#			     1 - projects only polyA out of collision
	#				 2 - projects only polyB out of collision
	
	overlap = (abs(result[1]))				#is the penetration amount
	dir = (result[2])                           #is the penetration direction
	d = polyA.center - polyB.center
	
	if d.dotProduct(dir) < 0:
		dir = -dir
		
	if resolve_type == 0:
		
		dir *= (overlap * 0.5)
		
		# move second shape out the way
		polyB.x -= dir.x
		polyB.y -= dir.y
		
		# if moving the other shape negate the operator acting on dir
		# move both shapes
		polyA.x += dir.x
		polyA.y += dir.y
		
			
	elif resolve_type == 1:
		
		dir *= overlap
		
		polyA.x += dir.x
		polyA.y += dir.y
		
			
	elif resolve_type == 2:
		
		dir *= overlap
		
		polyB.x -= dir.x
		polyB.y -= dir.y
		
			
	
		
class DEG45LU(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w, self.h)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
			
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
class DEG45RU(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w, self.h)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)

class DEG45LD(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w, self.h)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)

class DEG45RD(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w, self.h)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)

class DEG22LU(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w, self.h/2)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
			
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)

class DEG22RU(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w, self.h/2)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
	
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)

class DEG22LD(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w, self.h/2)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
class DEG22RD(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w, self.h/2)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)

class DEG67LU(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w/2, self.h)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)

class DEG67RU(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w/2, self.h)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.bottomleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.bottom + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)

class DEG67LD(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w/2, self.h)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topright, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.right + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.right, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
class DEG67RD(object):
	def __init__(self,param_list,x,y):
		
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the rect
		# param_list : [width,height,color]
		# width and height should be equal ideally
		# width,height are the length and the breadth of the rect area
		# color can be an RGB tuple or None ; if None the polygon is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x, self.y, self.w/2, self.h)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(3):
				if i==2:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
	
	def update(self):
	
		# update rect
		self.rect.topleft = (self.x, self.y)
		
		# calculate shape center
		vector_to_center = Vector2.Vector.from_points(self.rect.topleft, self.rect.center)
		distance_to_center = vector_to_center.get_magnitude()
		
		# normalize vector_to_center
		vector_to_center.normalize()
		vector_to_center *= (distance_to_center/2)
		
		centerx = self.rect.left + vector_to_center.x
		centery = self.rect.top + vector_to_center.y
		
		self.center=Vector2.Vector(centerx,centery)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.rect.left, self.rect.top)
		self.p1 = Vector2.Vector(self.rect.right, self.rect.top)
		self.p2 = Vector2.Vector(self.rect.left, self.rect.bottom)
		self.pointlist=[]
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.position.x,self.position.y))
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)

		
class Line(object):
	def __init__(self,point1,point2):
		# @param point1 and point2 are tuples
		self.x=point1[0]
		self.y=point1[1]
		
		self.x2=point2[0]
		self.y2=point2[1]
		
		self.type = "static"
		self.collision_type = "poly"
		
		# calculate x and y distances between points for use in the update method for movement
		self.xdistance=self.x2-self.x
		self.ydistance=self.y2-self.y
		
		# points of shapes represented as vectors
		
		self.pos=Vector2.Vector(self.x,self.y)
		self.p1=Vector2.Vector(self.x2,self.y2)
		
		# calculate center
		vector_points = self.p1 - self.pos
		mag = vector_points.get_magnitude()
		
		
		vector_points.normalize()
		#print dir
		vector_points *= (mag/2.0)
		
		self.center = self.pos + vector_points
		
		self.pointlist=[]
		self.pointlist.append(self.pos)
		self.pointlist.append(self.p1)
		
		
		# create edge
		self.egde=Vector2.Vector.from_points((self.pos.x,self.pos.y),(self.p1.x,self.p1.y))
		
		self.edgelist=[]
		self.edgelist.append(self.egde)
		
		self.ML=False
		self.MR=False
		self.MU=False
		self.MD=False
		
	def draw(self,SCREEN):
		pygame.draw.line(SCREEN,BLACK,(self.pointlist[0].x,self.pointlist[0].y),(self.pointlist[1].x,self.pointlist[1].y))
		
	
	def update(self):
	
		self.pos=Vector2.Vector(self.x,self.y)
		self.p1=Vector2.Vector(self.pos.x+self.xdistance,self.pos.y+self.ydistance)
		
		self.pointlist=[]
		self.pointlist.append(self.pos)
		self.pointlist.append(self.p1)
		
		# calculate center
		vector_points = self.p1 - self.pos
		mag = vector_points.get_magnitude()
		
		vector_points.normalize()
		#print dir
		vector_points *= (mag/2.0)
		
		self.center = self.pos + vector_points
		
		# create edges
		self.egde=Vector2.Vector.from_points((self.pos.x,self.pos.y),(self.p1.x,self.p1.y))
		self.edgelist=[]
		self.edgelist.append(self.egde)
		
	def move(self):
		if self.ML:
			self.x-=1
		elif self.MR:
			self.x+=1
		elif self.MU:
			self.y-=1
		elif self.MD:
			self.y+=1

class Point(object):
	def __init__(self,point1):
		# @param point1 is an (x,y) tuple
		self.x=point1[0]
		self.y=point1[1]
		
		self.type = "static"
		self.collision_type = "poly"
		# points of shapes represented as vectors
		
		self.pos=Vector2.Vector(self.x,self.y)
		
		self.pointlist=[]
		self.pointlist.append(self.pos)
		
		# create edge
		self.egde=Vector2.Vector.from_points((self.pos.x,self.pos.y),(self.pos.x+0.0001,self.pos.y+0.0001))
		
		self.edgelist=[]
		self.edgelist.append(self.egde)
		
	def draw(self,Surface):
		pygame.draw.rect(Surface,(0,200,0),(self.pointlist[0].x,self.pointlist[0].y,2,2))
		
	def update(self):
		self.pos = Vector2.Vector(self.x,self.y)
		
		self.pointlist=[]
		self.pointlist.append(self.pos)
		
		# create edge
		self.egde=Vector2.Vector.from_points((self.pos.x,self.pos.y),(self.pos.x,self.pos.y))
		
		self.edgelist=[]
		self.edgelist.append(self.egde)

class CircleSAT(object):
	"""Collision rect used for SAT collision detection (can only test against polygons, not other CircleSAT objects)"""
	def __init__(self,center_point,radius=20,color = (255,0,0)):
		
		self.x, self.y = center_point
		self.collision_type = "circle"
		self.radius = radius
		self.color = color
		self.center=Vector2.Vector(self.x,self.y)
	
	def draw(self,Surface):
		
		pygame.draw.circle(Surface,self.color,(int(self.x),int(self.y)),self.radius,1)
		
	def update(self):
		self.center = Vector2.Vector(self.x,self.y)

class Polygon(object):
	def __init__(self,param_list, color = None):
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the box
		# param_list : [a list of points used to build the collision shape] - clockwise winding
		# color can be an RGB tuple or None ; if None the rect is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = param_list[0][0]
		self.y = param_list[0][1]
		self.color = color
		self.pointlist = []
		self.edgelist = []
		
		# todo : calculate the centroid of a polygon
		
		centroid = self.calculateCentroid(param_list)
		self.center = Vector2.Vector(centroid[0],centroid[1])
		
		# points of shapes represented as vectors
		for point in param_list:
			v = Vector2.Vector(point[0],point[1])
			self.pointlist.append(v)
			
		for i in range(len(self.pointlist)):
			if i == len(self.pointlist)-1:
				j = 0
			else:
				j = i+1
				
			edge = Vector2.Vector.from_points((self.pointlist[i].x,self.pointlist[i].y),
												(self.pointlist[j].x,self.pointlist[j].y))
			
			self.edgelist.append(edge)
			
	def update(self):
		pass
	
	def calculateCentroid(self,pointlist):
		
		centroid_x = 0
		centroid_y = 0
		signed_area = 0
		for i in range(len(pointlist)-1):
			
			x0 = float(pointlist[i][0])
			y0 = float(pointlist[i][1])
			x1 = float(pointlist[i+1][0])
			y1 = float(pointlist[i+1][1])
			
			a = x0*y1 - x1*y0
			signed_area += a
			centroid_x += (x0 + x1) * a
			centroid_y += (y0 + y1) * a
		
		x0 = float(pointlist[-1][0])
		y0 = float(pointlist[-1][1])
		x1 = float(pointlist[0][0])
		y1 = float(pointlist[0][1])
			
		a = x0*y1 - x1*y0
		signed_area += a
		centroid_x += (x0 + x1) * a
		centroid_y += (y0 + y1) * a
		
		signed_area *= 0.5
		centroid_x /= (6.0 * signed_area)
		centroid_y /= (6.0 * signed_area)
		
		return (centroid_x, centroid_y)
		
		
	def draw(self,Surface):
		
		if self.color is not None:
			for i in range(4):
				if i == len(self.pointlist)-1:
					j=0
				else:
					j=i+1
				
				pygame.draw.line(Surface,self.color,(self.pointlist[i].x,self.pointlist[i].y),
													(self.pointlist[j].x,self.pointlist[j].y))
				
		
class RectSAT(object):
	def __init__(self,param_list,x,y):
		"""Collision rect used for SAT collision detection"""
		# @params x,y are the topleft position of the box
		# param_list : [width,height,color]
		# width,height are the length and the breadth of the box
		# color can be an RGB tuple or None ; if None the rect is not drawn(useful in room editor)
		
		self.imageloader_func = param_list		# imageloader_func is an alias for param_list used by room data loaders
		
		self.collision_type = "poly"
		self.type = "static"
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		self.xw = self.w / 2
		self.yw = self.h / 2
		
		self.rect = pygame.Rect(self.x, self.y, self.w, self.h)
		
		self.center=Vector2.Vector(self.x + self.xw, self.y + self.yw)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.x, self.y)	# this is the registration point of the entire shape
		self.p1 = Vector2.Vector(self.position.x + self.w ,self.position.y)
		self.p2 = Vector2.Vector(self.p1.x, self.p1.y + self.h)
		self.p3 = Vector2.Vector(self.p2.x - self.w, self.p2.y)
		self.pointlist = []
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		self.pointlist.append(self.p3)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.p3.x,self.p3.y))
		self.egde4=Vector2.Vector.from_points((self.p3.x,self.p3.y),(self.position.x,self.position.y))
		
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		self.edgelist.append(self.egde4)
		
		# create image
		self.image = pygame.Surface((self.rect.width+1,self.rect.height+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		# resolve points to origin
		draw_points_list = []
		
		for item in self.pointlist:
			draw_x = item.x - self.x
			draw_y = item.y - self.y
			draw_vector = Vector2.Vector(draw_x,draw_y)
			draw_points_list.append(draw_vector)
		
		if self.color != None:
			for i in range(4):
				if i==3:
					j=0
				else:
					j=i+1
				pygame.draw.line(self.image,self.color,(draw_points_list[i].x,draw_points_list[i].y),(draw_points_list[j].x,draw_points_list[j].y))
				i+=1
		
	def draw(self,Surface):
		
		if self.color != None:
			Surface.blit(self.image,self.rect)
		
	def update(self):
		
		self.rect.topleft = (self.x,self.y)
		
		self.center = Vector2.Vector(self.x + self.xw, self.y + self.yw)
		# points of shapes represented as vectors
		
		self.position = Vector2.Vector(self.x, self.y)	# this is the registration point of the entire shape
		self.p1 = Vector2.Vector(self.position.x + self.w ,self.position.y)
		self.p2 = Vector2.Vector(self.p1.x, self.p1.y + self.h)
		self.p3 = Vector2.Vector(self.p2.x - self.w, self.p2.y)
		self.pointlist = []
		self.pointlist.append(self.position)
		self.pointlist.append(self.p1)
		self.pointlist.append(self.p2)
		self.pointlist.append(self.p3)
		
		# create edges
		self.egde1=Vector2.Vector.from_points((self.position.x,self.position.y),(self.p1.x,self.p1.y))
		self.egde2=Vector2.Vector.from_points((self.p1.x,self.p1.y),(self.p2.x,self.p2.y))
		self.egde3=Vector2.Vector.from_points((self.p2.x,self.p2.y),(self.p3.x,self.p3.y))
		self.egde4=Vector2.Vector.from_points((self.p3.x,self.p3.y),(self.position.x,self.position.y))
		
		self.edgelist=[]
		self.edgelist.append(self.egde1)
		self.edgelist.append(self.egde2)
		self.edgelist.append(self.egde3)
		self.edgelist.append(self.egde4)
