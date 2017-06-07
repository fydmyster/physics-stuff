import sys,pygame,math
from pygame.locals import*

################ 
# Generic collision objects
class Line(object):
	def __init__(self,p1,p2,color=(0,0,0)):
		
		self.p1=p1
		self.p2=p2
		self.p1x=float(self.p1[0])
		self.p1y=float(self.p1[1])
		self.p2x=float(self.p2[0])
		self.p2y=float(self.p2[1])
		
		self.xvec=float(self.p2x-self.p1x)
		self.yvec=float(self.p2y-self.p1y)
		self.length=int(math.sqrt((self.xvec*self.xvec)+(self.yvec*self.yvec)))
		
		if self.xvec==0:
			# prevent zero division error hack
			self.m=self.yvec/(self.xvec-0.01)
		else:
			self.m=self.yvec/self.xvec
		
		self.b=self.p2y-(self.m * self.p2x)
		
		#COLOR
		self.color=color
		
	def draw(self,Surface):
		pygame.draw.line(Surface,self.color,self.p1,self.p2,1)
		
	def update(self,start,end):
		"""Updates the line by passing in updated start and end points for line"""
	
		self.p1=start
		self.p2=end
		self.p1x=self.p1[0]
		self.p1y=self.p1[1]
		self.p2x=self.p2[0]
		self.p2y=self.p2[1]
		
		self.xvec=float(self.p2x-self.p1x)
		self.yvec=float(self.p2y-self.p1y)
		self.length=int(math.sqrt((self.xvec*self.xvec)+(self.yvec*self.yvec)))
		
		if self.xvec==0:
			# prevent zero division error hack
			self.m=self.yvec/(self.xvec-0.01)
		else:
			self.m=self.yvec/self.xvec
		
		self.b=self.p2y-(self.m * self.p2x)
		
		
	def __str__(self):		
		return "LENGTH IS %s ,and slope is %s  intercept is, %s" % (self.length,self.m,self.b)
	
	def collideLine(self,otherline):
		segment_self=False
		segment_other=False
		
		if (otherline.m-self.m) == 0:
			# lines are parallel
			return (False,(0,0))
		else:
			xPoint=(self.b-otherline.b)/(otherline.m-self.m)
			
		yPoint=(self.m*xPoint)+self.b
		
		# check if segments themselves intersect
		if (xPoint >=self.p1x and  xPoint<=self.p2x) or \
		(xPoint <= self.p1x and xPoint >= self.p2x) or \
		(yPoint >= self.p1y and yPoint <= self.p2y) or \
		(yPoint <= self.p1y and yPoint >= self.p2y):
			segment_self = True
				
		if (xPoint >= otherline.p1x and xPoint <= otherline.p2x) or \
		(xPoint <= otherline.p1x and xPoint >= otherline.p2x) or \
		(yPoint >= otherline.p1y and yPoint <= otherline.p2y) or \
		(yPoint <= otherline.p1y and yPoint >= otherline.p2y):
			segment_other = True
		
		if segment_self and segment_other:
			return (True,(xPoint,yPoint))
		else:
			return (False,(xPoint,yPoint))
	
def collideLine(p1,p2,p3,p4):
	
	segment_self = False
	segment_other = False
		
	if (otherline.m-self.m) == 0:
		# lines are parallel
		return (False,(0,0))
	else:
		xPoint=(self.b-otherline.b)/(otherline.m-self.m)
			
	yPoint=(self.m*xPoint)+self.b
		
	# check if segments themselves intersect
	if (xPoint >=self.p1x and  xPoint<=self.p2x) or \
	(xPoint <= self.p1x and xPoint >= self.p2x) or \
	(yPoint >= self.p1y and yPoint <= self.p2y) or \
	(yPoint <= self.p1y and yPoint >= self.p2y):
		segment_self = True
				
	if (xPoint >= otherline.p1x and xPoint <= otherline.p2x) or \
	(xPoint <= otherline.p1x and xPoint >= otherline.p2x) or \
	(yPoint >= otherline.p1y and yPoint <= otherline.p2y) or \
	(yPoint <= otherline.p1y and yPoint >= otherline.p2y):
		segment_other = True
		
	if segment_self and segment_other:
		return (True,(xPoint,yPoint))
	else:
		return (False,(xPoint,yPoint))
	

def calculateGradient(p1,p2):
	# ensure that line is not vertical
	if p1[0] != p2[0]:
		m = (p1[1] - p2[1])/(p1[0] - p2[0])
		return m
			
	else:
		return None
		
def calculateYAxisIntercept(p,m):
	return p[1] - (m * p[0])
	
def getIntersectPoint(p1,p2,p3,p4):
	"""Calculates the point where two infinitely long lines intersect
	Used mainly as a helper function for calculateIntersectPoint"""
	m1 = calculateGradient(p1,p2)
	m2 = calculateGradient(p3,p4)
		
	# see if lines are parallel
	if m1 != m2:
		# not parallel
			
		# see if either line vertical
		if m1 is not None and m2 is not None:
			# neither is vertical
			b1 = calculateYAxisIntercept(p1,m1)
			b2 = calculateYAxisIntercept(p3,m2)
				
			x = (b2-b1)/(m1-m2)
			y = (m1 * x) + b1
			
		else:
			# line 1 is vertical so use line 2's values
			if m1 is None:
				b2 = calculateYAxisIntercept(p3,m2)
					
				x = p1[0]
				y = (m2 * x)+b2
					
			# line 2 is vertical so use line 1's values
			elif m2 is None:
				b1 = calculateYAxisIntercept(p1,m1)
					
				x = p3[0]
				y = (m1 * x) + b1
					
			else: 
				assert False
		return ((x,y),)
		
	else:
		# parallel lines with same b value
			
		b1,b2 =None,None # vertical lines have no b value
			
		if m1 is not None:
			b1 = calculateYAxisIntercept(p1,m1)
				
		if m2 is not None:
			b2 = calculateYAxisIntercept(p3,m2)
				
		# if these parallel lines lay on one another
		if b1 == b2:
			return p1,p2,p3,p4
		else:
			return None
				
def calculateIntersectPoint(p1,p2,p3,p4):
	"""Tests if two lines are colliding based on the points passed in
	Returns None if there is no intersection"""
	
	# always pass in floats or things get sketchy
	
	p = getIntersectPoint(p1,p2,p3,p4)
		
	if p is not None:
		width = p2[0] - p1[0]
		height = p2[1] -p1[1]
		r1 = pygame.Rect(p1,(width,height))
		r1.normalize()
			
		width = p4[0] - p3[0]
		height = p4[1] -p3[1]
		r2 = pygame.Rect(p3,(width,height))
		r2.normalize()
		
		tolerance=1
		
		if r1.width < tolerance:
			r1.width = tolerance
	
		if r1.height < tolerance:
			r1.height = tolerance
	
		if r2.width < tolerance:
			r2.width = tolerance
		
		if r2.height < tolerance:
			r2.height = tolerance
			
		for point in p:
			try:
				res1 = r1.collidepoint(point)
				res2 = r2.collidepoint(point)
				if res1 and res2:
					point = [int(pp) for pp in point]
					
					return point
			except:
				str = "point was in invalid"
				print str
				
		return None
	else:
		return None
		
	
########
# Some collision helper objects
# feel free to use them in the room editor

# param_list :[w,h,color]		
class BoxTile(object):
	def __init__(self,param_list,x=0,y=0):
	
		self.imageloader_func = param_list # this is an alias;  every room editor compatible object must own this attribute
		
		self.x = x
		self.y = y
		self.w = param_list[0]
		self.h = param_list[1]
		self.color = param_list[2]
		
		self.rect = pygame.Rect(self.x,self.y,self.w,self.h)
		self.true_rect = pygame.Rect(self.x-1,self.y-1,self.w+1,self.h+1)
		
		self.image = pygame.Surface((self.rect.width,self.rect.height))
		self.image.fill(self.color)
		
	def draw(self,Surface):
		Surface.blit(self.image,self.rect)
		
	def updatePosition(self,timepassed):
		self.rect.topleft = (self.x,self.y)
	
	def update(self,timepassed):
		self.updatePosition(timepassed)
						
# param_list :[size,color]
class DEG180Line(object):
	"""Collision line facing up and to the left"""
	# Any object that interacts with this class requires the following attributes:
	# x, y, xvel, yvel
	def __init__(self,param_list,x=0,y=0):
		
		self.imageloader_func = param_list # this is an alias;  every room editor compatible object must own this attribute
		
		self.x = float(x)
		self.y = float(y)
		self.size = param_list[0]
		self.color = param_list[1]
		self.type = "NULL"
		
		self.rect = pygame.Rect(self.x,self.y,self.size,5)
		
		self.p0 = float(self.rect.topleft[0]),float(self.rect.topleft[1])
		self.p1 = float(self.rect.topright[0]),float(self.rect.topright[1])
		
		self.image = pygame.Surface((self.rect.w+1,self.rect.h+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		p0draw_x = self.p0[0] - self.x
		p0draw_y = self.p0[1] - self.y
		p1draw_x = self.p1[0] - self.x
		p1draw_y = self.p1[1] - self.y
		
		pygame.draw.line(self.image,self.color,(p0draw_x,p0draw_y),(p1draw_x,p1draw_y))
		
	def updatePosition(self,timepassed):
		
		self.rect.topleft = (self.x,self.y)
		self.p0 = float(self.rect.topleft[0]),float(self.rect.topleft[1])
		self.p1 = float(self.rect.topright[0]),float(self.rect.topright[1])
		
	def update(self,timepassed):
		
		self.updatePosition(timepassed)
		
	def draw(self,Surface):
		
		Surface.blit(self.image,self.rect)
	
		
# param_list :[size,color]
class DEG45LineLU(object):
	"""Collision line facing up and to the left"""
	# Any object that interacts with this class requires the following attributes:
	# x, y, xvel, yvel
	def __init__(self,param_list,x=0,y=0):
		
		self.imageloader_func = param_list # this is an alias;  every room editor compatible object must own this attribute
		
		self.x = float(x)
		self.y = float(y)
		self.size = param_list[0]
		self.color = param_list[1]
		self.type = "LU"
		
		self.rect = pygame.Rect(self.x,self.y,self.size,self.size)
		
		self.p0 = float(self.rect.bottomleft[0]),float(self.rect.bottomleft[1])
		self.p1 = float(self.rect.topright[0]),float(self.rect.topright[1])
		
		self.image = pygame.Surface((self.rect.w+1,self.rect.h+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		p0draw_x = self.p0[0] - self.x
		p0draw_y = self.p0[1] - self.y
		p1draw_x = self.p1[0] - self.x
		p1draw_y = self.p1[1] - self.y
		
		pygame.draw.line(self.image,self.color,(p0draw_x,p0draw_y),(p1draw_x,p1draw_y))
			
	def updatePosition(self,timepassed):
		
		self.rect.topleft = (self.x,self.y)
		self.p0 = float(self.rect.bottomleft[0]),float(self.rect.bottomleft[1])
		self.p1 = float(self.rect.topright[0]),float(self.rect.topright[1])
		
	def update(self,timepassed):
		
		self.updatePosition(timepassed)
		
	def draw(self,Surface):
		
		Surface.blit(self.image,self.rect)
	
# param_list :[size,color]
class DEG45LineRU(object):
	"""Collision line facing up and to the left"""
	# Any object that interacts with this class requires the following attributes:
	# x, y, xvel, yvel
	def __init__(self,param_list,x=0,y=0):
		
		self.imageloader_func = param_list # this is an alias;  every room editor compatible object must own this attribute
		
		self.x = float(x)
		self.y = float(y)
		self.size = param_list[0]
		self.color = param_list[1]
		self.type = "RU"
		
		self.rect = pygame.Rect(self.x,self.y,self.size,self.size)
		
		self.p0 = float(self.rect.topleft[0]),float(self.rect.topleft[1])
		self.p1 = float(self.rect.bottomright[0]),float(self.rect.bottomright[1])
		
		self.image = pygame.Surface((self.rect.w+1,self.rect.h+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		p0draw_x = self.p0[0] - self.x
		p0draw_y = self.p0[1] - self.y
		p1draw_x = self.p1[0] - self.x
		p1draw_y = self.p1[1] - self.y
		
		pygame.draw.line(self.image,self.color,(p0draw_x,p0draw_y),(p1draw_x,p1draw_y))
		
	def updatePosition(self,timepassed):
		
		self.rect.topleft = (self.x,self.y)
		self.p0 = float(self.rect.topleft[0]),float(self.rect.topleft[1])
		self.p1 = float(self.rect.bottomright[0]),float(self.rect.bottomright[1])
		
	def update(self,timepassed):
		
		self.updatePosition(timepassed)
		
	def draw(self,Surface):
		
		Surface.blit(self.image,self.rect)
	
class DEG22LineLU(object):
	"""Collision line facing up and to the left"""
	# Any object that interacts with this class requires the following attributes:
	# x, y, xvel, yvel
	def __init__(self,param_list,x=0,y=0):
		
		self.imageloader_func = param_list # this is an alias;  every room editor compatible object must own this attribute
		
		self.x = float(x)
		self.y = float(y)
		self.size = param_list[0]
		self.color = param_list[1]
		self.type = "LU"
		
		self.rect = pygame.Rect(self.x,self.y,self.size,self.size)
		
		self.p0 = float(self.rect.bottomleft[0]),float(self.rect.bottomleft[1])
		self.p1 = float(self.rect.midright[0]),float(self.rect.midright[1])
		
		self.image = pygame.Surface((self.rect.w+1,self.rect.h+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		p0draw_x = self.p0[0] - self.x
		p0draw_y = self.p0[1] - self.y
		p1draw_x = self.p1[0] - self.x
		p1draw_y = self.p1[1] - self.y
		
		pygame.draw.line(self.image,self.color,(p0draw_x,p0draw_y),(p1draw_x,p1draw_y))
		
	def updatePosition(self,timepassed):
		
		self.rect.topleft = (self.x,self.y)
		self.p0 = float(self.rect.bottomleft[0]),float(self.rect.bottomleft[1])
		self.p1 = float(self.rect.midright[0]),float(self.rect.midright[1])
		
	def update(self,timepassed):
		
		self.updatePosition(timepassed)
		
	def draw(self,Surface):
		
		Surface.blit(self.image,self.rect)
	
# param_list :[size,color]
class DEG22LineRU(object):
	"""Collision line facing up and to the left"""
	# Any object that interacts with this class requires the following attributes:
	# x, y, xvel, yvel
	def __init__(self,param_list,x=0,y=0):
		
		self.imageloader_func = param_list # this is an alias;  every room editor compatible object must own this attribute
		
		self.x = float(x)
		self.y = float(y)
		self.size = param_list[0]
		self.color = param_list[1]
		self.type = "RU"
		
		self.rect = pygame.Rect(self.x,self.y,self.size,self.size)
		
		self.p0 = float(self.rect.midleft[0]),float(self.rect.midleft[1])
		self.p1 = float(self.rect.bottomright[0]),float(self.rect.bottomright[1])
		self.image = pygame.Surface((self.rect.w+1,self.rect.h+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		p0draw_x = self.p0[0] - self.x
		p0draw_y = self.p0[1] - self.y
		p1draw_x = self.p1[0] - self.x
		p1draw_y = self.p1[1] - self.y
		
		pygame.draw.line(self.image,self.color,(p0draw_x,p0draw_y),(p1draw_x,p1draw_y))
		
	def updatePosition(self,timepassed):
		
		self.rect.topleft = (self.x,self.y)
		self.p0 = float(self.rect.midleft[0]),float(self.rect.midleft[1])
		self.p1 = float(self.rect.bottomright[0]),float(self.rect.bottomright[1])
		
	def update(self,timepassed):
		
		self.updatePosition(timepassed)
		
	def draw(self,Surface):
		
		Surface.blit(self.image,self.rect)
	
class DEG67LineLU(object):
	"""Collision line facing up and to the left"""
	# Any object that interacts with this class requires the following attributes:
	# x, y, xvel, yvel
	def __init__(self,param_list,x=0,y=0):
		
		self.imageloader_func = param_list # this is an alias;  every room editor compatible object must own this attribute
		
		self.x = float(x)
		self.y = float(y)
		self.size = param_list[0]
		self.color = param_list[1]
		self.type = "LU"
		
		self.rect = pygame.Rect(self.x,self.y,self.size,self.size)
		
		self.p0 = float(self.rect.bottomleft[0]),float(self.rect.bottomleft[1])
		self.p1 = float(self.rect.midtop[0]),float(self.rect.midtop[1])
		
		self.image = pygame.Surface((self.rect.w+1,self.rect.h+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		p0draw_x = self.p0[0] - self.x
		p0draw_y = self.p0[1] - self.y
		p1draw_x = self.p1[0] - self.x
		p1draw_y = self.p1[1] - self.y
		
		pygame.draw.line(self.image,self.color,(p0draw_x,p0draw_y),(p1draw_x,p1draw_y))
		
	def updatePosition(self,timepassed):
		
		self.rect.topleft = (self.x,self.y)
		self.p0 = float(self.rect.bottomleft[0]),float(self.rect.bottomleft[1])
		self.p1 = float(self.rect.midtop[0]),float(self.rect.midtop[1])
		
	def update(self,timepassed):
		
		self.updatePosition(timepassed)
		
	def draw(self,Surface):
		
		Surface.blit(self.image,self.rect)
	
# param_list :[size,color]
class DEG67LineRU(object):
	"""Collision line facing up and to the left"""
	# Any object that interacts with this class requires the following attributes:
	# x, y, xvel, yvel
	def __init__(self,param_list,x=0,y=0):
		
		self.imageloader_func = param_list # this is an alias;  every room editor compatible object must own this attribute
		
		self.x = float(x)
		self.y = float(y)
		self.size = param_list[0]
		self.color = param_list[1]
		self.type = "RU"
		
		self.rect = pygame.Rect(self.x,self.y,self.size,self.size)
		
		self.p0 = float(self.rect.topleft[0]),float(self.rect.topleft[1])
		self.p1 = float(self.rect.midbottom[0]),float(self.rect.midbottom[1])
		
		self.image = pygame.Surface((self.rect.w+1,self.rect.h+1))
		self.image.set_colorkey((255,0,255))
		self.image.fill((255,0,255))
		
		p0draw_x = self.p0[0] - self.x
		p0draw_y = self.p0[1] - self.y
		p1draw_x = self.p1[0] - self.x
		p1draw_y = self.p1[1] - self.y
		
		pygame.draw.line(self.image,self.color,(p0draw_x,p0draw_y),(p1draw_x,p1draw_y))
		
	def updatePosition(self,timepassed):
		
		self.rect.topleft = (self.x,self.y)
		self.p0 = float(self.rect.topleft[0]),float(self.rect.topleft[1])
		self.p1 = float(self.rect.midbottom[0]),float(self.rect.midbottom[1])
		
	def update(self,timepassed):
		
		self.updatePosition(timepassed)
		
	def draw(self,Surface):
		
		Surface.blit(self.image,self.rect)
	
######
# Circle/Line collision detection and resolution functions

def CircleLineCollide(circlepos,circleradius,linepos1,linepos2):
	# calculate line vector
	linevx= linepos2[0] - linepos1[0]
	linevy= linepos2[1] - linepos1[1]

	# calculate the left hand normal
	lhnx = linevy
	lhny = -linevx
	
	# calculate vector from line start point to circle  center
	lineballvecx = circlepos[0] - linepos1[0]
	lineballvecy = circlepos[1] - linepos1[1]
	
	# calculate perpendicular length by projection on left normal
	# normalise the lhn
	len = math.sqrt( lhnx*lhnx + lhny*lhny )
	lhnx/=len
	lhny/=len
	
	# project lineball vector onto left normal
	lnDotbline = lhnx * lineballvecx + lhny * lineballvecy
	
	if abs(lnDotbline) <= circleradius:		# without the abs check we get inconsistent results
		 
		# check that we're within line segments bounds
		# calculate delta vector from line start point to circle  center
		lineballdeltax = circlepos[0] - linepos1[0]
		lineballdeltay = circlepos[1] - linepos1[1]
		
		
		# get length (magnitude) of line vector 
		linelength = math.sqrt(linevx * linevx + linevy * linevy ) 
		
		# normalise line vector
		linevx /= linelength
		linevy /= linelength
		
		# project linepos1-circle center vector onto the line vector
		# check dot product between line vector and linepos1-circle center vector
		deltaDotLine = lineballdeltax * linevx + lineballdeltay * linevy
		
		# check if the projections length is greater than lineseg length
		if deltaDotLine > 0 and deltaDotLine < linelength:
			#print "collision is happening resolve"
			return True
		else:
			return False
			
	return False

def resolveCircleLineReflect(circlexvel,circleyvel,linepos1,linepos2):
	"""Call this before calling resolveCircleLineReflect
	Alters the velocity of circle that collides with a line"""
	# redefine velocity
	# project velocity onto left normal
	
	linevx= linepos2[0] - linepos1[0]
	linevy= linepos2[1] - linepos1[1]

	# calculate the left hand normal
	lhnx = linevy
	lhny = -linevx
	
	# normalise the lhn
	len = math.sqrt( lhnx*lhnx + lhny*lhny )
	lhnx/=len
	lhny/=len
	
	# project velocities
	velDotLnormal = circlexvel * lhnx + circleyvel * lhny
	proj = abs(velDotLnormal)
	
	# set new magnitude for left normal
	lhnx *= velDotLnormal
	lhny *= velDotLnormal
	
	# the tut i followed says add but i have to subtract i i dont reverse the velocity
	circlexvel =(circlexvel - (lhnx *(2))) # the coefficient here refers to the bounce elasticity
	circleyvel =(circleyvel - (lhny *(2)))
	
	return (circlexvel,circleyvel)

def resolveCircleLineSlide(circlepos,circleradius,linepos1,linepos2):
	""" Call this before calling resolveCircleLineReflect
	Returns a repositioned point to set the circle after it collides with a line seg"""
	# reposition circle
	# get line vector
	linevx= linepos2[0] - linepos1[0]
	linevy= linepos2[1] - linepos1[1]
	
	# get length (magnitude) of line vector 
	linelength = math.sqrt(linevx * linevx + linevy * linevy ) 
		
	# normalise the line vector
	linevx /= linelength
	linevy /= linelength
	
	# project lineballdelta onto the line
	
	# calculate delta vector from line start point to circle  center
	lineballdeltax = circlepos[0] - linepos1[0]
	lineballdeltay = circlepos[1] - linepos1[1]
	
	# project linepos1-circle center vector onto the line vector
	# check dot product between line vector and linepos1-circle center vector
	deltaDotLine = (lineballdeltax * linevx + lineballdeltay * linevy)
	
	# vectorB is the circle line vector projected onto the line
	vectorBX = linevx * deltaDotLine
	vectorBY = linevy * deltaDotLine
	
	# determine which normal to use depending on if we're above or below the line
	# calculate the dot product between lineballdelta and the rhn
	# if its positive use the rhn for repositioning ;else use lhn
	
	rhnx = -linevy
	rhny = linevx
	
	deltaDotRhn = lineballdeltax * rhnx + lineballdeltay * rhny
	
	if deltaDotRhn>=0:
		# we are under the line ; use rhn for repositioning
		repoNormalX=rhnx
		repoNormalY=rhny
	else:
		# we are above ; use lhn for repositioning
		repoNormalX=linevy
		repoNormalY=-linevx
		
	# calculate the left hand normal
	lhnx = linevy
	lhny = -linevx
	
	# normalise the lhn
	len = math.sqrt( repoNormalX*repoNormalX + repoNormalY*repoNormalY )
	repoNormalX/=len
	repoNormalY/=len
	
	
	# vector with length of circle radius(or radius - 1 if its jittery)
	repoNormalX *= (circleradius)
	repoNormalY *= (circleradius)
	
	# vector a is the vector from linepos1 to the circle center
	ax = vectorBX +	repoNormalX
	ay = vectorBY + repoNormalY
		
	# add the normal onto the line vector
	repositionX = linepos1[0] + ax
	repositionY = linepos1[1] + ay
	
	# return the new circle position
	return (repositionX,repositionY)
