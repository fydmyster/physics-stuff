import math

# def vector class
class Vector(object):
	def __init__(self,x=0.0,y=0.0):
		self.x=x
		self.y=y
		
	def __str__(self):
		return "(%s,%s)" %(self.x,self.y)
		
	@classmethod
	def from_points(cls,P1,P2):
		return cls(P2[0]-P1[0],P2[1]-P1[1])
		
	def get_magnitude(self):
		return math.sqrt((self.x * self.x)+(self.y * self.y))
		
	def normalize(self):
		magnitude=self.get_magnitude()
		if magnitude>0:
			self.x /= magnitude
			self.y /= magnitude
				
	def to_point(self):
		return(self.x,self.y)
		
	def __add__(self,rhs):
		return Vector(self.x+rhs.x,self.y+rhs.y)
		
	def __sub__(self,rhs):
		return Vector(self.x-rhs.x,self.y-rhs.y)
		
	def __neg__(self):
		return Vector(-self.x,-self.y)
		
	def __mul__(self,scalar):
		return Vector(self.x * scalar,self.y * scalar)
		
	def __div__(self,scalar):
		return Vector(self.x / scalar,self.y /scalar)
		
	def dotProduct(self,otherVector):
		thisDotOther=(self.x*otherVector.x)+(self.y * otherVector.y)
		return thisDotOther
	
