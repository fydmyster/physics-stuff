import math,pygame,sat,gen_collision,Vector2

def collidePolyPoly(bodyA,bodyB):
	"""Handles a collision between 2 polygonal bodies"""
	
	if bodyA == bodyB:
		return
	
	
	shapeA = bodyA.shape.shape_collider
	shapeA_lines = bodyA.shape.line_colliders
	shapeA_system = bodyA.system
	
	shapeB = bodyB.shape.shape_collider
	shapeB_lines = bodyB.shape.line_colliders
	shapeB_system = bodyB.system
	
	if shapeA_system.mass > 100000 and shapeB_system.mass > 100000:
		return
	
	result = sat.collidePolygon(shapeA, shapeB)
	
			
	if result[0]:
		
		bodyA.system.num_collisions += 1
		bodyB.system.num_collisions += 1
		
		delta_mass = shapeA_system.mass + shapeB_system.mass 
		m_A = shapeB_system.mass / float(delta_mass)
		m_B = shapeA_system.mass / float(delta_mass)
		
		overlap = (abs(result[1])) * 0.999 	# hacky var here: 0.8 seems more stable than 0.95
		
		dir = (result[2])                           #is the penetration direction
		dir_other = Vector2.Vector(result[2].x,result[2].y)
		
		d = shapeA.center - shapeB.center
		d_other = shapeB.center - shapeA.center
		
		if d.dotProduct(dir) < 0:
			dir = -dir
	
		if d_other.dotProduct(dir_other) < 0:
			dir_other = -dir_other
	
		
		dir *= overlap
		dir_other *= overlap
		
		for line_collider,index in shapeA_lines:
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
		for line,index in shapeA_lines:
		
			c_vec = (line.p1 - line.pos)
			mag = c_vec.get_magnitude()
			c_vec.normalize()
			true_centroid = line.pos + (c_vec * (mag/2.0))
			
			# dist from poly center
			d_vec = true_centroid - shapeB.center
			dist = d_vec.get_magnitude()
			
			#if dist < smallest_dist:
			#	smallest_dist = dist
				#line_to_test = (line,index)
			
			lines_dict[dist] = (line,index)
			#epsilon += 0.01
		
		d_keys = lines_dict.keys()
		d_keys.sort()
	
		del shapeA_lines[:]
		for key in d_keys:
			line = lines_dict[key]
			shapeA_lines.append(line)
	
		
		ret = False
		offending_index = None
		offending_collider = None
		offending_collider_other = None
		offending_index_other = None
		
		for line_collider,index in shapeA_lines:
			
			if ret:
				break
			
			for line,ind in shapeB_lines:
		
				stick = gen_collision.Line(((line_collider.pos.x),(line_collider.pos.y)),
											((line_collider.p1.x),(line_collider.p1.y)))
				line_seg = gen_collision.Line(((line.pos.x),(line.pos.y)),
											((line.p1.x),(line.p1.y)))
			
					
				res = stick.collideLine(line_seg)
				if res[0]:
					#print "this is happening"
					dcp = res[1]
				
					
				scp = dcp
			
				if dcp is not None:
					rx = dcp[0] - (dir.x )
					ry = dcp[1] - (dir.y )
					dcp = [rx,ry]
					offending_index = index
					offending_collider = line_collider
					
					ret = True
					break
		
		
		for line_collider,index in shapeA_lines:
			line_collider.x -= dir.x
			line_collider.y -= dir.y
			line_collider.update()
		
		
		
		for line_collider,index in shapeB_lines:
			line_collider.x += dir_other.x
			line_collider.y += dir_other.y
			line_collider.update()
		
	
		######################################################################
		# sort the lines
		######################################################################
		
		lines_dict = {}
		smallest_dist = 1000
		#line_to_test = None
		
		#epsilon = 0.01
		for line,index in shapeB_lines:
			c_vec = (line.p1 - line.pos)
			mag = c_vec.get_magnitude()
			c_vec.normalize()
			true_centroid = line.pos + (c_vec * (mag/2.0))
			
			# dist from poly center
			d_vec = true_centroid - shapeA.center
			dist = d_vec.get_magnitude()
			
			lines_dict[dist] = (line,index)
				
		d_keys = lines_dict.keys()
		d_keys.sort()
	
		del shapeB_lines[:]
			
		for key in d_keys:
			line = lines_dict[key]
			shapeB_lines.append(line)
	
		
		
		ret = False
		offending_collider_other = None
		offending_index_other = None
		
		for line_collider,index in shapeB_lines:
				
			if ret:
				break
				
			for line,ind in shapeA_lines:
			
				stick = gen_collision.Line(((line_collider.pos.x),(line_collider.pos.y)),
											((line_collider.p1.x),(line_collider.p1.y)))
				line_seg = gen_collision.Line(((line.pos.x),(line.pos.y)),
											((line.p1.x),(line.p1.y)))
				
						
				res = stick.collideLine(line_seg)
				if res[0]:
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
			
		
		for line_collider,index in shapeB_lines:
			line_collider.x -= dir_other.x
			line_collider.y -= dir_other.y
			line_collider.update()
		
		
		
		if scp is not None:
			
			d_vec = offending_collider.p1 - offending_collider.pos 
			d_mag = d_vec.get_magnitude()
			
			p_vec = Vector2.Vector(scp[0],scp[1])
			p_vec = p_vec - offending_collider.pos
			p_mag = p_vec.get_magnitude()
			
			ratio = p_mag/float(d_mag)
			
			c2 = ratio				# I swapped these around
			c1 = 1 - ratio			# I swapped these around
			scp_v = Vector2.Vector(scp[0],scp[1])
			dcp_v = Vector2.Vector(dcp[0],dcp[1])
			
			shapeA_system.friction_ratios[offending_index[0]] = c1
			shapeA_system.friction_ratios[offending_index[1]] = c2
			
			dis_vec = scp_v - dcp_v
			dis_vec_n = scp_v - dcp_v
			dis_vec_n.normalize()
			
			numerator = dis_vec.dotProduct(dis_vec_n) 
			denom =  dis_vec_n.dotProduct(dis_vec_n) * (c1 * c1 + c2 * c2)		# used a normal here 
			
			# prevent division by 0
			if denom == 0:
				denom = 0.000001
			
			mys = numerator/float(denom)
			
			x1 = offending_collider.pointlist[0] + ((dis_vec_n *(c1 * mys * m_A)))		# used a normal here too 
			x2 = offending_collider.pointlist[1] + ((dis_vec_n *(c2 * mys * m_A))) 
			
			
			shapeA_system.currentParticlePos[offending_index[0]].x = x1.x
			shapeA_system.currentParticlePos[offending_index[0]].y = x1.y
			
			shapeA_system.currentParticlePos[offending_index[1]].x = x2.x
			shapeA_system.currentParticlePos[offending_index[1]].y = x2.y
			
			
		if  scp_other is not None :
			
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
			
			shapeB_system.friction_ratios[offending_index_other[0]] = c1
			shapeB_system.friction_ratios[offending_index_other[1]] = c2
			
			dis_vec = scp_v - dcp_v
			dis_vec_n = scp_v - dcp_v
			dis_vec_n.normalize()
			
			
			numerator = dis_vec.dotProduct(dis_vec_n) 
			denom =  dis_vec_n.dotProduct(dis_vec_n) * (c1 * c1 + c2 * c2)		# used a normal here 
			
			# prevent division by 0
			if denom == 0:
				denom = 0.000001
			
			mys = numerator/float(denom)
			
			x1 = offending_collider_other.pointlist[0] + ((dis_vec_n *(c1 * mys * m_B)))		# used a normal here too 
			x2 = offending_collider_other.pointlist[1] + ((dis_vec_n *(c2 * mys * m_B))) 
			
			
			shapeB_system.currentParticlePos[offending_index_other[0]].x = x1.x
			shapeB_system.currentParticlePos[offending_index_other[0]].y = x1.y
			
			shapeB_system.currentParticlePos[offending_index_other[1]].x = x2.x
			shapeB_system.currentParticlePos[offending_index_other[1]].y = x2.y

def collidePolyCircle(poly, circle):
	"""Handles a collision between a polygonal body and circle"""
	
	shapeA = poly.shape.shape_collider
	shapeA_lines = poly.shape.line_colliders
	shapeA_system = poly.system
	
	shapeB = circle.shape.shape_collider
	#shapeB_lines = circle.shape.line_colliders
	shapeB_system = circle.system
	
	if shapeA_system.mass > 100000 and shapeB_system.mass > 100000:
		return
	
	result = sat.collidePolyCircle(shapeB, shapeA)
	
			
	if result[0]:
		
		circle.system.friction_ratios[0] = 1.0
		poly.system.num_collisions += 1
		circle.system.num_collisions += 1
		
		delta_mass = shapeA_system.mass + shapeB_system.mass 
		
		if delta_mass == 0:
			m_A = 0
			m_B = 0
		else:
			m_A = shapeB_system.mass / float(delta_mass)
			m_B = shapeA_system.mass / float(delta_mass)
			
			# i swapped the masses around
			
		overlap = (abs(result[1])) 
		overlap2 = (abs(result[1])) * 0.9999 	# hacky var here: 0.8 seems more stable than 0.95
		
		
		dir = (result[2])                           #is the penetration direction
		dir2 = Vector2.Vector(dir.x,dir.y)
		
		d = shapeA.center - shapeB.center
		
		if d.dotProduct(dir) < 0:
			dir = -dir
			dir2 = -dir2
			
		dir *= (overlap) 
		dir2 *= overlap2
		
		for line_collider,index in shapeA_lines:
			line_collider.x += dir2.x
			line_collider.y += dir2.y
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
		
		for line,index in shapeA_lines:
		
			c_vec = (line.p1 - line.pos)
			mag = c_vec.get_magnitude()
			c_vec.normalize()
			true_centroid = line.pos + (c_vec * (mag/2.0))
			
			# dist from poly center
			d_vec = true_centroid - shapeB.center
			dist = d_vec.get_magnitude()
			
			#if dist < smallest_dist:
			#	smallest_dist = dist
				#line_to_test = (line,index)
			
			lines_dict[dist] = (line,index)
			#epsilon += 0.01
		
		d_keys = lines_dict.keys()
		d_keys.sort()
	
		del shapeA_lines[:]
		for key in d_keys:
			line = lines_dict[key]
			shapeA_lines.append(line)
	
		
		offending_index = None
		offending_collider = None
		
		for line_collider,index in shapeA_lines:
			
			pts = sat.lineCircleClip(shapeB.center.x,shapeB.center.y,shapeB.radius,
												line_collider.pos,line_collider.p1)
				
			
			if pts[0] is None and pts[1] is None:
				continue
			
			if pts[0] is not None:
				dcp = pts[1]
			
				
			scp = dcp
		
			if dcp is not None:
				rx = dcp[0] - (dir2.x)
				ry = dcp[1] - (dir2.y)
				dcp = [rx,ry]
				offending_index = index
				offending_collider = line_collider
				break
		
		for line_collider,index in shapeA_lines:
			line_collider.x -= dir2.x
			line_collider.y -= dir2.y
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
			shapeA_system.friction_ratios[offending_index[0]] = c1
			shapeA_system.friction_ratios[offending_index[1]] = c2
			
			
			dis_vec = scp_v - dcp_v
			dis_vec_n = scp_v - dcp_v
			dis_vec_n.normalize()
			
			numerator = dis_vec.dotProduct(dis_vec_n) 
			denom =  dis_vec_n.dotProduct(dis_vec_n) * (c1 * c1 + c2 * c2)		# used a normal here 
			
			# prevent division by 0
			if denom == 0:
				denom = 0.000001
			
			mys = numerator/float(denom)
			
			
			x1 = offending_collider.pointlist[0] + ((dis_vec_n *(c1 * mys * m_A)))		# used a normal here too 
			x2 = offending_collider.pointlist[1] + ((dis_vec_n *(c2 * mys * m_A))) 
			
			shapeA_system.currentParticlePos[offending_index[0]].x = x1.x
			shapeA_system.currentParticlePos[offending_index[0]].y = x1.y
			
			shapeA_system.currentParticlePos[offending_index[1]].x = x2.x
			shapeA_system.currentParticlePos[offending_index[1]].y = x2.y
		
		
		shapeB.x -= (dir.x * m_B)
		shapeB.y -= (dir.y * m_B)
		shapeB.update()
		
		pos = shapeB.center
		shapeB_system.currentParticlePos[0].x = pos.x
		shapeB_system.currentParticlePos[0].y = pos.y
		
def collideCircleCircle(circleA, circleB):
	"""Handles a collision between a polygonal body and circle"""
	
	shapeA = circleA.shape.shape_collider
	shapeA_system = circleA.system
	
	shapeB = circleB.shape.shape_collider
	shapeB_system = circleB.system
	
	if shapeA_system.mass > 100000 and shapeB_system.mass > 100000:
		return
	
	result = sat.collideCircleCircleV(shapeA.center,shapeA.radius,shapeB.center,shapeB.radius)
			
	if result[0]:
		
		circleA.system.friction_ratios[0] = 1.0
		circleB.system.friction_ratios[0] = 1.0
		circleA.system.num_collisions += 1
		circleB.system.num_collisions += 1
		
		delta_mass = shapeA_system.mass + shapeB_system.mass 
		
		if delta_mass == 0:
			m_A = 0
			m_B = 0
		else:
			m_A = shapeB_system.mass / float(delta_mass)
			m_B = shapeA_system.mass / float(delta_mass)
			
			# i swapped the masses around
			
		overlap = (abs(result[1])) 
		
		
		dir = (result[2])                           #is the penetration direction
		
		d = shapeA.center - shapeB.center
		
		if d.dotProduct(dir) < 0:
			dir = -dir
			
		dir *= (overlap) 
		
		
		shapeA.x += (dir.x * m_A)
		shapeA.y += (dir.y * m_A)
		
		shapeB.x -= (dir.x * m_B)
		shapeB.y -= (dir.y * m_B)
		
		shapeA_system.currentParticlePos[0].x = shapeA.x
		shapeA_system.currentParticlePos[0].y = shapeA.y
		
		shapeB_system.currentParticlePos[0].x = shapeB.x
		shapeB_system.currentParticlePos[0].y = shapeB.y
		
