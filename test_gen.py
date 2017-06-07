import pygame,sys, gen_collision
from pygame.locals import*

pygame.init()

WIDTH = 320
HEIGHT = 240

pygame.display.set_caption("DEBUG LINES")
screen = pygame.display.set_mode((WIDTH,HEIGHT))

l1s = [10.0,10.0]
l1e = [60.0,15.0]

l2s = [90.0,10.0]
l2e = [160.0,115.0]

line1 = gen_collision.Line(l1s,l1e,(100,0,100))
line2 = gen_collision.Line(l2s,l2e,(100,200,0))

dt = 0
clock = pygame.time.Clock()

while True:
	for event in pygame.event.get():
		if event.type == QUIT:
			pygame.quit()
			sys.exit()
			
		if event.type == KEYDOWN:
		#	if event.key == K_LEFT:
		#		l1s[0] -= 1 ; l1e[0] -= 1 
			
			if event.key == K_ESCAPE:
				pygame.quit()
				sys.exit()
		
		#	if event.key == K_RIGHT:
		#		l1s[0] += 1 ; l1e[0] += 1 
				
		#	if event.key == K_UP:
		#		l1s[1] -= 1 ; l1e[1] -= 1 
			
				
		#	if event.key == K_DOWN:
		#		l1s[1] += 1 ; l1e[1] += 1 
		
	states = pygame.key.get_pressed()
	
	if states[K_LEFT]:
		l1s[0] -= 1 ; l1e[0] -= 1
	if states[K_RIGHT]:
		l1s[0] += 1 ; l1e[0] += 1
	if states[K_UP]:
		l1s[1] -= 1 ; l1e[1] -= 1
	if states[K_DOWN]:
		l1s[1] += 1 ; l1e[1] += 1
		
				
	line1.update(l1s,l1e)
	
	result = gen_collision.calculateIntersectPoint(l2s,l2e,l1s,l1e)
	
	screen.fill((0,0,0))
	
	line1.draw(screen)
	line2.draw(screen)
	
	if result is not None:
		
		pygame.draw.circle(screen,(255,10,0),result,4)
	
	pygame.display.flip()
	
	
	dt = clock.tick(60)