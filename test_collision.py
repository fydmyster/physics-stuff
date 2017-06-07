import pygame,sys,sat
from pygame.locals import*
import Vector2

W = 320
H = 240

pygame.init()

screen = pygame.display.set_mode((W,H))

clock = pygame.time.Clock()
timepassed = 0

polyA = sat.RectSAT([40,40,(250,0,0)],30,60)
polyB = sat.Polygon([(50,100),(75,50),(100,100),(75,140)],(20,200,40))
#polyB.center = Vector2.Vector(75,95) 
circleA = sat.CircleSAT((100,100))

while True:
	for event in pygame.event.get():
		if event.type == QUIT:
			pygame.quit()
			sys.exit()
			
		if event.type == KEYDOWN:
			if event.key == K_ESCAPE:
				pygame.quit()
				sys.exit()
			if event.key == K_LEFT:
				polyA.x -= 4
			if event.key == K_RIGHT:
				polyA.x += 4
			if event.key == K_UP:
				polyA.y -= 4
			if event.key == K_DOWN:
				polyA.y += 4
	
	polyA.update()
	polyB.update()
	circleA.update()
	
	result = sat.collidePolygon(polyB,polyA)
	
	if result[0]:
		sat.resolveCollision(result,polyB,polyA,2)
	
	screen.fill((20,30,20))
	
	
	if result[0]:
		pygame.draw.rect(screen,(0,20,240),(0,0,80,10))
	
	polyA.draw(screen)
	polyB.draw(screen)
	circleA.draw(screen)
	
	pygame.display.flip()
	
	clock.tick(40)