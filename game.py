import pygame
import math
import random
import sys
import numpy as np
from pygame.locals import *
from environment import *

pygame.init()

clock = pygame.time.Clock()
FPS = 120

screen_width = 1600
screen_height = 900
screen = pygame.display.set_mode((screen_width,screen_height))
pygame.display.set_caption("Angry Birds")

run = True

dot_surface = pygame.Surface((5 * 2, 5 * 2), pygame.SRCALPHA)
pygame.draw.circle(dot_surface, 'red', (5, 5), 5)

environment = Environment()

environment.addBody(Body('t1.png', np.array([800.0,450.0]), 10, 10, True, 0, 1600,60,"rect","rect", np.array([0.0,0.0]), 0,0 ))
while(run):
    screen.fill((0, 0, 0))  # fill screen with black (or any background color)

    for event in pygame.event.get():
        if(event.type == pygame.QUIT):
            run = False

        elif(event.type == pygame.MOUSEBUTTONDOWN):
            if(pygame.mouse.get_pressed()[0]):
                mx,my = pygame.mouse.get_pos()
                mx = float(mx)
                my = float(my)
                body = Body('t1.png', np.array([mx,my]), 10,5*7200,False, 10, 60,60, "rect", "rect", np.array([0.0,0.0]), 0, 0)
                environment.addBody(body)
            elif(pygame.mouse.get_pressed()[2]):
                mx,my = pygame.mouse.get_pos()
                mx = float(mx)
                my = float(my)

                body = Body('t1.png', np.array([mx,my]), 10, 3600, False, 30, 60, 60, "circle", "circle", np.array([0.0, 0.0]), 0, 0)
                environment.addBody(body)

    keys = pygame.key.get_pressed()
    if(keys[pygame.K_DOWN]):
        environment.bodyList[1].linearVelocity[1]+=5
    if(keys[pygame.K_UP]):
        environment.bodyList[1].linearVelocity[1]-=5
    if(keys[pygame.K_RIGHT]):
        environment.bodyList[1].linearVelocity[0]+=5
    if(keys[pygame.K_LEFT]):
        environment.bodyList[1].linearVelocity[0]-=5
    if(keys[pygame.K_u]):
        environment.bodyList[1].angle += 1
    environment.update(1/60)

    for body in environment.bodyList:
        temp = pygame.image.load(body.img)
        temp.set_colorkey(0)
        image = pygame.transform.rotate(temp, -body.angle*180/np.pi)
        rect = image.get_rect(center=body.center)
        screen.blit(image, rect)
        if(body.shape == "rect"):
            
            for j in body.getVertices():
                
                screen.blit(dot_surface, j-np.array([5,5]))

        
    
    pygame.display.flip()
    clock.tick(FPS)