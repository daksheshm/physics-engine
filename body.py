import pygame
import math
import random
import sys
import numpy as np
from pygame.locals import *

class Body:
    def __init__(self,img, center, mass, inertia, isStatic, radius, width, height, type, shape, linearVelocity, angle, angularVelocity):
        self.img = img
        self.center = center
        self.mass = mass
        self.inertia = inertia
        self.isStatic = isStatic
        self.radius = radius
        self.width = width
        if(isStatic):
            self.invMass = 0
            self.invInertia = 0
        else:
            self.invMass = 1/mass
            self.invInertia = 1/inertia
        self.height = height
        self.type = type
        self.shape = shape
        self.angle = angle
        self.angularVelocity = angularVelocity
        self.linearVelocity = linearVelocity


    def getVertices(self):
        angle = self.angle
        width = self.height
        height = self.width
        alpha = np.arctan(height/width)
        diag = np.sqrt(width*width + height*height)

        vertices = []
        vertices.append(np.array([self.center[0]-diag/2*np.sin(angle+alpha)+np.sin(angle)*width, self.center[1]+diag/2*np.cos(angle+alpha)-width*np.cos(angle)]))
        vertices.append(np.array([self.center[0]-diag/2*np.sin(angle+alpha)+np.sin(angle)*width+height*np.cos(angle), self.center[1]+diag/2*np.cos(angle+alpha)-width*np.cos(angle)+np.sin(angle)*height]))
        vertices.append(np.array([self.center[0]-diag/2*np.sin(angle+alpha)+height*np.cos(angle), self.center[1]+diag/2*np.cos(angle+alpha)+np.sin(angle)*height]))
        vertices.append(np.array([self.center[0]-diag/2*np.sin(angle+alpha), self.center[1]+diag/2*np.cos(angle+alpha)]))
        
        return vertices
    
    
    def move(self, amount):
        if(self.isStatic):
            return
        
        self.center += amount
        pass

    def rotate(self, amount):
        self.angle += amount

    def addForce(self, amount):
        self.force = amount

    def type(self):
        return self.type

    def update(self,gravity, time, iterations):
        if(self.isStatic):
            return
        time/=iterations

        self.linearVelocity += gravity*time
        self.center += self.linearVelocity*time
        self.angle += self.angularVelocity*time
        self.force = np.array([0.0,0.0])
