import pygame
import math
import random
import sys
import numpy as np
from pygame.locals import *

class Contact:
    def __init__(self,bodyA, bodyB, normal, depth, contactp1, contactp2, contactC):
        self.bodyA = bodyA
        self.bodyB = bodyB
        self.normal = normal
        self.depth = depth
        self.contactp1 = contactp1
        self.contactp2 = contactp2
        self.contactC = contactC