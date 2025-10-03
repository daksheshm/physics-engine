import pygame
import math
import random
import sys
import numpy as np
from pygame.locals import *

class Collision:

    def findContactPointCR(circle, rect):
        contactp1 = np.array([0,0])
        min_dist_sq = float('inf')
        
        vertices1 = rect.getVertices()
        for i in range(4):
            va = vertices1[i]
            vb = vertices1[(i+1)%4]

            dist_sq, contact = Collision.distPointLine(circle.center, va, vb)

            if dist_sq < min_dist_sq:
                min_dist_sq = dist_sq
                contactp1 = contact
        
        
        return contactp1

    def findContactPointCC(circle1, circle2):
        vec = circle2.center - circle1.center
        vec /= np.linalg.norm(vec)

        return circle1.center + circle1.radius * vec

    def findContactPointsRR(rect1, rect2):
        vertices1 = rect1.getVertices()
        vertices2 = rect2.getVertices()

        contactp1 = np.array([0,0])
        contactp2 = np.array([0,0])
        contact_count = 0
        min_dist_sq = float('inf')

        for p in vertices1:
            for j in range(len(vertices2)):
                va = vertices2[j]
                vb = vertices2[(j + 1) % len(vertices2)]
                dist_sq, cp = Collision.distPointLine(p, va, vb)

                if (dist_sq-min_dist_sq <= 0.005 and dist_sq-min_dist_sq>=-0.005):
                    if (np.linalg.norm(cp-contactp1) > 0.005):
                        contactp2 = cp
                        contact_count = 2
                elif dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    contactp1 = cp
                    contact_count = 1

        for p in vertices2:
            for j in range(len(vertices1)):
                va = vertices1[j]
                vb = vertices1[(j + 1) % len(vertices1)]
                dist_sq, cp = Collision.distPointLine(p, va, vb)

                if abs(dist_sq-min_dist_sq) <= 0.005:
                    if np.linalg.norm(cp-contactp1) > 0.005:
                        contactp2 = cp
                        contact_count = 2
                elif dist_sq < min_dist_sq:
                    min_dist_sq = dist_sq
                    contactp1 = cp
                    contact_count = 1

        if contact_count == 2:
            return contactp1, contactp2,2
        else:
            return contactp1, np.array([0.0,0.0]), 1



    def distPointLine(point, endpt1, endpt2):
        seg12 = endpt2 - endpt1
        seg1p = point - endpt1

        proj = np.dot(seg12, seg1p)

        d = proj/np.linalg.norm(seg12)
        d /= np.linalg.norm(seg12)

        closestPt = np.array([0,0])

        if(d <= 0):
            closestPt = endpt1

        elif(d >= 1):
            closestPt = endpt2
        
        else:
            closestPt = endpt1 + seg12 * d

        sqDistance = np.linalg.norm(point-closestPt)*np.linalg.norm(point-closestPt)

        return sqDistance, closestPt
    
    def findContacts(bodyA, bodyB):
        contactp1 = np.array([0,0])
        contactp2 = np.array([0,0])
        contactC = 0

        if(bodyA.shape == "rect"):
            if(bodyB.shape == "rect"):
                contactp1, contactp2, contactC = Collision.findContactPointsRR(bodyA, bodyB)
                
            else:
                contactp1 = Collision.findContactPointCR(bodyB, bodyA)
                contactC = 1

        else:
            if(bodyB.shape == "rect"):
                contactp1 = Collision.findContactPointCR(bodyA, bodyB)
                contactC = 1
            else:
                contactp1 = Collision.findContactPointCC(bodyA, bodyB)
                contactC = 1
        
        if(contactC == 1):
            return contactp1, np.array([0.0,0.0]), contactC
        else:
            return contactp1, contactp2, contactC
        

    def Collide(bodyA, bodyB):

        normal = np.array([0,0])
        depth = 0
        isCollide = False

        if(bodyA.shape == "rect"):
            if(bodyB.shape == "rect"):
                normal, depth, isCollide = Collision.intersectRR(bodyA, bodyB)
               
                

            else:
                normal, depth, isCollide = Collision.intersectCR(bodyB, bodyA)

        else:
            if(bodyB.shape == "rect"):
                normal, depth, isCollide = Collision.intersectCR(bodyA, bodyB)
            else:
                normal, depth, isCollide = Collision.intersectCC(bodyA, bodyB)

        return normal, depth, isCollide
    def projectVertices(axis, vertices):
        minVal, maxVal = np.dot(axis, vertices[0]), np.dot(axis, vertices[0])
        for i in range(len(vertices)):
            minVal = min(minVal, np.dot(axis,vertices[i]))
            maxVal = max(maxVal, np.dot(axis,vertices[i]))

        return minVal, maxVal


    def intersectRR(rect1, rect2):
        normal = np.array([0,0])
        depth = float('inf')
        vertices1 = rect1.getVertices()
        vertices2 = rect2.getVertices()
        for i in range(4):
            va = vertices1[i]
            vb = vertices1[(i+1)%4]

            edge = vb-va
            axis = np.array([-edge[1], edge[0]])
            axis /= np.linalg.norm(axis)

            minA, maxA = Collision.projectVertices(axis, vertices1)
            minB, maxB = Collision.projectVertices(axis, vertices2)

            if(minA >= maxB or minB >= maxA):
                return np.array([0.0,0.0]),-1,False
            
            axisDepth = min(maxB-minA, maxA-minB)

            if(axisDepth < depth):
                depth = axisDepth
                normal = axis

                if(np.dot(normal, rect2.center-rect1.center) < 0):
                    normal = -normal

        return normal, depth, True
    
    def intersectCR(circle, rect):

        normal = np.array([0.0,0.0])
        depth = float('inf')
        axis = normal
        axisDepth = 0.0
        vertices = rect.getVertices()

        for i in range(4):
            va = vertices[i]
            vb = vertices[(i+1)%4]

            edge = vb - va
            axis = np.array([-edge[1], edge[0]])
            axis /= np.linalg.norm(axis)

            minR, maxR = Collision.projectVertices(axis, vertices)
            minC, maxC = np.dot(circle.center,axis)-circle.radius, np.dot(circle.center, axis)+circle.radius

            if(minC >= maxR or minR >= maxC):
                return np.array([0.0,0.0]),-1,False
            
            axisDepth = min(maxC-minR ,  maxR-minC)
            if(axisDepth < depth):
                depth = axisDepth
                normal = axis

        verticesNew = sorted(vertices, key = lambda x:np.linalg.norm(circle.center-x))

        axis = verticesNew[0] - circle.center
        axis /= np.linalg.norm(axis)

        minR, maxR = Collision.projectVertices(axis, vertices)
        minC, maxC = np.dot(circle.center,axis)-circle.radius, np.dot(circle.center, axis)+circle.radius

        if(minC >= maxR or minR >= maxC):
            return np.array([0.0,0.0]),-1,False
            
        axisDepth = min(maxC-minR ,  maxR-minC)
        if(axisDepth < depth):
            depth = axisDepth
            normal = axis


        if(np.dot(rect.center-circle.center, normal) < 0):
            normal = -normal
     
        return normal,depth, True
    
    def intersectCC(circle1, circle2):
        normal = (circle2.center-circle1.center)/np.linalg.norm(circle1.center - circle2.center)
        depth = circle2.radius + circle1.radius - np.linalg.norm(circle1.center - circle2.center)

        if(depth <= 0):
            return np.array([0.0,0.0]),-1,False
        
        else:
            return normal, depth, True
        




    