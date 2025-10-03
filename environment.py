import pygame
import math
import random
import sys
import numpy as np
from pygame.locals import *
from collision import *
from contact import *
from body import *



class Environment:

    gravity = np.array([0.0, 0.0])
    bodyList = []
    contactPairs = []
    contactList = np.array([[0.0,0.0],
                            [0.0,0.0]])
    impulseList = np.array([[0.0,0.0],
                            [0.0,0.0]])
    raList = np.array([[0.0,0.0],
                            [0.0,0.0]])
    rbList = np.array([[0.0,0.0],
                            [0.0,0.0]])
    frictionImpulseList = np.array([[0.0,0.0],
                            [0.0,0.0]])
    jList = np.array([0.0,0.0])
    

    def bodyCount(self):
        return len(self.bodyList)
    
    def addBody(self,bodyA):
        self.bodyList.append(bodyA)

    def removeBody(self, bodyA):
        self.bodyList.remove(bodyA)

    def getBody(self, index):
        if(index >= len(self.bodyList)):
            return False
        else:
            return self.bodyList[index]

    def processPhysics(self):
        for i in range(len(self.bodyList)):
            for j in range(i+1, len(self.bodyList)):
                bodyA = self.bodyList[i]
                bodyB = self.bodyList[j]
                normal = np.array([0.0,0.0])
                depth = -1
                
                normal, depth, isCollide = Collision.Collide(bodyA, bodyB)
                


                if(isCollide):
                    if(np.dot(normal, bodyB.center-bodyA.center) < 0):
                        normal = -normal
                    Environment.separate(bodyA, bodyB, np.dot(normal,depth))
                    contactp1, contactp2, contactC = Collision.findContacts(bodyA, bodyB)
                    contact = Contact(bodyA, bodyB, normal, depth, contactp1, contactp2, contactC)
                    self.resolveCollision(contact)
                
    def update(self, time):

        for i in range(10):
            self.contactPairs.clear()
            self.updateBodies(time, 10)
            self.processPhysics()

    def updateBodies(self, time, iterations):
        for i in range(len(self.bodyList)):
            self.bodyList[i].update(self.gravity, time, iterations)



    def separate(bodyA, bodyB, minDisp):
    
        if(bodyA.isStatic):
            bodyB.move(minDisp)

        elif(bodyB.isStatic):
            bodyA.move(-minDisp)

        else:
            bodyA.move(-minDisp/2)
            bodyB.move(+minDisp/2)

    def resolveCollision(self,contact):
        bodyA = contact.bodyA
        bodyB = contact.bodyB

        normal = contact.normal

        contactp1 = contact.contactp1
        contactp2 = contact.contactp2

        contactC = contact.contactC

        e = 0.3 # Create way to get e
        staticMu = 0.3
        dynamicMu = 0.2

        self.contactList[0] = contactp1
        self.contactList[1] = contactp2

        for i in range(contactC):
            self.impulseList = np.array([[0.0,0.0],
                            [0.0,0.0]])
            self.raList = np.array([[0.0,0.0],
                                    [0.0,0.0]])
            self.rbList = np.array([[0.0,0.0],
                                    [0.0,0.0]])
            self.frictionImpulseList = np.array([[0.0,0.0],
                                    [0.0,0.0]])
            self.jList = np.array([0.0,0.0])
            
            
       

        for i in range(contactC):

            ra = self.contactList[i] - bodyA.center
            rb = self.contactList[i] - bodyB.center
            
            self.raList[i] = ra
            self.rbList[i] = rb
            raPerp = np.array([-ra[1], ra[0]])
            rbPerp = np.array([-rb[1], rb[0]])

            velA = np.dot(raPerp,bodyA.angularVelocity)
            velB = np.dot(rbPerp,bodyB.angularVelocity)

            relativeVelocity = bodyB.linearVelocity + velB - bodyA.linearVelocity - velA
            

            contactVelocity = np.dot(relativeVelocity, normal)
            
            if(contactVelocity > 0):
                continue

            raPerpN = np.dot(raPerp, normal)
            rbPerpN = np.dot(rbPerp, normal)

            denominator = denominator = (
                bodyA.invMass
                + bodyB.invMass
                + (raPerpN * raPerpN) *(bodyA.invInertia) + (rbPerpN*rbPerpN)*(bodyB.invInertia)
            )

            impulse = -1*(1+e)*contactVelocity
            impulse/=denominator
            impulse/=contactC

            self.jList[i] = impulse

            self.impulseList[i] = impulse*normal


        

        for i in range(contactC):
            impulse = self.impulseList[i]
            ra = self.raList[i]
            rb = self.rbList[i]

            bodyA.linearVelocity += -(impulse)*bodyA.invMass
            bodyB.linearVelocity += impulse*bodyB.invMass

            bodyA.angularVelocity += -np.cross(ra, impulse)*bodyA.invInertia
            bodyB.angularVelocity += np.cross(rb, impulse)*bodyB.invInertia


        for i in range(contactC):
            ra = self.contactList[i] - bodyA.center
            rb = self.contactList[i] - bodyB.center

            self.raList[i] = ra
            self.rbList[i] = rb

            raPerp = np.array([-ra[1], ra[0]])
            rbPerp = np.array([-rb[1], rb[0]])



            velA = raPerp * bodyA.angularVelocity
            velB = rbPerp * bodyB.angularVelocity

            relativeVelocity = bodyB.linearVelocity + velB - bodyA.linearVelocity - velA

            tangent = relativeVelocity - np.dot(relativeVelocity, normal) * normal


            if(np.linalg.norm(tangent) <= 0.005):
                continue

            else:
                tangent /= np.linalg.norm(tangent)

            raPerpDotT = np.dot(raPerp, tangent)
            rbPerpDotT = np.dot(rbPerp, tangent)

            denominator = bodyA.invMass + bodyB.invMass + (raPerpDotT*raPerpDotT)*(bodyA.invInertia) + (rbPerpDotT*rbPerpDotT)*(bodyB.invInertia)

            jt = -np.dot(relativeVelocity, tangent) 
            jt /= denominator
            jt /= contactC

            frictionImpulse = 0
            
            impulse = self.jList[i]

            if(abs(jt) <= (impulse*staticMu)):
                frictionImpulse = jt*tangent

            else:
                frictionImpulse = -impulse*tangent*dynamicMu

            self.frictionImpulseList[i] = frictionImpulse
            

        for i in range(contactC):
            frictionImpulse = self.frictionImpulseList[i]
            ra = self.raList[i]
            rb = self.rbList[i]

            bodyA.linearVelocity += -frictionImpulse*bodyA.invMass
            bodyB.linearVelocity += frictionImpulse*bodyB.invMass

            bodyA.angularVelocity += -np.cross(ra, frictionImpulse)*bodyA.invInertia
            bodyB.angularVelocity += np.cross(rb, frictionImpulse)*bodyB.invInertia


