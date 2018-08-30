#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version by Ken Lauer / sirkne at gmail dot com
# 
# This software is provided 'as-is', without any express or implied
# warranty.  In no event will the authors be held liable for any damages
# arising from the use of this software.
# Permission is granted to anyone to use this software for any purpose,
# including commercial applications, and to alter it and redistribute it
# freely, subject to the following restrictions:
# 1. The origin of this software must not be misrepresented; you must not
# claim that you wrote the original software. If you use this software
# in a product, an acknowledgment in the product documentation would be
# appreciated but is not required.
# 2. Altered source versions must be plainly marked as such, and must not be
# misrepresented as being the original software.
# 3. This notice may not be removed or altered from any source distribution.

import pygame
from framework import *
import spriteUtils4
import Assets
PPM = 20 #Pixels Per Meter
biteSize = 10 #Initial domain cut radius
particlePerBite = 10 #Particals Per cut diameter

specialContact = True
class Pyramid (Framework):
    name="Pyramid"
    delay = 100
    interval = 200
    tempBodies = []
    tempBodiesLimit = 100
    contactPoints = {}
    contactVels = {}
    breakBodies = []
    breakables = []
    groundMethod = 'Default'#New1
    upForce = 1000000
    pitchTorque = 10000000
    def __init__(self):
        super(Pyramid, self).__init__()
        self.flightModel = 'Realistic'#'Arcade' #'Realistic'
        #ground = self.addTerrain()
        #self.addPyramids()
        vertLimit = 2000
        self.ground = self.addTestTerrain(vertLimit)
        self.ground.userData = 'Ground'
        self.ground.bullet = True
        self.settings.drawContactPoints = True
        self.settings.hz= 240.0
        self.settings.enableSubStepping = False
        self.settings.velocityIterations = 1
        self.settings.positionIterations = 1
        self.cutRadius = biteSize
        self.packing = particlePerBite #particles per cutRadius chord        
        
        
        position = (40,50)
        
        
        
        
        #position = (40,10)
        
        userData = { 'ID': 'PlayerAircraft',
                     'limitKE': 1170.315759,
                     'Polygon': [],
                     'Breakable': True,
                     'Stiffness': 10000}
        
        self.crusader1 = crusader1 = self.addCrusader(position)
        self.crusader1.userData = userData
        
        userData = { 'ID': 'Box1',
                     'limitKE': 917.315759,
                     'Polygon': [],
                     'Breakable': True,
                     'Stiffness': 10}
        
        body2 = self.world.CreateDynamicBody(
                position=position, 
                userData = userData
                )
        bH=80
        bL = 50
        boxV = [(0,0),
                    (bL,0),
                    (bL,bH),
                    (0,bH),]
        fixture = body2.CreatePolygonFixture(vertices=boxV, density=.1, friction=1)
#        boxV = [(bL,0),
#                    (2*bL,0),
#                    (2*bL,bH),
#                    (bL,bH),]
        position = (40+bL,50)
        userData = { 'ID': 'Box2',
                     'limitKE': 917.315759,
                     'Polygon': [],
                     'Breakable': True,
                     'Stiffness': 10}
        body3 = self.world.CreateDynamicBody(
                position=position, 
                userData = userData
                )
        body3.CreatePolygonFixture(vertices=boxV, density=.1, friction=1)
        
        self.playerBody = playerBody = crusader1#body2#crusader1
        
        if playerBody:
            self.player = Assets.PlayerObject()
            self.addPlayer(playerBody,position)
            
        else:
            self.player = False
        
#        if self.flightModel != 'Realistic':
#            self.player.vehicle.fixedRotation = True
        #self.crusader1.ApplyForce((-2000000,1),(-1000,0))
    
        
        
        """
        geometry subtraction section
        """
        #create square
        squareVerts = [(1,1),
                       (1,2),
                       (2,1),
                       (2,2)]
        squareEdges = [(1,2),
                       (2,3),
                       (3,4),
                       (4,1)]
        
        #create fake contact point on square
        contact = (1,.5)
        
        #create simulation area for squre (a circle to start with that will be 
        #subtracted from the square geometry as a hole and square will be remeshed
        #with trimesh
    def addPlayer(self,playerBody,position):
        
        
        self.player.vehicle = playerBody
        self.player.vehicle.sleepingAllowed = False
        self.player.vehicle.bullet = True
        self.contactPoints[playerBody.userData['ID']] = []
        """
        Player manuvering parameters(TODO: Impliment within vehicle definition)
        """
        self.player.upForce = -1000000
        self.player.pitchTorque = 10000000
        self.player.moveDist = .6 #(meters)
        
    def BeginContact(self, contact):
        """
        Called when two objects are touching
        """
        #print contact
        A = contact.fixtureA.body
        B = contact.fixtureB.body
#        if self.crusader1 in [A,B]:
#            print"Player Hit"
        #print A
        #print B
        """
        Simple contact points relative velocity approximation assuming little input from rotation (valid except for baseball bats)
        """
        mass = B.mass+A.mass
        relativeVelocity = A.linearVelocity-B.linearVelocity
        
        import math
        relativeVelocityMag = math.sqrt(relativeVelocity[0]*relativeVelocity[0] + relativeVelocity[1]*relativeVelocity[1])
        relVelNorm = relativeVelocity*1.0/relativeVelocityMag
#        print mass
#        print relativeVelocity
#        print relativeVelocityMag
#        print relativeVelocityMag*relativeVelocityMag
        kineticEnergy =  KE = 1.0/2.0*mass*relativeVelocityMag*relativeVelocityMag
        #print kineticEnergy
        point = contact.worldManifold.points[0]
        
        #pointB = contact.worldManifold.points[1]
        Acan = type(A.userData)==type({})
        Bcan = type(B.userData)==type({})
        contactAllowed = Acan and Bcan
        allowing = True
        A_KE  = A.fixtures[0].density*relativeVelocityMag*relativeVelocityMag/2.0
        B_KE  = B.fixtures[0].density*relativeVelocityMag*relativeVelocityMag/2.0
        if specialContact:
            #if contactAllowed or allowing:
            for C in [A,B]:
                thisKE = C.fixtures[0].density*relativeVelocityMag*relativeVelocityMag/2.0
                if thisKE == A_KE:
                    otherKE = B_KE
                    other = B
                else:
                    otherKE = A_KE
                    other = A
                if type(C.userData)==type({}) and otherKE>1000:
                    print C.userData
                    print"otherKE: %f"%otherKE
                #steel should break at 13715996?
                if type(C.userData)==type({}):
                    if otherKE > C.userData['limitKE']:
                        #print"Object will break"
                        #if not C.userData['ID'] in self.contactPoints.keys():
                        if not C in self.breakables:
                            self.breakables.append(C)
                            self.contactPoints[C.userData['ID']]=[]
                            self.contactVels[C.userData['ID']]=[] 
                        self.contactPoints[C.userData['ID']].append(C.GetLocalPoint(point))
                        self.contactVels[C.userData['ID']].append(other.linearVelocity)#FromWorldPoint(other.GetWorldPoint(point)))
                        #self.breakBodies.append((C,KE,point,relVelNorm))
                        
                        contact.enabled = False
                    
        #pygame.quit()

    def EndContact(self, contact):
        """
        Called only once, as soon as a contact is done happening
        """
        
    def PreSolve(self, contact, oldManifold):
        """
        Collision response has not been calculated yet
        """
        pass  
    
    def PostSolve(self, contact, impulse):
        """
        Collision response is available
        """
        #print impulse
        
    
        
    def Step(self, settings):
        """
        The main physics step.

        Takes care of physics drawing (callbacks are executed after the world.Step() )
        and drawing additional information.
        """
        
        #Apply special collision functions (Such as destructable bodies/terrain
        #FEA, sph, dynamic meshing, energy calculations, dynamic timestepping, etc. etc.
#        body_pairs = [(p['fixtureA'].body, p['fixtureB'].body) for p in self.points]
#        #print body_pairs
#        if self.player.vehicle.contacts:
#            cList = self.player.vehicle.contacts
#            
#            p1=cList[0].contact.worldManifold.points[0]
#            p2=cList[0].contact.worldManifold.points[1]
#            n1 = cList[0].contact.worldManifold.normal
#
#            self.renderer.DrawPoint(self.renderer.to_screen(p1), 10, self.colors['contact_add'])
#            
#            p1a = self.renderer.to_screen(p1)
#            p2a = self.renderer.axisScale * n1 + p1a
#            print self.points
#            #self.renderer.DrawSegment(p1a, p2a, self.colors['contact_normal']) 
#                    
#            self.jumpDist = 5
#            self.player.vehicle.position += n1*self.jumpDist
#            #self.tempBodies.append(Assets.addCircle(self.world,p1,self.jumpDist))
#            print "OUCH!"
#            #print self.player.vehicle.contacts
#        #print 
#        for body1, body2 in body_pairs:
#            print 'hey'
#            if self.player.vehicle in [body1,body2]:
#                print"IMPACT!!!"

        #for C,KE,point,relVelNorm in self.breakBodies:
        for body in self.breakables:
        #if len(self.breakables)>0:
            #POC = point
            r = self.cutRadius
            #body = C
            #body = self.player.vehicle
            #Assets.takeBite(POC,r,body)
            POCs = self.contactPoints[body.userData['ID']]
            VOCs = self.contactVels[body.userData['ID']]
            #Assets.takeBites(POCs,r,body)
            #Assets.takeBites4(POCs,r,body)
            Assets.takeBites5(POCs,r,body,VOCs = VOCs,packing=self.packing)
            
#            if len(self.tempBodies) < self.tempBodiesLimit:
#                self.tempBodies.append(Assets.addCircle(self.world,point,1))
#                self.tempBodies[-1].bullet = True
        #self.contactPoints[self.player.vehicle.userData['ID']] = []     
        self.contactPoints = {}  
        self.breakBodies = []
        self.breakables = []
        if self.player:
            self.player.update()
        
        self.stepCount+=1
        # Don't do anything if the setting's Hz are <= 0
        if settings.hz > 0.0:
            timeStep = 1.0 / settings.hz
        else:
            timeStep = 0.0
        
        # If paused, display so
        if settings.pause:
            if settings.singleStep:
                settings.singleStep=False
            else:
                timeStep = 0.0

            self.Print("****PAUSED****", (200,0,0))

        # Set the flags based on what the settings show
        if self.renderer:
            self.renderer.flags=dict(
                    drawShapes=settings.drawShapes,
                    drawJoints=settings.drawJoints,
                    drawAABBs =settings.drawAABBs,
                    drawPairs =settings.drawPairs,
                    drawCOMs  =settings.drawCOMs,
                    # The following is only applicable when using b2DrawExtended.
                    # It indicates that the C code should transform box2d coords to
                    # screen coordinates.
                    convertVertices=isinstance(self.renderer, b2DrawExtended) 
                    )

        # Set the other settings that aren't contained in the flags
        self.world.warmStarting=settings.enableWarmStarting
        self.world.continuousPhysics=settings.enableContinuous
        self.world.subStepping=settings.enableSubStepping

        # Reset the collision points
        self.points = []

        # Tell Box2D to step
        t_step=time()
        self.world.Step(timeStep, settings.velocityIterations, settings.positionIterations)
        self.world.ClearForces()
        t_step=time()-t_step

        # Update the debug draw settings so that the vertices will be properly
        # converted to screen coordinates
        t_draw=time()
        if self.renderer:
            self.renderer.StartDraw()

        self.world.DrawDebugData()

        # If the bomb is frozen, get rid of it.
        if self.bomb and not self.bomb.awake:
            self.world.DestroyBody(self.bomb)
            self.bomb = None
        
        # Take care of additional drawing (fps, mouse joint, slingshot bomb, contact points)

        if self.renderer:
            # If there's a mouse joint, draw the connection between the object and the current pointer position.
            if self.mouseJoint:
                p1 = self.renderer.to_screen(self.mouseJoint.anchorB)
                p2 = self.renderer.to_screen(self.mouseJoint.target)

                self.renderer.DrawPoint(p1, settings.pointSize, self.colors['mouse_point'])
                self.renderer.DrawPoint(p2, settings.pointSize, self.colors['mouse_point'])
                self.renderer.DrawSegment(p1, p2, self.colors['joint_line'])

            # Draw the slingshot bomb
            if self.bombSpawning:
                self.renderer.DrawPoint(self.renderer.to_screen(self.bombSpawnPoint), settings.pointSize, self.colors['bomb_center'])
                self.renderer.DrawSegment(self.renderer.to_screen(self.bombSpawnPoint), self.renderer.to_screen(self.mouseWorld), self.colors['bomb_line'])

            # Draw each of the contact points in different colors.
            if self.settings.drawContactPoints:
                for point in self.points:
                    if point['state'] == b2_addState:
                        self.renderer.DrawPoint(self.renderer.to_screen(point['position']), settings.pointSize, self.colors['contact_add'])
                    elif point['state'] == b2_persistState:
                        self.renderer.DrawPoint(self.renderer.to_screen(point['position']), settings.pointSize, self.colors['contact_persist'])

            if settings.drawContactNormals:
                for point in self.points:
                    p1 = self.renderer.to_screen(point['position'])
                    p2 = self.renderer.axisScale * point['normal'] + p1
                    self.renderer.DrawSegment(p1, p2, self.colors['contact_normal']) 

            self.renderer.EndDraw()
            t_draw=time()-t_draw

            t_draw=max(b2_epsilon, t_draw)
            t_step=max(b2_epsilon, t_step)

            try:
                self.t_draws.append(1.0/t_draw)
            except:
                pass
            else:
                if len(self.t_draws) > 2:
                    self.t_draws.pop(0)

            try:
                self.t_steps.append(1.0/t_step)
            except:
                pass
            else:
                if len(self.t_steps) > 2:
                    self.t_steps.pop(0)

            if settings.drawFPS:
                self.Print("Combined FPS %d" % self.fps)

            if settings.drawStats:
                self.Print("bodies=%d contacts=%d joints=%d proxies=%d" %
                    (self.world.bodyCount, self.world.contactCount, self.world.jointCount, self.world.proxyCount))

                self.Print("hz %d vel/pos iterations %d/%d" %
                    (settings.hz, settings.velocityIterations, settings.positionIterations))

                if self.t_draws and self.t_steps:
                    self.Print("Potential draw rate: %.2f fps Step rate: %.2f Hz" % (sum(self.t_draws)/len(self.t_draws), sum(self.t_steps)/len(self.t_steps)))
            
    
    
    def Keyboard(self, key):
        if self.player:
            if key in (Keys.K_1, Keys.K_2, Keys.K_3, Keys.K_4, Keys.K_5):
                    self.addCrusader((40,50))
                    
            if self.flightModel == 'Realistic':
                if key == Keys.K_w:
                    f = self.playerBody.GetWorldVector(localVector=(0.0, self.upForce))
                    p = self.playerBody.GetWorldPoint(localPoint=self.playerBody.localCenter)
                    self.playerBody.ApplyForce(f, p,)
                if key == Keys.K_s:
                    f = self.playerBody.GetWorldVector(localVector=(0.0, self.upForce))
                    p = self.playerBody.GetWorldPoint(localPoint=self.playerBody.localCenter)
                    self.playerBody.ApplyForce(-f, p,)
                if key == Keys.K_a:
                    self.playerBody.ApplyTorque(self.pitchTorque)
                if key == Keys.K_d:
                    self.playerBody.ApplyTorque(-self.pitchTorque)
            if self.flightModel == 'Arcade':
                if key == Keys.K_w:
                    self.player.move['Up'] = True
                if key == Keys.K_a:
                    self.player.move['Left'] = True
                if key == Keys.K_s:
                    self.player.move['Down'] = True
                if key == Keys.K_d:
                    self.player.move['Right'] = True
                
    def KeyboardUp(self, key):
        if self.flightModel == 'Arcade':
            if key == Keys.K_w:
                self.player.move['Up'] = False
            if key == Keys.K_a:
                self.player.move['Left'] = False
            if key == Keys.K_s:
                self.player.move['Down'] = False
            if key == Keys.K_d:
                self.player.move['Right'] = False
        

        
    def addCrusader(self,position):
        crusaderBase = Assets.Crusader(PPM,self.flightModel)
        #crusader1 = crusaderBase.add2World(self.world,(80,15))
        crusader1 = crusaderBase.add2World(self.world,position)
        #crusader2 = crusaderBase.add2World(self.world,(5,40))
        crusader1.angle=3.1415
        return crusader1
    
    def addTerrain(self):
        # The ground
        ground = self.world.CreateStaticBody(
                    shapes=b2EdgeShape(vertices=[(-800,0),(800, 0)])
                )
        return ground
    
    def addTestTerrain(self,vertLimit):
        return Assets.terrainTester(self.world,vertLimit)
    
    #def addRealTerrain(self):
    
    def addPyramids(self):
        box_half_size = (0.5, 0.5)
        box_density = 5.0
        box_rows = 20

        x=b2Vec2(-7, 0.75)
        deltaX=(0.5625, 1.25)
        deltaY=(1.125, 0)
        for i in range(box_rows):
            y = x.copy()

            for j in range(i, box_rows):
                self.world.CreateDynamicBody(
                    position=y,
                    fixtures=b2FixtureDef(
                            shape=b2PolygonShape(box=box_half_size),
                            density=box_density)
                    )

                y += deltaY

            x += deltaX
        pyramids = 3
        for i in range(pyramids):
            x=b2Vec2(-7-20*i, 0.75)
            for i in range(box_rows):
                y = x.copy()
    
                for j in range(i, box_rows):
                    self.world.CreateDynamicBody(
                        position=y,
                        fixtures=b2FixtureDef(
                                shape=b2PolygonShape(box=box_half_size),
                                density=box_density)
                        )
    
                    y += deltaY
    
                x += deltaX


if __name__=="__main__":
     main(Pyramid)
