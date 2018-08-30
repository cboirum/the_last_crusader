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
PPM = 20

class PygameDraw(b2DrawExtended):
    """
    This debug draw class accepts callbacks from Box2D (which specifies what to draw)
    and handles all of the rendering.

    If you are writing your own game, you likely will not want to use debug drawing.
    Debug drawing, as its name implies, is for debugging.
    """
    surface = None
    axisScale = 10.0
    def __init__(self, **kwargs): 
        b2DrawExtended.__init__(self, **kwargs)
        self.flipX = False
        self.flipY = True
        self.convertVertices = True

    def StartDraw(self):
        self.zoom=self.test.viewZoom
        self.center=self.test.viewCenter
        self.offset=self.test.viewOffset
        self.screenSize=self.test.screenSize

    def EndDraw(self): pass

    def DrawPoint(self, p, size, color):
        """
        Draw a single point at point p given a pixel size and color.
        """
        self.DrawCircle(p, size/self.zoom, color, drawwidth=0)
        
    def DrawAABB(self, aabb, color):
        """
        Draw a wireframe around the AABB with the given color.
        """
        points = [  (aabb.lowerBound.x, aabb.lowerBound.y ),
                    (aabb.upperBound.x, aabb.lowerBound.y ),
                    (aabb.upperBound.x, aabb.upperBound.y ),
                    (aabb.lowerBound.x, aabb.upperBound.y ) ]
        
        pygame.draw.aalines(self.surface, color, True, points)

    def DrawSegment(self, p1, p2, color):
        """
        Draw the line segment from p1-p2 with the specified color.
        """
        pygame.draw.aaline(self.surface, color.bytes, p1, p2)

    def DrawTransform(self, xf):
        """
        Draw the transform xf on the screen
        """
        p1 = xf.position
        p2 = self.to_screen(p1 + self.axisScale * xf.R.col1)
        p3 = self.to_screen(p1 + self.axisScale * xf.R.col2)
        p1 = self.to_screen(p1)

        pygame.draw.aaline(self.surface, (255,0,0), p1, p2)
        pygame.draw.aaline(self.surface, (0,255,0), p1, p3)

    def DrawCircle(self, center, radius, color, drawwidth=1):
        """
        Draw a wireframe circle given the center, radius, axis of orientation and color.
        """
        radius *= self.zoom
        if radius < 1: radius = 1
        else: radius = int(radius)

        pygame.draw.circle(self.surface, color.bytes, center, radius, drawwidth)

    def DrawSolidCircle(self, center, radius, axis, color):
        """
        Draw a solid circle given the center, radius, axis of orientation and color.
        """
        radius *= self.zoom
        if radius < 1: radius = 1
        else: radius = int(radius)

        pygame.draw.circle(self.surface, (color/2).bytes+[127], center, radius, 0)
        pygame.draw.circle(self.surface, color.bytes, center, radius, 1)
        pygame.draw.aaline(self.surface, (255,0,0), center, (center[0] - radius*axis[0], center[1] + radius*axis[1])) 

    def DrawPolygon(self, vertices, color):
        """
        Draw a wireframe polygon given the screen vertices with the specified color.
        """
        if not vertices:
            return

        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color.bytes, vertices[0], vertices)
        else:
            pygame.draw.polygon(self.surface, color.bytes, vertices, 1)
        
    def DrawSolidPolygon(self, vertices, color):
        """
        Draw a filled polygon given the screen vertices with the specified color.
        """
        if not vertices:
            return

        if len(vertices) == 2:
            pygame.draw.aaline(self.surface, color.bytes, vertices[0], vertices[1])
        else:
            pygame.draw.polygon(self.surface, (color/2).bytes+[127], vertices, 0)
            pygame.draw.polygon(self.surface, color.bytes, vertices, 1)
            
class Pyramid (Framework):
    name="Pyramid"
    delay = 100
    interval = 200
    def __init__(self):
        super(Pyramid, self).__init__()
        self.renderrer = PygameDraw(surface=self.screen, test=self)
        self.flightModel = 'Arcade' #'Realistic'
        #ground = self.addTerrain()
        #self.addPyramids()
        self.ground = self.addTestTerrain()
        self.ground.userData = 'Ground'
        self.ground.bullet = True
        
        position = (40,50)
        self.crusader1 = crusader1 = self.addCrusader(position)
        self.crusader1.userData = 'PlayerAircraft'
        self.player = Assets.PlayerObject()
        self.player.vehicle = crusader1
        self.player.vehicle.sleepingAllowed = False
        self.player.vehicle.bullet = True
        self.player.vehicle.fixedRotation = True
        #self.crusader1.ApplyForce((-2000000,1),(-1000,0))
    
        """
        Player manuvering parameters(TODO: Impliment within vehicle definition)
        """
        self.player.upForce = -1000000
        self.player.pitchTorque = 10000000
        self.player.moveDist = .6 #(meters)
    
    def Step(self, settings):
        """
        The main physics step.

        Takes care of physics drawing (callbacks are executed after the world.Step() )
        and drawing additional information.
        """
        
        #Apply special collision functions (Such as destructable bodies/terrain
        #FEA, sph, dynamic meshing, energy calculations, dynamic timestepping, etc. etc.
        body_pairs = [(p['fixtureA'].body, p['fixtureB'].body) for p in self.points]
        #print body_pairs
        if self.player.vehicle.contacts:
            cList = self.player.vehicle.contacts
            
            p1=cList[0].contact.worldManifold.points[0]
            p2=cList[0].contact.worldManifold.points[1]
            n1 = cList[0].contact.worldManifold.normal

            self.renderer.DrawPoint(self.renderer.to_screen(p1), 10, self.colors['contact_add'])
            self.world.CreateDynamicBody()
            p1a = self.renderer.to_screen(p1)
            p2a = self.renderer.axisScale * n1 + p1a
            print self.points
            #self.renderer.DrawSegment(p1a, p2a, self.colors['contact_normal']) 
                    
            self.jumpDist = 5
            self.player.vehicle.position += n1*self.jumpDist
            print "OUCH!"
            #print self.player.vehicle.contacts
        #print 
        for body1, body2 in body_pairs:
            print 'hey'
            if self.player.vehicle in [body1,body2]:
                print"IMPACT!!!"
        
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
        if key in (Keys.K_1, Keys.K_2, Keys.K_3, Keys.K_4, Keys.K_5):
                self.addCrusader((40,50))
                
        if self.flightModel == 'Realistic':
            if key == Keys.K_w:
                f = self.crusader1.GetWorldVector(localVector=(0.0, self.upForce))
                p = self.crusader1.GetWorldPoint(localPoint=self.crusader1.localCenter)
                self.crusader1.ApplyForce(f, p,)
            if key == Keys.K_s:
                f = self.crusader1.GetWorldVector(localVector=(0.0, self.upForce))
                p = self.crusader1.GetWorldPoint(localPoint=self.crusader1.localCenter)
                self.crusader1.ApplyForce(-f, p,)
            if key == Keys.K_a:
                self.crusader1.ApplyTorque(self.pitchTorque)
            if key == Keys.K_d:
                self.crusader1.ApplyTorque(-self.pitchTorque)
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
    
    def addTestTerrain(self):
        return Assets.terrainTester(self.world,2000)
    
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
