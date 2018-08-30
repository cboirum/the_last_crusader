#!/usr/bin/env python
# -*- coding: utf-8 -*-
#
# C++ version Copyright (c) 2006-2007 Erin Catto http://www.box2d.org
# Python version Copyright (c) 2010 kne / sirkne at gmail dot com
# 
# Implemented using the pybox2d SWIG interface for Box2D (pybox2d.googlecode.com)
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
 
# Ported from the Cloth test by Paril, originally for Box2CS:
#   http://www.box2d.org/forum/viewtopic.php?f=6&t=6124
#
from framework import *

remove_neighbors = True #remove joints next those that break (to eliminate stringy leftover joints)
distance_factor_param = 1.33
segment_count=(4,20)
body_size=.5


def create_cloth(world, segment_count, body_size, position=(0,30),
        groupIndex=1, bar_height=0.5, base_hz=15*100000, 
        base_damping=100000000.11, density=1, **kwargs):
    segment_w, segment_h=segment_count
    body_spacing_w=body_size*2
    total_w=body_spacing_w*segment_w
    position=b2Vec2(*position)
     
    # The static bar at the top which holds the cloth
    #bar=world.CreateStaticBody(position=position)
    bar=world.CreateDynamicBody(position=position)
    bar.CreatePolygonFixture(box=(total_w/2.0+body_spacing_w, bar_height/2.0), 
            groupIndex=groupIndex)
    ground = world.CreateBody(
                    shapes=b2EdgeShape(vertices=[(-40,40),
                                                 (-40,0),
                                                 (40,0),
                                                 (40,40)]) 
                )
    
    box_fixture=b2FixtureDef(shape=b2CircleShape(radius=body_size),
                             groupIndex=groupIndex, density=density)
  
    weld_joints=[]
    distance_joints=[]
    cloth=[[None]*segment_h for x in range(segment_w)]
    for y in range(segment_h):
        pos=position-(total_w/2.0, y*body_spacing_w)
        for x in range(segment_w):
            pos+=(body_spacing_w, 0.0)
            body=world.CreateDynamicBody(position=pos, fixtures=box_fixture)
            body.fixed_rotation = True
            body.bullet = True
            cloth[x][y]=body

            if y==0:
                joint=world.CreateWeldJoint(bodyA=body, bodyB=bar, anchor=body.position)
                weld_joints.append(joint)
   
    connect_bodies=[] 
    #Build a list of body pairs to connect
    for y in range(segment_h):
        for x in range(segment_w):
            #if not on last one in row
            if x <= segment_w-2:
                left_body=cloth[x][y]
                right_body=cloth[x+1][y]
                connect_bodies.append((left_body, right_body))
                #if 
                if y < segment_h-1:
                    diag_body = cloth[x+1][y+1]
                    connect_bodies.append((left_body,diag_body))
                    if x > 0:
                        diag_body = cloth[x-1][y+1]
                        connect_bodies.append((left_body,diag_body))
            if y > 0:
                left_body=cloth[x][y]
                right_body=cloth[x][y-1]
                connect_bodies.append((left_body, right_body))

    #actually creat joints between listed bodies
    for bodyA, bodyB in connect_bodies:
        joint=world.CreateDistanceJoint(
                bodyA=bodyA,
                bodyB=bodyB,
                anchorA=bodyA.position,
                anchorB=bodyB.position,
                frequencyHz=base_hz+b2Random(0, base_hz/2.0),
                dampingRatio=base_damping+b2Random(0.01, base_damping),
                )
        distance_joints.append(joint)
        
    check_segments=[]
    for y in range(segment_h):
        for x in range(segment_w):
            if y>0:
                check_segments.append((cloth[x][y], cloth[x][y-1]))
            if x<=segment_w-2:
                check_segments.append((cloth[x][y], cloth[x+1][y]))
                if y < segment_h-1:
                    check_segments.append((cloth[x][y], cloth[x+1][y+1]))

    return cloth, weld_joints, distance_joints

def step_cloth(world, cloth, wind, body_size, segment_count, distance_joints, check_segments, broken,
        wind_dir=(1,1), wind_rand=0.0, distance_factor=distance_factor_param, **kwargs):
    segment_w, segment_h=segment_count
    body_spacing_w=body_size*2
    if wind:
        for x in range(segment_w):
            w=(b2Random(wind_dir[0]-wind_rand/2.0,wind_dir[0]+wind_rand/2.0), 
               b2Random(wind_dir[1]-wind_rand/2.0,wind_dir[1]+wind_rand/2.0)) 
            cloth[x][-1].linearVelocity+=w

    # If any two points are too far from one another, find the joint connecting them
    # and destroy it.
#    check_segments=[]
#    for y in range(segment_h):
#        for x in range(segment_w):
#            if y>0:
#                check_segments.append((cloth[x][y], cloth[x][y-1]))
#            if x<=segment_w-2:
#                check_segments.append((cloth[x][y], cloth[x+1][y]))

    for c1, c2 in check_segments:
        if (c1,c2) not in broken:
            if (c1.worldCenter-c2.worldCenter).length > body_spacing_w*distance_factor:
                for i,joint in enumerate(distance_joints):
                    if (joint.bodyA==c1 and joint.bodyB==c2) or (joint.bodyA==c2 and joint.bodyB==c1):
                        world.DestroyJoint(joint)
                        distance_joints.remove(joint)
                        broken.append((c1,c2))
                        if remove_neighbors:
                            try:
                                joint1 = distance_joints[i-1]
                                joint2 = distance_joints[i+1]
                                world.DestroyJoint(joint1)
                                distance_joints.remove(joint1)
                                world.DestroyJoint(joint2)
                                distance_joints.remove(joint2)
                            except:
                                pass
                            #broken.append((c1,c2))
                        break
    return broken
            
class Cloth(Framework):
    name = "Cloth"
    description="(w) Toggle wind"
    def __init__(self):
        super(Cloth, self).__init__()
        self.wind=False
        self.segment_count=segment_count
        self.body_size=body_size

        cloth_info=create_cloth(self.world, self.segment_count, self.body_size)
        self.cloth, self.weld_joints, self.dist_joints=cloth_info
        
        segment_w, segment_h=self.segment_count
        self.check_segments=[]
        self.broken = []
        for y in range(segment_h):
            for x in range(segment_w):
                if y>0:
                    self.check_segments.append((self.cloth[x][y], self.cloth[x][y-1]))
                if x<=segment_w-2:
                    self.check_segments.append((self.cloth[x][y], self.cloth[x+1][y]))

    def Keyboard(self, key):
        if key == Keys.K_w:
            self.wind=not self.wind
            
    def Step(self, settings):
       super(Cloth, self).Step(settings)
       if self.wind:
            self.Print('Wind enabled')
       self.broken = step_cloth(self.world, self.cloth, self.wind, self.body_size, 
            self.segment_count, self.dist_joints,self.check_segments, self.broken)
       #print len(self.broken)
            
if __name__=="__main__":
    main(Cloth)

