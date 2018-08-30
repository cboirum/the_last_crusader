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
import Assets
from matplotlib.path import Path
import matplotlib.patches as patches
#from shapely.geometry import LineString
from shapely.geometry import Polygon,Point
import triMesh
import matplotlib.pyplot as plt
from math import sin,cos
import shapely


remove_neighbors = True #remove joints next those that break (to eliminate stringy leftover joints)
distance_factor_param = 1.3
segment_count=(10,10)
body_size=.5

base_joint_frequency = 1000000000#15*100000
base_joint_damping = base_joint_frequency*10000#100000000.11
element_density = 1

on_ground = True
world = []

def create_cloth(world, segment_count, body_size, position=(0,30),
        groupIndex=1, bar_height=0.5, base_hz=base_joint_frequency, 
        base_damping=base_joint_damping, density=element_density, **kwargs):
    segment_w, segment_h=segment_count
    body_spacing_w=body_size*2
    total_w=body_spacing_w*segment_w
    total_h = body_spacing_w*segment_h
    if on_ground:
        position = (position[0],position[1])
        position=b2Vec2(*position)
    else:
        position=b2Vec2(*position)
    print("Position: %d,%d",position)
    # The static bar at the top which holds the cloth
    #bar=world.CreateStaticBody(position=position)
#    bar=world.CreateDynamicBody(position=position)
#    bar.CreatePolygonFixture(box=(total_w/2.0+body_spacing_w, bar_height/2.0), 
#            groupIndex=groupIndex)
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

#            if y==0:
#                joint=world.CreateWeldJoint(bodyA=body, bodyB=bar, anchor=body.position)
#                weld_joints.append(joint)
                
            if y==segment_h-1:
                joint=world.CreateWeldJoint(bodyA=body, bodyB=ground, anchor=body.position)
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
       
class TriMesh(Framework):
    name = "TriMesh"
    description="(w) Toggle wind"
    def __init__(self):
        super(TriMesh, self).__init__()
        self.wind=False
        self.segment_count=segment_count
        self.body_size=body_size
        self.density = 1
        self.friction = 1
        ground = self.world.CreateBody(
                    shapes=b2EdgeShape(vertices=[(-40,40),
                                                 (-4000,-10),
                                                 (4000,-10),
                                                 (40,40)]) 
                )
        import triMesh
        md = triMesh.makeMesh()
        mH = Assets.MeshHelper(self.world)
        print"enter Debug"
        """
        create dynamic circle bodies at vertex positions
        """
        makeVertCircles = False
        if makeVertCircles:
            for i in range(md.vertexCount):
                position = md.Vertices[i]
                radius = .1
                Assets.addCircle(self.world,position,radius)

        """
        Create all (inner) triangles for visualization with static edges
        """
        makeStaticEdges = False
        if makeStaticEdges:
            mH.makeStaticEdges(md)

        """
        Create all inner triangles as dynamic bodies
        """
        makeDynamicTriangles = True#False
        if makeDynamicTriangles:
            mH.makeDynamicTriangles(md)


    def Keyboard(self, key):
        if key == Keys.K_w:
            self.wind=not self.wind
            
    def Step(self, settings):
       super(TriMesh, self).Step(settings)
#       if self.wind:
#            self.Print('Wind enabled')
#       self.broken = step_cloth(self.world, self.cloth, self.wind, self.body_size, 
#            self.segment_count, self.dist_joints,self.check_segments, self.broken)
       #print len(self.broken)

class ConcaveDecomp(Framework):
    name = "Concave Decomposition Test"
    description=""
    def __init__(self):
        import triMesh
        super(ConcaveDecomp, self).__init__()
        self.segment_count=segment_count
        self.body_size=body_size
        self.density = 1
        self.friction = 1
        ground = self.world.CreateBody(
                    shapes=b2EdgeShape(vertices=[(-40,40),
                                                 (-4000,-10),
                                                 (4000,-10),
                                                 (40,40)]) 
                )
        
        """
        End of world initialization
        """
        #(0) create b2body
        position = (0,0)
        density = 1
        friction = 1
#        boxV = [(0,1),
#                    (1,1),
#                    (1,0),
#                    (0,0)]
        boxV = [(0,0),
                    (1,0),
                    (1,1),
                    (0,1),]
        boxS = [(1,2),
                   (2,3),
                   (3,4),
                   (4,1)]
        a = Polygon(boxV)
        xy = a.exterior.coords.xy
        boxV = [(xy[0][i],xy[1][i]) for i in range(len(xy[0])-1)]
        
        
        body=self.world.CreateDynamicBody(position=position)
        body.CreatePolygonFixture(vertices=boxV, density=density, friction=friction)
        
        POC = (0,.5)
        r = .25
        
        #self.takeBite(POC,r,body)
        Assets.takeBite(POC,r,body)

    def takeBite(self,POC,r,body):
        """
        Takes a bite out of a body, triangulates, etc. does not return anything.
        """
        #(1) get verts of b2body assuming it only has 1 fixture
        fixture = body.fixtures[0]
        boxV = fixture.shape.vertices
        density = fixture.density
        friction = fixture.friction
        
        
        #(2a) make box path from verts
        a = Polygon(boxV)
        #(2b) make circle path from
        #     Point of Contact (POC) 
        #     radius (r)
        #     number of points in path (N)
#        POC = (0,.5)
#        r = .25
        N = 20
        Dangle = (3.1415*2)/N
        Sangle = .1 #starting angle of points
        angles = [(Sangle + Dangle*i) for i in range(N)]
        x,y = POC
        circV = [(r*cos(angle)+x,r*sin(angle)+y) for angle in angles]
        b = Polygon(circV)
        
        #(3) take bite and get C's verts
        c = a.difference(b)
        xy = c.exterior.coords.xy
        cVerts = [(xy[0][i],xy[1][i]) for i in range(len(xy[0])-1)]

        #(4b) triangulate C into triangles
        md = triMesh.makeMyMesh(cVerts)

        #(5) replace input body's fixture with new one made of multiple triangles
        oldFixture = body.fixtures[0]
        body.DestroyFixture(oldFixture)
        for t in md.Triangles[0:md.triangleCount]:
            if t.inside:
                vertices = [(v.x,v.y) for v in t.v]
                x = [v.x for v in t.v]
                y = [v.y for v in t.v]
                testPoint = Point(sum(x)/3.0,sum(y)/3.0)
                if testPoint.within(c):
                    newTri=body.CreatePolygonFixture(vertices=vertices, density=density, friction=friction)

    def XY2Verts_Segs(self,coords):
        x,y = coords.xy
        verts = []*len(x)
        verts = [(x[i],y[i])for i in range(len(x))]
        
    def findCorner(self,SCVP,poly):
        """
        Finds the new corners from the geometry subtraction. 
        1. uses shapely
        """
        
        
        line1 = LineString(SCVP[0])
        line2 = LineString(SCVP[1])
        plt.plot(inX,inY)
        plt.plot(outX,outY)
        plt.show()
        
        #self.plotPatch(circPath)
    def findInsidePoints(self,ply1V,ply2V):
        """
        returns:
        inVerts = verts of poly2 that are inside poly1
        outVerts = verts oustide of poly1
        SCVP = sine change vertex pairs ordered, [outside,inside]
        """
        #circPath = self.makeClosedPath(circV)
        from  matplotlib.nxutils import points_inside_poly as pointTest
        
        inVerts = []
        outVerts = []
        SCVP = [[None,None],[None,None]] #Sine Change Point Pairs
        
        test = pointTest(ply2V,ply1V)
        #print test
        j = 0
        for i,result in enumerate(test):
            if result:
                inVerts.append(ply2V[i])
            else:
                outVerts.append(ply2V[i])
            if i<len(test)-1:
                if result != test[i+1]:
                    SCVP[j][0] = ply2V[i]
                    SCVP[j][1] = ply2V[i+1]
                    j+=1
        print inVerts,outVerts,SCVP
        return inVerts,outVerts,SCVP
        
    
    def makeClosedPath(self,verts):
        verts.append((0,0))
        codes = [Path.LINETO]*len(verts)
        codes[0] = Path.MOVETO
        codes[-1] = Path.CLOSEPOLY
        path = Path(verts,codes)
        return path
    
    def plotPatch(self,path):
        fig = plt.figure()
        ax = fig.add_subplot(111)
        patch = patches.PathPatch(path, facecolor='orange', lw=2)
        ax.add_patch(patch)
        ax.set_xlim(-2,2)
        ax.set_ylim(-2,2)
        plt.show()
        
    def makeALD(self,POC,r,N):
        """
        Creates a circular polygon of N vertices for the Area of Local Deformation (ALD)
        given a point of contact (POC), radius (r), and vertex count (N).
        """
        Dangle = (3.1415*2)/N
        Sangle = .1 #starting angle of points
        angles = [(Sangle + Dangle*i) for i in range(N)]
        x,y = POC
        verts = [(r*cos(angle)+x,r*sin(angle)+y) for angle in angles]
        segs = [(1+i,2+i) for i in range(N-1)]
        segs.append((N,1))
        #print segs
        return verts,segs
        
    def plotPolyPoints(self,verts,segs):
        x = [None]*(len(segs)*2)
        y = [None]*(len(segs)*2)
        i = 0
        for seg in segs:
            try:
                x[i],y[i] = verts[seg[0]-1]
                x[i+1],y[i+1] = verts[seg[1]-1]
                i+=2
            except:
                print"oops"
        return x,y

    def verts2XY(self,verts):
        x = [None] * len(verts)
        y = [None] * len(verts)
        for i,vert in enumerate(verts):
            x[i]=vert[0]
            y[i]=vert[1]
        return x,y
        
             
            
    def Step(self, settings):
       super(ConcaveDecomp, self).Step(settings)
        
if __name__=="__main__":
    #main(TriMesh)
    main(ConcaveDecomp)

