#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
An attempt at some simple, self-contained pygame-based examples.
Example 02

In short:
One static body:
    + One fixture: big polygon to represent the ground
Two dynamic bodies:
    + One fixture: a polygon
    + One fixture: a circle
And some drawing code that extends the shape classes.

kne
"""
import pygame
import math
from pygame.locals import *

import Box2D # The main library
from Box2D.b2 import * # This maps Box2D.b2Vec2 to vec2 (and so on)
import Box2D
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import numpy as np

# --- constants ---
# Box2D deals with meters, but we want to display pixels, 
# so define a conversion factor:
PPM=71.26 # pixels per meter
TARGET_FPS=60
TIME_STEP=1.0/TARGET_FPS
#SCREEN_WIDTH, SCREEN_HEIGHT=1920,1080
SCREEN_WIDTH, SCREEN_HEIGHT=640,480

# --- pygame setup ---
screen=pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock=pygame.time.Clock()

# --- Supporting Function Definitions -- 
class spritesheet(object):
    def __init__(self, filename):
        try:
            self.sheet = pygame.image.load(filename).convert()
        except pygame.error, message:
            print 'Unable to load spritesheet image:', filename
            raise SystemExit, message
    # Load a specific image from a specific rectangle
    def image_at(self, rectangle, colorkey = None):
        "Loads image from x,y,x+offset,y+offset"
        rect = pygame.Rect(rectangle)
        image = pygame.Surface(rect.size).convert()
        image.blit(self.sheet, (0, 0), rect)
        if colorkey is not None:
            if colorkey is -1:
                colorkey = image.get_at((0,0))
            image.set_colorkey(colorkey, pygame.RLEACCEL)
        return image
    # Load a whole bunch of images and return them as a list
    def images_at(self, rects, colorkey = None):
        "Loads multiple images, supply a list of coordinates" 
        return [self.image_at(rect, colorkey) for rect in rects]
    # Load a whole strip of images
    def load_strip(self, rect, image_count, colorkey = None):
        "Loads a strip of images and returns them as a list"
        tups = [(rect[0]+rect[2]*x, rect[1], rect[2], rect[3])
                for x in range(image_count)]
        return self.images_at(tups, colorkey)
    
class SpriteStripAnim(object):
    """sprite strip animator
    
    This class provides an iterator (iter() and next() methods), and a
    __add__() method for joining strips which comes in handy when a
    strip wraps to the next row.
    """
    def __init__(self, filename, rect, count, colorkey=None, loop=False, frames=1):
        """construct a SpriteStripAnim
        
        filename, rect, count, and colorkey are the same arguments used
        by spritesheet.load_strip.
        
        loop is a boolean that, when True, causes the next() method to
        loop. If False, the terminal case raises StopIteration.
        
        frames is the number of ticks to return the same image before
        the iterator advances to the next image.
        """
        colorkey=Color(96,128,192)
        #colorkey = image.get_at((0,0))
        colorkey = -1
        self.filename = filename
        #ss = spritesheet.spritesheet(filename)
        ss = spritesheet(filename)
        self.images = ss.load_strip(rect, count, colorkey)
        self.i = 0
        self.loop = loop
        self.frames = frames
        self.f = frames
    def iter(self):
        self.i = 0
        self.f = self.frames
        return self
    def next(self):
        if self.i >= len(self.images):
            if not self.loop:
                raise StopIteration
            else:
                self.i = 0
        image = self.images[self.i]
        self.f -= 1
        if self.f == 0:
            self.i += 1
            self.f = self.frames
        return image
    def __add__(self, ss):
        self.images.extend(ss.images)
        return self
    
def getSpriteStrips():
    import sys
    from pygame.locals import Color, KEYUP, K_ESCAPE, K_RETURN
    topLeft = [113+1,72+1]
    botRight = [145,96]
    botLeft = [113,96]
    topRight = [143,72]
    width,height = botRight[0] - topLeft[0],botRight[1] - topLeft[1]
    x5 = width
    y5 = height
    frames = 12
    
    sizes = ([36,43],
             [122,82],
             [122,93],
             [122,102],
             [x5,y5],
             [258,535]
             )
    x1,y1 = sizes[0]
    x2,y2 = sizes[1]
    x3,y3 = sizes[2]
    x4,y4 = sizes[3]
    x6,y6 = sizes[5]
    strips = [
            
    SpriteStripAnim('data\PlayerExplode.bmp', (0,1,x1,y1), 7, 1, True, frames),
    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,4,x2,y2), 4, 1, True, frames),
    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,88,x3,y3), 4, 1, True, frames),
    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,184,x4,y4), 4, 1, True, frames),
    SpriteStripAnim('data\LandAircraftCareer.bmp', (topLeft[0],topLeft[1],x5,y5), 5, 1, True, frames),
    SpriteStripAnim('data\F-8 Left.png', (0,1,789,259), 1, 1, True, frames),  
    SpriteStripAnim('data\F-8 Left XX.png',(0,1,1174,328), 1, 1, True, frames),
    ]
    return strips

# --- pybox2d world setup ---
# Create the world
#world=world(gravity=(0,-10),doSleep=True)
world=world(gravity=(0,-10),doSleep=True)

# And a static body to hold the ground shape
ground_body=world.CreateStaticBody(
    position=(0,0),
    shapes=polygonShape(box=(50,1)),
    )

# Create a couple dynamic bodies
body=world.CreateDynamicBody(position=(20,45))
circle=body.CreateCircleFixture(radius=0.5, density=1, friction=0.3)

# --- make box correct aspect ratio for sprite of F-8 --- 
boxL_halfm = 2
boxL = boxL_halfm
boxH_halfm = boxL*328/1174.0
boxH = boxH_halfm

#boxL_halfm = 2
#boxL = boxL_halfm
#boxH_halfm = 1
#boxH = boxH_halfm

boxL_pix = boxL_halfm*2.0*PPM
boxH_pix = boxH_halfm*2.0*PPM

body=world.CreateDynamicBody(position=(14,10), angle=0)
box=body.CreatePolygonFixture(box=(boxL,boxH), density=1, friction=0.3)

body=world.CreateDynamicBody(position=(15,10), angle=0)
box2 = body.CreatePolygonFixture(box=(boxL,boxH), density=1, friction=0.3)

cV = [          #(0,0),
                (28,5),
                (92,2),
                (313,161),
                (520,174),
                (957,154),
                (1167,222),
                (1110,250),
                (1108,280),
                (377,317),
                (65,291),
                (61,220),
                (103,209),
                (148,208),
                (150,295),
                (1106,202),
                (753,167),
                (759,298),
                (956,291),

                ]

points = np.array(cV)/PPM
parts = [0]*6
#parts = {}
#parts{'tail'} = 
vertLists = [[1,2,3,12,13],
            [10,11,13,14],
            [3,9,14,13,4],
            [9,4,16,17],
            [16,5,18,17],
            [5,15,7,8,18],
            [15,6,7],
            ]

#p1=plt.plot(points[:,0],points[:,1],'o')
#plt.hold(True)
#ps = []*len(parts)
##allTris = []
##nAllTris = []
##for i in range(len(parts)):
##    tri = Delaunay(points[np.array(vertList[i])-1])
##    tris = [0]*tri.vertices.shape[0]
##    ntris = [0]*tri.vertices.shape[0]
##    for j in range(tri.vertices.shape[0]):
##        tris[j] = points[tri.vertices[j]].tolist()
##        ntris[j] = points[tri.vertices[j]]
##        plt.plot(ntris[j][:,0],ntris[j][:,1])
##
##    allTris.extend(tris)
##    nAllTris.extend(ntris)
##    parts[i] = tris
##    plt.show()
##        
##allTris = allTris[1:]
import  matplotlib.tri as tri
allTris = []
nAllTris = []
for vertList in vertLists:
    partPoints = points[np.array(vertList)-1]
    x = partPoints[:,0]
    y = partPoints[:,1]
    triang = tri.Triangulation(x, y)
    plt.triplot(triang, 'bo-')
    for triangle in triang.triangles:
        allTris.append(partPoints[triangle].tolist())
plt.show()
    
xx=1
#x = points[:,0]
#y = points[:,1]
#triang = tri.Triangulation(x, y)
#p1=plt.triplot(triang, 'bo-')
#plt.gca().set_aspect('equal')
#plt.show()

#min_radius = 0.25
#xmid = x[triang.triangles].mean(axis=1)
#ymid = y[triang.triangles].mean(axis=1)
#mask = np.where(xmid*xmid + ymid*ymid < min_radius*min_radius, 1, 0)
#triang.set_mask(mask)
#p1=plt.triplot(triang, 'bo-')
#plt.gca().set_aspect('equal')
#plt.show()
    
    
#    tri = Delaunay(partPoints)
#    for verts in tri.vertices:
#        plot.triplot
#        plt.plot(points[verts])
        
#    tris = [0]*tri.vertices.shape[0]
#    ntris = [0]*tri.vertices.shape[0]
#    for j in range(tri.vertices.shape[0]):
#        tris[j] = points[tri.vertices[j]].tolist()
#        ntris[j] = points[tri.vertices[j]]
#        plt.plot(ntris[j][:,0],ntris[j][:,1])

#    allTris.extend(tris)
#    nAllTris.extend(ntris)
#    parts[i] = tris
#    plt.show()
        
allTris = allTris[1:]

piHead = (984/PPM,171/PPM)
piHr = 8/PPM
piBody = (985/PPM,199/PPM)
piBr = 16/PPM

body=world.CreateDynamicBody(position=(5,5))
pilotHead=body.CreateCircleFixture(pos=piHead,radius=piHr, density=1, friction=0.3)
pilotBody=body.CreateCircleFixture(pos=piBody,radius=piBr, density=1, friction=0.3)
for triangle in allTris:
    newTri=body.CreatePolygonFixture(vertices=triangle, density=1, friction=0.3)
#part1 = body.CreatePolygonFixture(b2PolygonShape(vertices=))


#pilotHead = world.CreateDynamicBody(
#                    position=(10, 10), 
#                    angle=pi,
#                    angularDamping=5,
#                    linearDamping=0.1,
#                    #shapes=[Box2D.b2PolygonShape(vertices=x) for x in allTris],
#                    shapes=[Box2D.b2PolygonShape(vertices=parts[0][0]),
#                            Box2D.b2PolygonShape(vertices=parts[0][1])],
#                            #Box2D.b2PolygonShape(vertices=restm) ],
#                    shapeFixture=Box2D.b2FixtureDef(density=20.0),
#                    )
#body = world.CreateDynamicBody(position=(10, 10),angle=pi,angularDamping=5,linearDamping=0.1,shapes=[Box2D.b2PolygonShape(vertices=parts[0][0])],shapeFixture=Box2D.b2FixtureDef(density=20.0))

colors = {
    staticBody  : (255,255,255,255),
    dynamicBody : (127,127,127,255),
}
strips = getSpriteStrips()

spriteStrips = {
    staticBody  : strips[1],
    dynamicBody : strips[6]
}


# Let's play with extending the shape classes to draw for us.
def my_draw_polygon(polygon, body, fixture):
    body.angularVelocity = .5
    vertices=[(body.transform*v)*PPM for v in polygon.vertices]
    vertices=[(v[0], SCREEN_HEIGHT-v[1]) for v in vertices]
    #if body.userData and 'box' in body.userData:
    baseImage = spriteStrips[body.type].next()
    (x,y)= body.worldCenter*PPM
    im = baseImage.get_rect()
    sX = boxL_pix
    sY = boxH_pix
    print sX,sY
    #print x,y
    image = pygame.transform.scale(baseImage,(sX,sY))
    image = pygame.transform.rotate(image, math.degrees(body.angle))
    
    #image = pygame.transform.rotozoom(baseImage, math.degrees(body.angle),.5)
    rect = image.get_rect()
    sprX, sprY = (x-(rect.width/2),-(rect.height/2)-y+SCREEN_HEIGHT)
    print ("sprite:" +str(sprX) +" " + str(sprY))
    print ("body  :" + str(x) + " " + str(y))
    print ("Diff  :" + str(x-sprX) + " " + str(y-sprY))
    pygame.draw.polygon(screen, colors[body.type], vertices)
    screen.blit(image, (sprX,sprY))
    
    
polygonShape.draw=my_draw_polygon
    

def my_draw_circle(circle, body, fixture):
    position=body.transform*circle.pos*PPM
    position=(position[0], SCREEN_HEIGHT-position[1])
    pygame.draw.circle(screen, colors[body.type], [int(x) for x in position], int(circle.radius*PPM))
    # Note: Python 3.x will enforce that pygame get the integers it requests,
    #       and it will not convert from float.
circleShape.draw=my_draw_circle


    
def SpriteSheetExample():
    import sys
    from pygame.locals import Color, KEYUP, K_ESCAPE, K_RETURN
     
    #surface = pygame.display.set_mode((350,400))
    #surface = pygame.display.set_mode((800,600))
    surface = screen
    FPS = 12
    frames = FPS / 12

    topLeft = [113+1,72+1]
    botRight = [145,96]
    botLeft = [113,96]
    topRight = [143,72]
    width,height = botRight[0] - topLeft[0],botRight[1] - topLeft[1]
    x5 = width
    y5 = height
    
    sizes = ([36,43],
             [122,82],
             [122,93],
             [122,102],
             [x5,y5],
             [258,535]
             )
    x1,y1 = sizes[0]
    x2,y2 = sizes[1]
    x3,y3 = sizes[2]
    x4,y4 = sizes[3]
    x6,y6 = sizes[5]
    #Measure corners of bounding box in MS paint (pixel just AFTER las pixel of sprite)
    
    strips = [
            
    SpriteStripAnim('data\PlayerExplode.bmp', (0,1,x1,y1), 7, 1, True, frames),
    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,4,x2,y2), 4, 1, True, frames),
    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,88,x3,y3), 4, 1, True, frames),
    SpriteStripAnim('data\MLRS_Chassis.bmp', (9,184,x4,y4), 4, 1, True, frames),
    SpriteStripAnim('data\LandAircraftCareer.bmp', (topLeft[0],topLeft[1],x5,y5), 5, 1, True, frames),
    SpriteStripAnim('data\F-8 Left.png', (0,1,789,259), 1, 1, True, frames),  
    SpriteStripAnim('data\F-8 LeftXX.png',(0,1,1174,328), 1, 1, True, frames),
    ]
    black = Color('black')
    clock = pygame.time.Clock()
    n = 0
    strips[n].iter()
    image = strips[n].next()
    aka = 1
    while True:
        for e in pygame.event.get():
            if e.type == KEYUP:
                if e.key == K_ESCAPE:
                    sys.exit()
                elif e.key == K_RETURN:
                    n += 1
                    aka = 1
                    if n >= len(strips):
                        n = 0
                    strips[n].iter()
        
        
        #surface.fill(black)
        xSz,ySz = sizes[n]
        surface.blit(image, (0,0))
        #pygame.display.flip()
        image = strips[n].next()
        #image = pygame.transform.scale(image,(xSz*3,ySz*3))
        clock.tick(FPS)

# --- main game loop ---

running=True
while running:
    # Check the event queue
    for event in pygame.event.get():
        if event.type==QUIT or (event.type==KEYDOWN and event.key==K_ESCAPE):
            # The user closed the window or pressed escape
            running=False

    screen.fill((0,0,0,0))
    # Draw the world
    for body in world.bodies:
        for fixture in body.fixtures:
            fixture.shape.draw(body, fixture)

    # Make Box2D simulate the physics of our world for one step.
    world.Step(TIME_STEP, 10, 10)

    # Flip the screen and try to keep at the target FPS
    pygame.display.flip()
    clock.tick(TARGET_FPS)

pygame.quit()
print('Done!')
