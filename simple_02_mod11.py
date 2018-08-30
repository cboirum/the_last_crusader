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
import spriteUtils

# --- constants ---
debug = False #printing & plotting of stuff
# Box2D deals with meters, but we want to display pixels, 
# so define a conversion factor:
PPM=71.26 # pixels per meter
TARGET_FPS=60
TIME_STEP=1.0/TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT=1920,1080
#SCREEN_WIDTH, SCREEN_HEIGHT=640,480

# --- pygame setup ---
screen=pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock=pygame.time.Clock()

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

vertLists = [[1,2,3,12,13],
            [10,11,13,14],
            [3,9,14,13,4],
            [9,4,16,17],
            [16,5,18,17],
            [5,15,7,8,18],
            [15,6,7],
            ]

#points = np.array(cV)/PPM
#parts = [0]*6
#
#import  matplotlib.tri as tri
#allTris = []
#nAllTris = []
#for vertList in vertLists:
#    partPoints = points[np.array(vertList)-1]
#    x = partPoints[:,0]
#    y = partPoints[:,1]
#    triang = tri.Triangulation(x, y)
#    if debug:
#        plt.triplot(triang, 'bo-')
#    for triangle in triang.triangles:
#        allTris.append(partPoints[triangle].tolist())
#if debug:
#    plt.show()


#allTris = allTris[1:]

piHead = (984/PPM,171/PPM)
piHead = (984/PPM,171/PPM)
piHr = 8/PPM
piBody = (985/PPM,199/PPM)
piBr = 16/PPM
print(piHead)

body=world.CreateDynamicBody(position=(5,5))
pilotHead=body.CreateCircleFixture(pos=piHead,radius=piHr, density=1, friction=0.3)
pilotBody=body.CreateCircleFixture(pos=piBody,radius=piBr, density=1, friction=0.3)

allTris = spriteUtils.decompAndScale(cV,vertLists,PPM)

for triangle in allTris:
    newTri=body.CreatePolygonFixture(vertices=triangle, density=1, friction=0.3)

colors = {
    staticBody  : (255,255,255,255),
    dynamicBody : (127,127,127,255),
}
strips = spriteUtils.getSpriteStrips()

spriteStrips = {
    staticBody  : strips[1],
    dynamicBody : strips[6]
}

def draw_Sprite_on_Body(body):
    #Draws an animated sprite onto a body (Need a better way 
    #to assign sprite to body)
    baseImage = spriteStrips[body.type].next()
    (x,y)= body.worldCenter*PPM
    im = baseImage.get_rect()
    sX = boxL_pix
    sY = boxH_pix
    if debug:
        print sX,sY
        print ("sprite:" +str(sprX) +" " + str(sprY))
        print ("body  :" + str(x) + " " + str(y))
        print ("Diff  :" + str(x-sprX) + " " + str(y-sprY))
    image = pygame.transform.scale(baseImage,(sX,sY))
    image = pygame.transform.rotate(image, math.degrees(body.angle))
    rect = image.get_rect()
    sprX, sprY = (x-(rect.width/2),-(rect.height/2)-y+SCREEN_HEIGHT)
    screen.blit(image, (sprX,sprY))

# Let's play with extending the shape classes to draw for us.
def my_draw_polygon(polygon, body, fixture):
    #body.angularVelocity = .5
    vertices=[(body.transform*v)*PPM for v in polygon.vertices]
    vertices=[(v[0], SCREEN_HEIGHT-v[1]) for v in vertices]
    pygame.draw.polygon(screen, colors[body.type], vertices,1)
    draw_Sprite_on_Body(body)
    #print("Drawing a polygon")
    
    
polygonShape.draw=my_draw_polygon
    

def my_draw_circle(circle, body, fixture):
    position=body.transform*circle.pos*PPM
    position=(position[0], SCREEN_HEIGHT-position[1])
    #print("drawing a circle")
    #thisColor = colors[body.type]
    thisColor = (255,0,0,255)
    pygame.draw.circle(screen, thisColor, [int(x) for x in position], int(circle.radius*PPM),1)
    # Note: Python 3.x will enforce that pygame get the integers it requests,
    #       and it will not convert from float.
    
circleShape.draw=my_draw_circle

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
