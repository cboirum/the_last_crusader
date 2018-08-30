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
import Assets

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

#body=world.CreateDynamicBody(position=(14,10), angle=0)
#box=body.CreatePolygonFixture(box=(boxL,boxH), density=1, friction=0.3)
#
#body=world.CreateDynamicBody(position=(15,10), angle=0)
#box2 = body.CreatePolygonFixture(box=(boxL,boxH), density=1, friction=0.3)


crusaderBase = Assets.Crusader2(PPM)
crusader1 = crusaderBase.add2World(world,(5,5))
crusader2 = crusaderBase.add2World(world,(1,10))


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
    if body == crusader1:
        pass
    else:
        baseImage = pygame.transform.scale(baseImage,(sX,sY))
    image = pygame.transform.rotate(baseImage, math.degrees(body.angle))
    rect = image.get_rect()
    sprX, sprY = (x-(rect.width/2)+71.26,-(rect.height/2)-y+SCREEN_HEIGHT-45)
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
