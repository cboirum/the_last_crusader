'''
Created on Jun 30, 2014

@author: BOIRUM
'''

import Box2D
import pygame



#screen height and width
sW = 640
sH = 480



PPM=20.0 # pixels per meter
TARGET_FPS=60
TIME_STEP=1.0/TARGET_FPS
SCREEN_WIDTH, SCREEN_HEIGHT=640,480

screen=pygame.display.set_mode((SCREEN_WIDTH,SCREEN_HEIGHT), 0, 32)
pygame.display.set_caption('Simple pygame example')
clock=pygame.time.Clock()

world = world(gravity=(0,-10),doSleep=True)