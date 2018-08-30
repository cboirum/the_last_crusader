'''
Created on Jul 1, 2014

@author: BOIRUM

Various utilities for game graphics, such as animated sprites
'''

import pygame
import sys
from pygame.locals import Color, KEYUP, K_ESCAPE, K_RETURN, K_UP, K_LEFT, K_RIGHT
from matplotlib import tri
from numpy import array
import matplotlib.pyplot as plt

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
    """
    Special function for generating specific sheet of sprites, here for example
    """
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

def spriteBox(bounds):
    """
    formats bounds for sprite strip manipulation
    """
    #bounds = [leftX, topY, rightX, bottomY]
    spacing = [5,0]
    topLeftX = bounds[0]
    topLeftY = bounds[1]
    width = bounds[2]-bounds[0]
    height = bounds[3]-bounds[1]
    box = [topLeftX,topLeftY,width,height]
    return box

def SpriteTester():
    """
    Tests sprite offsets
    """
    
     
    #surface = pygame.display.set_mode((350,400))
    surface = pygame.display.set_mode((800,600))
    FPS = 12
    frames = FPS / 12

    #bounds = [leftX, topY, rightX, bottomY] = pixel bounds of object
    CrusaderLeftBig = [14,4,114,34]
    CrusaderRightBig = [120,4,219,34]
    CrusaderLeftNormalBankR = [13,37,46,49]
    CrusaderLeftNormalLevel = [47,37,80,49]
    CrusaderLeftNormalBankL = [82,37,115,49]
    #spacing = [horizontal, vertical] = pixel space between objects
#    spacing = [5,0]
#    topLeftX = bounds[0]
#    topLeftY = bounds[1]
#    width = bounds[2]-bounds[0]
#    height = bounds[3]-bounds[1]
#    box = [topLeftX,topLeftY,width,height]

    filePath = 'data\Allied_Fighters.gif'
    stateList = [CrusaderLeftBig,
                 CrusaderRightBig,
                 CrusaderLeftNormalBankR,
                 CrusaderLeftNormalLevel,
                 CrusaderLeftNormalBankL
                 ]
   # strips = [SpriteStripAnim(filePath,spriteBox(x),1,1,True,frames) for x in stateList]
    strips = [SpriteStripAnim(filePath,spriteBox(x),1,1,True,frames) for x in stateList]
#    strips = [
#            
#    SpriteStripAnim('data\Allied_Fighters.gif', spriteBox(CrusaderLeftBig), 1, 1, True, frames),  
#    SpriteStripAnim('data\Allied_Fighters.gif', spriteBox(CrusaderRightBig), 1, 1, True, frames),
#    SpriteStripAnim('data\Allied_Fighters.gif', spriteBox(CrusaderLeftNormalBankR), 1, 1, True, frames),
#    SpriteStripAnim('data\Allied_Fighters.gif', spriteBox(CrusaderLeftNormalBankR), 1, 1, True, frames),
#    ]
    black = Color('black')
    white = Color('white')
    backColor = black
    clock = pygame.time.Clock()
    n = 0
    strips[n].iter()
    image = strips[n].next()
    while True:
        for e in pygame.event.get():
            if e.type == KEYUP:
                if e.key == K_ESCAPE:
                    sys.exit()
                elif e.key == K_RIGHT:
                    print"righted"
                    n += 1
                    if n >= len(strips):
                        n = 0
                    print(n)
                    strips[n].iter()
                elif e.key == K_LEFT:
                    print"lefted"
                    n -= 1
                    if n < 0:
                        n = len(strips)-1
                    print(n)
                    strips[n].iter()
                elif e.key == K_UP:
                    print "upped"
                    if backColor == white:
                        backColor = black
                    else:
                        backColor = white
        
        
        surface.fill(backColor)
        #image = pygame.surface.scale(image())
        surface.blit(image, (0,0))
        pygame.display.flip()
        image = strips[n].next()
        #image = pygame.transform.scale(image,(xSz*3,ySz*3))
        clock.tick(FPS)

def decompAndScale(verts,vertLists,PPM, debug=False):
    """
    Uses delaunay triangulation to decompose the mesh into triangles, and 
    scales the coordinates based on PPM (Pixels Per Meter). VertList allows
    concave shapes to be specified as convex ones, otherwise a convex hull 
    is created and the mesh returned. 
    
    Returns "allTris" which is an N x 3 x 2 list of triangle vertex pairs
    in x and y.
    
    verts = list of (x,y) pairs
    vertLists = list of vertex indecies that define a shape
    PPM = Pixels Per Meter for use with Box2D
    debug = optional flag to display a plot of the mesh
    """
    points = array(verts)/PPM
    allTris = []
    for vertList in vertLists:
        partPoints = points[array(vertList)-1]
        x = partPoints[:,0]
        y = partPoints[:,1]
        triang = tri.Triangulation(x, y)
        if debug:
            plt.triplot(triang, 'bo-')
        for triangle in triang.triangles:
            allTris.append(partPoints[triangle].tolist())
    if debug:
        plt.show()
    return (allTris)

if __name__ == "__main__":
    SpriteTester()