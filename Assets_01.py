'''
Created on Jul 1, 2014

@author: BOIRUM
Storage file for defining game assets for Crusader
'''
from matplotlib import tri
from numpy import array
import matplotlib.pyplot as plt
import spriteUtils
from scipy import interpolate
import pygame
import math

class triMesh(object):
    verticies = []
    polygons = []
    sprites = []
    scaledVerts = []
    triangles = []
    player = []
    
    def __init__(self, PPM):
        self.PPM = PPM
    
    def decompAndScale(self,debug=False):
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
        points = array(self.verticies)/self.PPM
        allTris = []
        for vertList in self.polygons:
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
        self.triangles = allTris
        self.scaledVerts = points
        
    def add2World(self,world,position):
        body=world.CreateDynamicBody(position=position)
        for triangle in self.triangles:
            newTri=body.CreatePolygonFixture(vertices=triangle, density=5, friction=0.3)
        if self.player:
            self.makePlayer(body)
        return body
    
class Crusader(triMesh):
    
    verticies = [
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

    polygons = [[1,2,3,12,13],
                [10,11,13,14],
                [3,9,14,13,4],
                [9,4,16,17],
                [16,5,18,17],
                [5,15,7,8,18],
                [15,6,7],
                ]
    player = [[[984,171],8],
              [[985,199],16],
              ]
    
    
    def __init__(self,PPM):
        triMesh.__init__(self,PPM)
        self.decompAndScale()
        
    
        
    def makePlayer(self,body):
        PPM = self.PPM
        piHead = (self.player[0][0][0]/PPM,self.player[0][0][1]/PPM)
        piHr = self.player[0][1]/PPM
        piBody = (self.player[1][0][0]/PPM,self.player[1][0][1]/PPM)
        piBr = self.player[1][1]/PPM
#        piHead = (984/PPM,171/PPM)
#        piHr = 8/PPM
#        piBody = (985/PPM,199/PPM)
#        piBr = 16/PPM
        self.pilotHead=body.CreateCircleFixture(pos=piHead,radius=piHr, density=1, friction=0.3)
        self.pilotBody=body.CreateCircleFixture(pos=piBody,radius=piBr, density=1, friction=0.3)
        
class Crusader2(Crusader):
    
    def __init__(self,PPM):
        self.verticies = [
                (28,-5),
                (92,-2),
                (313,-161),
                (520,-174),
                (957,-154),
                (1167,-222),
                (1110,-250),
                (1108,-280),
                (377,-317),
                (65,-291),
                (61,-220),
                (103,-209),
                (148,-208),
                (150,-295),
                (1106,-202),
                (753,-167),
                (759,-298),
                (956,-291),
                ]
        Crusader.__init__(self,PPM)
        
class airplane(object):
    """
    Contains Graphics and state infromation
    """
    stateTree =   {'left': {
                          'Big': [],
                          'normal': {'bankR': [],
                                   'level': [],
                                   'bankL': [],
                                  },
                          'damaged': {'bankR': [],
                                   'level': [],
                                   'bankL': [],
                                  }
                          },
                   'right': {
                           'Big': [],
                           'normal': {'bankR': [],
                                    'level': [],
                                    'bankL': [],
                                   },
                           'damaged': {'bankR': [],
                                    'level': [],
                                    'bankL': [],
                                   }
                           }
                   }
    bank = 'level' #can be 'level', 'bankR', or 'bankL'
    damaged = 'normal' #can be 'normal', 'damaged'
    spritestrip = []
    
    def __init__(self):
        pass
    
class AlliedFighter(airplane):
    offsets = {'F-8 Crusader'  : [],
               'F-14 Tomcat'   : [],
               'F-2 Tigershark': [],
               'A-10 Warthog'  : [],
               'YF-23'         : [],
               'Efrit'         : []
               }
    
    def __init__(self,planeType):
        airplane.__init__(self)
        x1,y1 = self.offsets[planeType]
        self.bigSprite = spriteUtils.SpriteStripAnim('data\AlliedFighters.gif', (0,1,x1,y1), 7, 1, True, frames),
    
    def loadSprites(self):
        pass
    
VERTEX_COUNT=50
def get_ground_vertices(x1, vertices):
    y1=2.0*cos(x1/10.0*pi)
    for i in range(vertices):
        x2=x1+0.5
        y2=2.0*cos(x2/10.0*pi)
        yield (x1,y1), (x2,y2)
        x1, y1=x2, y2

def terrainTester(world):
    import scipy
    """
    Tests functionality for 
    1. load heightmap
    2. point and click choose path vectors
    1. 2d interpolation of a heightmap along a path
    2. resampling into heightlist
    5. creation of ground bodies for box2d
    """
    hmFileName = 'data\kaguya_lalt_500.jpg'
    hmFileName = "data\moon2.jpg"
    """
    moon2.jpg scale = 0.33047 km/pixel
    """
    hMap = pygame.image.load(hmFileName)
    hMap = pygame.surfarray.array2d(hMap)
    #print dir(hMap)
    #print hMap[0]
    #print hMap.shape
    v2 = hMap.shape
    vector = [[0,0],[v2[0]-1,v2[1]-1]]
    xS = vector[0][0]
    yS = vector[0][1]
    xE = vector[1][0]
    yE = vector[1][1]
    x = scipy.linspace(xS, xE, xE+1)
    y = scipy.linspace(yS, yE, yE+1)
    z = hMap
    middle = math.floor(xE/2)
    dX = .5
    numSamples = scipy.floor((xE-xS)/dX)
    x2 = scipy.linspace(xS, xE, numSamples)
    y2 = x2
    #interper = interpolate.interp2d(x,y,z)
    interper = interpolate.RectBivariateSpline(x,y,z)
    zNew = interper(x2,y2)
    middle2 = int(math.floor(zNew.shape[0]/2))
#    print zNew.shape
##    print dir(zNew)
##    tt2 = array(zNew)
#    
#    plt.plot(hMap[middle])
#    plt.show()
#    plt.plot(zNew[middle2][1:]/2)
#    plt.show()
    #interpolate.interp2d(x, y, z)
    #return ground
    hMapVscaleFactor = 1/100000.0
    hMapVoffset = 30
    numGrndVrts = 100
    
    nGY = array(zNew[middle2][1:numGrndVrts])
    print nGY[0]
    nGY = nGY*hMapVscaleFactor-hMapVoffset
    nGX = scipy.linspace(1,numGrndVrts)
    groundVrts = array([nGY,nGX])
    print nGY[0]
    #firstEdge = b2EdgeShape(vertices=[[nGX[0],nGY[0]],[nGX[1],nGY[1]],[nGX[2],nGY[2]]])
#    lastEdge = b2EdgeShape(vertices=[[nGX[end-2],nGY[end-2]],[nGX[end-1],nGY[end-1]],[nGX[end],nGY[end]]])
#    middleEdges = b2EdgeShape(vertices=[[nGX[x-1],nGY[x-1]],[nGX[x],nGY[x]],[nGX[x+1],nGY[x+1]],[nGX[x+2],nGY[x+2]]])
    from Box2D import b2EdgeShape
    middleEdges = [b2EdgeShape(vertices=[(nGX[x],nGY[x]),(nGX[x+1],nGY[x+1])])for x in range(nGX.size-1)]#,[nGX[x+1],nGY[x+1]],[nGX[x+2],nGY[x+2]]])
    #print middleEdges.shape
    if world:
        newGround = world.CreateStaticBody(
                                           shapes=[#firstEdge,
                                                   middleEdges,
                                                   #lastEdge,
                                                   ]
                                           )

if __name__ == "__main__":
    from Box2D import b2EdgeShape
    terrainTester(world=False)