'''
Created on Aug 14, 2014

@author: BOIRUM

Test for circle packing within circle algorithms for particle system creation within "bitten" geometry.
'''
import Box2D
import matplotlib.pyplot as plt
from descartes import PolygonPatch
from shapely.geometry import MultiPolygon,Polygon,MultiPoint,Point
from shapely import affinity
import math

circleResolution = 4
edgeLambda = math.sin(math.pi/(circleResolution*4))
print edgeLambda

def drawCircle(ax,r,x,y,res=circleResolution,alpha = 1):
    """
    Draws a circle on an axis
    """
    circ = Point(x,y).buffer(r,res)
    patch = PolygonPatch(circ,alpha = alpha)
    ax.add_patch(patch)
    return circ
    
def drawPoint(ax,x,y):
    drawCircle(ax,.05,x,y,alpha = .5)

def drawPoints(ax,xy):
    for x,y in xy:
        drawPoint(ax,x,y)
    
def plot_coords(ax, ob):
    x, y = ob.xy
    ax.plot(x, y, 'o', color='#999999', zorder=1)

def getEdgeLengths(r):
    return 2*r*edgeLambda

def drawCirlces():
    
    radius = 1
    edgeLengths = getEdgeLengths(radius)
    edgeR = edgeLengths/2
    
    
    
    circ1 = drawCircle(ax,1,0,0)
    print circ1.exterior.coords.xy
    for x,y in zip(*circ1.exterior.coords.xy):
        drawPoint(ax,x,y)
        drawCircle(ax,edgeR,x,y)
    
    newR = radius - 2*edgeR
    newEdgeR = getEdgeLengths(newR)/2
    newC = drawCircle(ax,newR,0,0)

def drawPoly(ax,xy):
    #print 
#    radius = 1
#    numEdges = len(xy[0])
#    edgeLengths = 2*radius*math.sin(math.pi/(numEdges))
#    edgeR = edgeLengths/2
    poly = Polygon(xy)#[Point(x,y) for x,y in xy])
    patch = PolygonPatch(poly, alpha = .5)
    ax.add_patch(patch)
#    for x,y in xy:
#        #drawPoint(ax,x,y)
#        drawCircle(ax,edgeR,x,y)

def drawPatch(ax,poly,alpha = .5):
    patch = PolygonPatch(poly, alpha = alpha)
    ax.add_patch(patch)

def drawHex(ax,r,x0,y0):
    numEdges = 6
    Dangle = 2*math.pi/numEdges
    Sangle = 0
    edgeR = r*math.sin(math.pi/(numEdges))
    xs = []
    ys = []
    for i in range(6):
        Sangle += Dangle
        x = r*math.cos(Sangle)+x0
        y = r*math.sin(Sangle)+y0
        xs.append(x)
        ys.append(y)
        dd = drawCircle(ax,edgeR,x,y)
    dd = drawCircle(ax,edgeR,x0,y0)
    xy = zip(xs,ys)
    drawPoly(ax,xy)
    
def getHexCoords(r,x0,y0):
    """
    Returns x,y coordinates of a single hexagon
    """
    numEdges = 6
    Dangle = 2*math.pi/numEdges
    Sangle = 0
    edgeR = r*math.sin(math.pi/(numEdges))
    xs = []
    ys = []
    for i in range(6):
        Sangle += Dangle
        x = r*math.cos(Sangle)+x0
        y = r*math.sin(Sangle)+y0
        xs.append(x)
        ys.append(y)
        #drawPoint(ax,x,y)
        #plt.show()
    xs.append(x0)
    ys.append(y0)
    return xs,ys

def getHexGrid(r,W,H):
    """
    Returns x,y coord of a hexagonal grid centered at 0,0 that spans WxH.
    W = width
    H = height
    """
    cols = W/(r*2)+1
    hexHeight = r*math.sin(math.pi/3)
    rows = H/(hexHeight*2)+1
    xy = []
    for col in range(cols):
        x0 = (2*r*col)
        for row in range(rows):
            x0=x0+r/2*(row%2)
            y0 = 2*hexHeight*row
            x,y = getHexCoords(r,x0,y0)
            newxy = zip(x,y)
            xy.extend(newxy)
    #print xy
    return xy

def getHexGrid2(r,W,H):
    """
    Returns x,y coord of a hexagonal grid centered at 0,0 that spans WxH.
    W = width
    H = height
    """
    cols = W/(r)+1
    hexHeight = r*math.sin(math.pi/3)
    rows = H/(hexHeight)+1
    xy = []
    for col in range(cols):
        x0 = (r*col)
        for row in range(rows):
            x00=x0+r/2.0*(row%2)
            y0 = hexHeight*row
            newxy = (x00,y0)
            xy.append(newxy)
    return xy

def makeCircles(r,xy,res = 4):
    """
    Returns a MultiPoly of circles.
    """
    circles = MultiPolygon([Point(x,y).buffer(r,res) for x,y in xy])
    return circles

def particleFill(circR,boundary,world, res = 4, parent = False):
    """
    takes in polygon, identifies circle centers for contained and intersecting circles.
    Adds circles to the physics world. 
    """
    if not world:
        debug = True
    """
    Portability code copy
    #gridXY = getHexGrid2(circR*2,10,10)
    """
    W = 10
    H = 10
    D = circR*2
    cols = W/(D)+1
    hexHeight = D*math.sin(math.pi/3)
    rows = H/(hexHeight)+1
    gridXY = []
    for col in range(cols):
        x0 = (D*col)
        for row in range(rows):
            x00=x0+D/2.0*(row%2)
            y0 = hexHeight*row
            newxy = (x00,y0)
            gridXY.append(newxy)
    
    gridPnts = MultiPoint([Point(xy) for xy in gridXY])
    """
    #circles = makeCircles(circR,gridXY)
    """
    circles = MultiPolygon([Point(x,y).buffer(circR,res) for x,y in gridXY])
    contained = filter(boundary.contains,circles)
    intersected = filter(boundary.intersects,circles)
    #circles = MultiPolygon([Point(x,y).buffer(circR,res) for x,y in gridXY])
    #return circles
    print "made circles"
    if world:
        for circle in contained:
            if parent:
                parent.CreateCircleFixture(pos=circle.coords.xy, radius=circR,friction = parent.friction, density = parent.density)
            else:
                body = world.CreateDynamicBody(position = circle.coords.xy, fixedRotation = False)
                body.CreateCircleFixture(radius=circR,friction = parent.friction, density = parent.density)
    if debug:
        fig = plt.figure(1, figsize = [6.4,4.8], dpi = 100)
        ax = fig.add_subplot(111)
        drawPoints(ax,gridXY)
        for poly in contained:
            drawPatch(ax,poly)
        for poly in intersected:
            drawPatch(ax,poly)
        
        ax.set_xlim(-10,10)
        ax.set_ylim(-10,10)
        ax.set_aspect(1)
        ax.set_title('Circle Packing Test')
        
        plt.show()
    
    
#fig = plt.figure(1, figsize = [6.4,4.8], dpi = 100)
#ax = fig.add_subplot(111)
#circR = .25
#gridXY = getHexGrid2(circR*2,10,10)
#circles = makeCircles(circR,gridXY)
#drawPoints(ax,gridXY)
#gridPnts = MultiPoint([Point(xy) for xy in gridXY])

verticies = [(0,0),
             (0,5),
             (5,5),
             (5,0)]
boundary = Polygon(verticies)
boundary = Point(5,5).buffer(3)
#drawPatch(ax,boundary)

"""
Need function that 
"""
world = False
circR = .25
particleFill(circR,boundary,world)

#contained = filter(boundary.contains,gridPnts)
#intersected = filter(boundary.intersects,gridPnts)
#intPnts = [(point.x,point.y) for point in intersected]
#drawPoints(ax,intPnts)

#contained = filter(boundary.contains,circles)
#intersects = filter(boundary.intersects,circles)
