'''
Created on Aug 14, 2014

@author: BOIRUM

Test for circle packing within circle algorithms for particle system creation within "bitten" geometry.
'''
import Box2D
import matplotlib.pyplot as plt
from descartes import PolygonPatch
from shapely.geometry import MultiPolygon,Polygon,Point
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
            print (row%2)
            y0 = hexHeight*row
            newxy = (x00,y0)
            xy.append(newxy)
    return xy
    

center = (0,0)
fig = plt.figure(1, figsize = [6.4,4.8], dpi = 100)
ax = fig.add_subplot(111)

#drawCirlces()
x = [1, -1, -2, -1, 1, 2]
y = [-2, -2, 0, 2, 2, 0]
xy = zip(x,y)
#drawPoly(ax,xy)
#drawHex(ax,1,0,0)
gridXY = getHexGrid(1,10,10)
gridXY2 = getHexGrid2(1,10,10)
print gridXY
print len(gridXY)
#s = set(gridXY)
#print len(s)
#drawPoints(ax,gridXY)
drawPoints(ax,gridXY2)


ax.set_xlim(-10,10)
ax.set_ylim(-10,10)
ax.set_aspect(1)
ax.set_title('Circle Packing Test')

plt.show()