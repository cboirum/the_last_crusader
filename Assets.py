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
from meshpy.triangle import MeshInfo, build
from Box2D import b2FixtureDef, b2CircleShape, b2EdgeShape
from shapely.geometry import MultiPolygon,Polygon,MultiPoint,Point
from shapely.ops import unary_union
import triMesh
from math import sin,cos
from descartes import PolygonPatch


class triangleMesh(object):
    verticies = []
    polygons = []
    sprites = []
    scaledVerts = []
    triangles = []
    player = []
    
    def __init__(self, PPM,density,friction):
        self.PPM = PPM
        self.density = density
        self.friction = friction
    
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
            newTri=body.CreatePolygonFixture(vertices=triangle, density=self.density, friction=self.friction)
        if self.player:
            self.makePlayer(body)
        if self.flightModel == 'Arcade':
            body.gravityScale = 0
        return body
    
class triMeshConstrained(triangleMesh):
    """
    Triangulation class that uses constrained Delaunay triangulation
    """
    def __init__(self,PPM,density,friction):
        triangleMesh.__init__(self,PPM,density,friction)
    
class PlayerObject(object):
    """
    Represents the player's interface to the game during level play.
    """
    move = {'Up': False,
            'Down': False,
            'Left': False,
            'Right': False
            }
    vehicle = False
    
    def __init__(self):
        pass
    
    def update(self):
        self.checkCollisions()
        #self.applyControls()
        
    def applyControls(self):
        if self.move['Up']:
            self.move['Down'] = False
            self.vehicle.position += (0,self.moveDist)
        if self.move['Down']:
            self.move['Up'] = False
            self.vehicle.position -= (0,self.moveDist)
        if self.move['Left']:
            self.move['Right'] = False
            self.vehicle.position -= (self.moveDist,0)
        if self.move['Right']:
            self.move['Left'] = False
            self.vehicle.position += (self.moveDist,0)
        
    def checkCollisions(self):
        pass
    
    def calculateDamage(self):
        pass
    
class Crusader(triangleMesh):
    
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
    
    
    def __init__(self,PPM,flightModel):
        self.flightModel = flightModel
        if flightModel == 'Realistic':
            density = 5
            friction = 0.3
        elif flightModel == 'Arcade':
            density = 5 
            friction = 0.3
        
        triangleMesh.__init__(self,PPM,density,friction)
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

def printStuff(thing):
        stuff = dir(thing)
        for item in stuff:
            print item
            
def getEdges(polygons):
    """
    Converts a list of polygon edge vertex indecies into a list of point pairs for each edge
    """
    edges = []
    for polygon in polygons:
        for i in range(len(polygon)):
            #print len(polygon)
            j = i+1
            if i+1 >= len(polygon):
                j = 0
            #print i, j
            edges.append([polygon[i],polygon[j]])
    return edges

class meshpyCrusader(Crusader):
    def __init__(self,PPM,flightModel):
        Crusader.__init__(self,PPM,flightModel)
    
    def decompAndScale(self,debug=False):
        """
        Uses MeshPy. VertList allows
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
        mesh_info = MeshInfo()
        mesh_info.set_points(points)
        edges = getEdges(self.polygons)
        
        #self.facets = 
        
        mesh_info.set_facets(edges)
        print "Done setting facets"
        mesh = build(mesh_info)
        print "Done building Mesh"
        printStuff(mesh)
        mesh.write_vtk("crusaderTest.vtk")
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
        
def addDeformable(world, position):
    """
    Adds a deformable body
    """
    
    verts = [(0,0),
             (1,0),
             (1,1),
             (0,1)]
    faces = [[1,2,3,4]]
    """
    Edges are pairs of indecies to "verts" list
    """
    edges = [(1,2),(2,3),(3,4),(4,1)]
    
        
class Crusader2(Crusader):
    
    def __init__(self,PPM,flightModel):
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
        Crusader.__init__(self,PPM,flightModel)
        
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
        
def addCircle(world,position,radius):
    
    fixture=b2FixtureDef(shape=b2CircleShape(radius=radius, p=(0,0)), density=1, friction=1) 
    body = world.CreateDynamicBody(
                position=position, 
                fixtures=fixture
                )
    return body

class MeshHelper(object):
    """
    Helper class for manipulating meshes
    """
    def __init__(self, world):
        self.world = world
    
    def makeStaticEdges(self,mesh):
        """
        Create all (inner) triangles for visualization with static edges
        """
        md = mesh
        for t in md.Triangles[0:md.triangleCount]:
            if t.inside:
                for e in t.e:
                    v1, v2 = e.v
                    if v1 and v2:
                        middleEdges = b2EdgeShape(vertices=[(v1.x,v1.y),(v2.x,v2.y)])
                        newGround = self.world.CreateStaticBody(shapes = middleEdges)
    
    def makeDynamicTriangles(self,mesh,position=False,density=1,friction=1):
        """
        Create all inner triangles as dynamic bodies
        """
        md = mesh
        if not position:
            position = (0,0)
        body=self.world.CreateDynamicBody(position=position)
        for t in md.Triangles[0:md.triangleCount]:
            if t.inside:
                vertices = [(v.x,v.y) for v in t.v]
                newTri=body.CreatePolygonFixture(vertices=vertices, density=density, friction=friction)
    
    def getTryVerts(self,mesh):
        """
        Returns list of triangles defined by lists of 3 verticies
        """
        md = mesh
        tris = []
        for t in md.Triangles[0:md.triangleCount]:
            if t.inside:
                vertices = [(v.x,v.y) for v in t.v]
                tris.append(vertices)
        return tris
    def plotTris(self,mesh):
        """
        Plots all triangles of a mesh as different colors
        """
        tris = self.getTryVerts(mesh)
        for tri in tris:
            x = [vert[0] for vert in tri]
            y = [vert[1] for vert in tri]
            plt.plot(x,y)
        plt.show()
        
    def plotTris2(self,mesh):
        from descartes.patch import PolygonPatch
        tris = self.getTryVerts(mesh)
        for tri in tris:
            x = [vert[0] for vert in tri]
            x.append(tri[0][0])
            y = [vert[1] for vert in tri]
            y.append(tri[0][1])
            plt.plot(x,y)
        plt.show()
        
def takeBite(POC,r,body):
        """
        Takes a bite out of a body, triangulates, etc. does not return anything.
        """

        #(1) loop through all fixtures of body
        for fixture in body.fixtures:
            try:
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
            except:
                    print "error cutting"
                    
def takeBites(POCs,r,body):
        """
        Takes bites out of a body by compositing a circular path for all POCs
        """
        #Tolerances for simplifying cut & body geometry
        cut_simp_tol = .1
        body_simp_tol = .1
        
        #(0) loop through all points of contact & create bite geometry
        N = 20
        Dangle = (3.1415*2)/N
        Sangle = .1 #starting angle of points
        angles = [(Sangle + Dangle*i) for i in range(N)]
        biteGons = []
        print"Contact Points:"
        print POCs
#        for POC in POCs:
#            x,y = POC
#            circV = [Point(r*cos(angle)+x,r*sin(angle)+y) for angle in angles]
#            
#            biteGons.append(circV)
        biteGons = [Point(POC).buffer(r) for POC in POCs]
        #m = MultiPolygon(biteGons)
        b = unary_union(biteGons)
        b.simplify(cut_simp_tol,preserve_topology=False)
        #(1) loop through all fixtures of body
        for fixture in body.fixtures:
            try:
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
#                N = 20
#                Dangle = (3.1415*2)/N
#                Sangle = .1 #starting angle of points
#                angles = [(Sangle + Dangle*i) for i in range(N)]
#                x,y = POC
#                circV = [(r*cos(angle)+x,r*sin(angle)+y) for angle in angles]
#                b = Polygon(circV)
                
                #(3) take bite and get C's verts
                c = a.difference(b)
                c = c.simplify(body_simp_tol,preserve_topology=False)
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
            except:
                    print "error cutting"

def takeBites2(POCs,r,body):
        """
        Takes bites out of a body by compositing a circular path for all POCs
        """
        #Tolerances for simplifying cut & body geometry
        cut_simp_tol = .1
        body_simp_tol = 2
        
        #(0) loop through all points of contact & create bite geometry
        N = 20
        Dangle = (3.1415*2)/N
        Sangle = .1 #starting angle of points
        angles = [(Sangle + Dangle*i) for i in range(N)]
        biteGons = []
        print"Contact Points:"
        print POCs
        #biteGons = [Point(POC).buffer(r,resolution = 2) for POC in POCs]

        #b = unary_union(biteGons)
        b = Point(POCs[0]).buffer(r,resolution = 2)
        b.simplify(cut_simp_tol,preserve_topology=False)
        #(1) loop through all fixtures of body
        for fixture in body.fixtures:
            try:
                fixture = body.fixtures[0]
                boxV = fixture.shape.vertices
                density = fixture.density
                friction = fixture.friction
                             
                #(2a) make box path from verts
                a = Polygon(boxV)
    
                c1 = a.difference(b)
                c = c1.simplify(body_simp_tol,preserve_topology=False)
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
            except:
                print"err"         
                
def takeBites3(POCs,r,body):
        """
        Takes bites out of a body's polygon by compositing all POCs for this body.
        Also uses and stores the "bitten" polygon geometry within the b2body's userData
        so that the triangulated geometry is never used for biting and is regenerated
        everytime the body's geometry changes (at most once per simulation step).
        
        In the future: limit how often geometry can change (impose # of timesteps between bites)
                        or even make seperate biting frequency limits based on event type
                            (as in bullet strike, ground strike, vehicle/building strike)
        """
        #Tolerances for simplifying cut & body geometry
        cut_simp_tol = .1
        body_simp_tol = 2
        bodySimplify = False
        cutSimplify = False
        cleaning = False #uses a buffer(0) command to "clean" body polygon
        scrubbing = False #dilates and then errodes polygon to clean "small" appendages & holes
        scrubbing_distance = .1 #distance in meters to dilate & then errode
        debugPlot = False
        convexBite = True #limits bite geometry to a convex hull of bite circles
        if debugPlot:
            fig = plt.figure(1, figsize = [10,10], dpi = 300)
            PltOriginal = fig.add_subplot(131)
            PltOriginal.set_xlim(-40,40)
            PltOriginal.set_ylim(-40,40)
            PltBite = fig.add_subplot(132)
            PltBite.set_xlim(-40,40)
            PltBite.set_ylim(-40,40)
            PltBitten = fig.add_subplot(133)
            PltBitten.set_xlim(-40,40)
            PltBitten.set_ylim(-40,40)
        """
        (0) loop through all points of contact & create bite geometry union
        """
        N = 20
        Dangle = (3.1415*2)/N
        Sangle = .1 #starting angle of points
        angles = [(Sangle + Dangle*i) for i in range(N)]
        biteGons = []
        #print"Contact Points:"
        #print POCs
        biteGons = [Point(POC).buffer(r,resolution = 2) for POC in POCs]
        
        b = unary_union(biteGons)
        if convexBite:
            b = b.convex_hull
        
        #b = Point(POCs[0]).buffer(r,resolution = 2)
        if cutSimplify:
            b.simplify(cut_simp_tol,preserve_topology=False)
        if debugPlot:
            patch = PolygonPatch(b)
            PltBite.add_patch(patch)
        """
        (1) loop through all fixtures of body and retrieve total outer geometry verticies. 
            (if there is no polygon already created for this body)
        """
        needsPoly = True
        if body.userData and 'Polygon' in body.userData.keys() and body.userData['Polygon']:
            a = body.userData['Polygon']
            needsPoly = False
            print 'Polygon Found'
        else:
            polygons = []
            for fixture in body.fixtures:
                try:
                    polygons.append(Polygon(fixture.shape.vertices))
                except:
                    print'fixture lost'
            a = unary_union(polygons)
#        try:
        if debugPlot:
            patch = PolygonPatch(a)
            PltOriginal.add_patch(patch)
            
        fixture = body.fixtures[0]
        density = fixture.density
        friction = fixture.friction
            
        c = a.difference(b)
        if bodySimplify:
            c = c.simplify(body_simp_tol,preserve_topology=False)
        if cleaning:
            c = c.buffer(0)
        if scrubbing:
            c = c.buffer(scrubbing_distance)
            c = c.buffer(-scrubbing_distance)
        body.userData['Polygon'] = c
        
        if debugPlot:
            patch = PolygonPatch(c)
            PltBitten.add_patch(patch)
        
        xy = c.exterior.coords.xy
        cVerts = [(xy[0][i],xy[1][i]) for i in range(len(xy[0])-1)]

        #(4b) triangulate C into triangles
        md = triMesh.makeMyMesh(cVerts)

        #(5) replace input body's fixture with new one made of multiple triangles
        for fixture in body.fixtures:
            body.DestroyFixture(fixture)
        for t in md.Triangles[0:md.triangleCount]:
            if t.inside:
                vertices = [(v.x,v.y) for v in t.v]
                x = [v.x for v in t.v]
                y = [v.y for v in t.v]
                testPoint = Point(sum(x)/3.0,sum(y)/3.0)
                if testPoint.within(c):
                    newTri=body.CreatePolygonFixture(vertices=vertices, density=density, friction=friction)
        if debugPlot:
            plt.show()
#        except:
#            print"err"
def takeBites4(POCs,r,body):
        """
        Takes bites out of a body's polygon by compositing all POCs for this body.
        Also uses and stores the "bitten" polygon geometry within the b2body's userData
        so that the triangulated geometry is never used for biting and is regenerated
        everytime the body's geometry changes (at most once per simulation step).
        
        **Chages from v3:
        Modified to create new bodies if seperation occurs. 
            This will need to be modified eventually to handle player objects by retaining 
            control of the vehicle segment still attached to an indestructable player body
            and/or just a control point.
        
        In the future: limit how often geometry can change (impose # of timesteps between bites)
                        or even make seperate biting frequency limits based on event type
                            (as in bullet strike, ground strike, vehicle/building strike)
        """
        #Tolerances for simplifying cut & body geometry
        cut_simp_tol = .1
        body_simp_tol = 2
        bodySimplify = False
        cutSimplify = False
        cleaning = False #uses a buffer(0) command to "clean" body polygon
        scrubbing = False #dilates and then errodes polygon to clean "small" appendages & holes
        scrubbing_distance = .1 #distance in meters to dilate & then errode
        debugPlot = False
        convexBite = True #limits bite geometry to a convex hull of bite circles
        if debugPlot:
            fig = plt.figure(1, figsize = [10,10], dpi = 300)
            PltOriginal = fig.add_subplot(131)
            PltOriginal.set_xlim(-40,40)
            PltOriginal.set_ylim(-40,40)
            PltBite = fig.add_subplot(132)
            PltBite.set_xlim(-40,40)
            PltBite.set_ylim(-40,40)
            PltBitten = fig.add_subplot(133)
            PltBitten.set_xlim(-40,40)
            PltBitten.set_ylim(-40,40)
        """
        (0) loop through all points of contact & create bite geometry union
        """
        N = 20
        Dangle = (3.1415*2)/N
        Sangle = .1 #starting angle of points
        angles = [(Sangle + Dangle*i) for i in range(N)]
        biteGons = []
        #print"Contact Points:"
        #print POCs
        biteGons = [Point(POC).buffer(r,resolution = 2) for POC in POCs]
        
        b = unary_union(biteGons)
        if convexBite:
            b = b.convex_hull
        
        #b = Point(POCs[0]).buffer(r,resolution = 2)
        if cutSimplify:
            b.simplify(cut_simp_tol,preserve_topology=False)
        if debugPlot:
            patch = PolygonPatch(b)
            PltBite.add_patch(patch)
        """
        (1) loop through all fixtures of body and retrieve total outer geometry verticies. 
            (if there is no polygon already created for this body)
        """
        needsPoly = True
        if body.userData and 'Polygon' in body.userData.keys() and body.userData['Polygon']:
            a = body.userData['Polygon']
            needsPoly = False
            print 'Polygon Found'
        else:
            polygons = []
            for fixture in body.fixtures:
                try:
                    polygons.append(Polygon(fixture.shape.vertices))
                except:
                    print'fixture lost'
            a = unary_union(polygons)
#        try:
        if debugPlot:
            patch = PolygonPatch(a)
            PltOriginal.add_patch(patch)
            
        fixture = body.fixtures[0]
        density = fixture.density
        friction = fixture.friction
            
        c = a.difference(b)
        if bodySimplify:
            c = c.simplify(body_simp_tol,preserve_topology=False)
        if cleaning:
            c = c.buffer(0)
        if scrubbing:
            c = c.buffer(scrubbing_distance)
            c = c.buffer(-scrubbing_distance)
        body.userData['Polygon'] = c
        
        if debugPlot:
            patch = PolygonPatch(c)
            PltBitten.add_patch(patch)
        """
        Check to see if body has segmented
        """
        if 'exterior' in dir(c):
            print"good"
            reMeshBody4(c,body,density,friction)
        elif len(c)>0 and 'exterior' in dir(c[0]):
            print"body separated, new body creation"
            """
            for now, the first new body will be the vehicle/existing body
            """
            reMeshBody4(c[0],body,density,friction)
            body.userData['Polygon'] = c[0]
            world = body.world
            
            
            for polygon in c[1:]:
                newUserData = dict(body.userData)
                newUserData['ID'] = newUserData['ID'] + '_1'
                newUserData['Polygon'] = polygon
                new_body=world.CreateDynamicBody(
                    userData = newUserData,                             
                    position=body.position,
                    angle=body.angle,
                    linearVelocity=body.linearVelocity,
                    angularVelocity=body.angularVelocity,
                    )
                reMeshBody4(polygon,new_body,density,friction)
        if debugPlot:
            plt.show()
            
def takeBites5(POCs,r,body,VOCs,packing=10):
        """
        Takes bites out of a body's polygon by compositing all POCs for this body.
        Also uses and stores the "bitten" polygon geometry within the b2body's userData
        so that the triangulated geometry is never used for biting and is regenerated
        everytime the body's geometry changes (at most once per simulation step).
        
        **Changes in v4:
        Modified to create new bodies if seperation occurs. 
            This will need to be modified eventually to handle player objects by retaining 
            control of the vehicle segment still attached to an indestructable player body
            and/or just a control point.
            
        **Changes in v5:
        Adds particles to the void created when biting.
        
        In the future: limit how often geometry can change (impose # of timesteps between bites)
                        or even make seperate biting frequency limits based on event type
                            (as in bullet strike, ground strike, vehicle/building strike)
                            
        
        """
        #Tolerances for simplifying cut & body geometry
        cut_simp_tol = 1
        body_simp_tol = .25
        biteResolution = 2
        bodySimplify = True
        cutSimplify = False
        cleaning = False #uses a buffer(0) command to "clean" body polygon
        scrubbing = False #dilates and then errodes polygon to clean "small" appendages & holes
        scrubbing_distance = .1 #distance in meters to dilate & then errode
        debugPlot = False
        convexBite = False #limits bite geometry to a convex hull of bite circles
        packingFactor = packing # number of circles to pack into a single bite circle of radius r
        print"PackingFactor:%d"%packingFactor
        if debugPlot:
            fig = plt.figure(1, figsize = [10,10], dpi = 300)
            PltOriginal = fig.add_subplot(131)
            PltOriginal.set_xlim(-40,40)
            PltOriginal.set_ylim(-40,40)
            PltBite = fig.add_subplot(132)
            PltBite.set_xlim(-40,40)
            PltBite.set_ylim(-40,40)
            PltBitten = fig.add_subplot(133)
            PltBitten.set_xlim(-40,40)
            PltBitten.set_ylim(-40,40)
        """
        (0) loop through all points of contact & create bite geometry union
        """
#        N = 20
#        Dangle = (3.1415*2)/N
#        Sangle = .1 #starting angle of points
#        angles = [(Sangle + Dangle*i) for i in range(N)]
        biteGons = []
        #print"Contact Points:"
        #print POCs
        biteGons = [Point(POC).buffer(r,resolution = biteResolution) for POC in POCs]
        
        b = unary_union(biteGons)
        if convexBite:
            b = b.convex_hull
        
        #b = Point(POCs[0]).buffer(r,resolution = 2)
        if cutSimplify:
            b.simplify(cut_simp_tol,preserve_topology=False)
        if debugPlot:
            patch = PolygonPatch(b)
            PltBite.add_patch(patch)
        """
        (1) loop through all fixtures of body and retrieve total outer geometry verticies. 
            (if there is no polygon already created for this body)
        """
        needsPoly = True
        if body.userData and 'Polygon' in body.userData.keys() and body.userData['Polygon']:
            a = body.userData['Polygon']
            needsPoly = False
            print 'Polygon Found'
        else:
            polygons = []
            for fixture in body.fixtures:
                try:
                    polygons.append(Polygon(fixture.shape.vertices))
                except:
                    print'fixture lost'
            a = unary_union(polygons)
#        try:
        if debugPlot:
            patch = PolygonPatch(a)
            PltOriginal.add_patch(patch)

        c = a.difference(b)
        if bodySimplify:
            c = c.simplify(body_simp_tol,preserve_topology=False)
        if cleaning:
            c = c.buffer(0)
        if scrubbing:
            c = c.buffer(scrubbing_distance)
            c = c.buffer(-scrubbing_distance)
        #
        d = a.intersection(b)
        gridSpacing = r/10
        #gX = range()
        if debugPlot:
            patch = PolygonPatch(c)
            PltBitten.add_patch(patch)
        """
        Check to see if body has segmented
        """
        remeshBodies(c,body)
        circR = r/packingFactor
        boundary = d
        world = body.world
        noTry = False
        #particleFill(circR,boundary,world, res = 4, parent = False)
        if noTry:
            particleFill(circR,boundary,world, res = 4, parent = body, biteGons=biteGons, VOCs=VOCs)
        else:
            
            try:
                isGood = boundary.centroid.xy
                particleFill(circR,boundary,world, res = 4, parent = body, biteGons=biteGons, VOCs=VOCs)
            except:
                pass
        if debugPlot:
            plt.show()

#class Chomper(object):
#    """
#    Contains settings & methods for geometry modification.
#    """
#    cut_simp_tol = 1
#    body_simp_tol = .25
#    biteResolution = 2
#    bodySimplify = True
#    cutSimplify = False
#    cleaning = False #uses a buffer(0) command to "clean" body polygon
#    scrubbing = False #dilates and then errodes polygon to clean "small" appendages & holes
#    scrubbing_distance = .1 #distance in meters to dilate & then errode
#    debugPlot = False
#    convexBite = False #limits bite geometry to a convex hull of bite circles
#    packingFactor = packing # number of circles to pack into a single bite circle of radius r
#    debug = False
#    gridW = 40
#    gridH = 40
#    W = 40
#    H = 40
#    
#    def __init__(self):
#        pass
#    
#    def getGridXY(self):
#        D = circR*2.0
#        cols = int(W/(D))+1
#        hexHeight = D*math.sin(math.pi/3)
#        rows = H/(hexHeight)+1
#        gridXY = []
#        gridOffset = boundary.centroid.xy
#        gO_X = gridOffset[0][0]-W/2.0
#        gO_Y = gridOffset[1][0]-H/2.0
#        for col in range(cols):
#            x0 = (D*col)
#            for row in range(rows):
#                x00=x0+D/2.0*(row%2)
#                y0 = hexHeight*row
#                newxy = (x00+gO_X,y0+gO_Y)
#                gridXY.append(newxy)
#    
#        self.gridPnts = MultiPoint([Point(xy) for xy in gridXY])
#        
#    def takeBites(self):
#        if debugPlot:
#            fig = plt.figure(1, figsize = [10,10], dpi = 300)
#            PltOriginal = fig.add_subplot(131)
#            PltOriginal.set_xlim(-40,40)
#            PltOriginal.set_ylim(-40,40)
#            PltBite = fig.add_subplot(132)
#            PltBite.set_xlim(-40,40)
#            PltBite.set_ylim(-40,40)
#            PltBitten = fig.add_subplot(133)
#            PltBitten.set_xlim(-40,40)
#            PltBitten.set_ylim(-40,40)
#            
#    def getBiteGeom(self):
#        """
#        (0) loop through all points of contact & create bite geometry union
#        """
#        biteGons = []
#        biteGons = [Point(POC).buffer(r,resolution = biteResolution) for POC in POCs]
#        
#        b = unary_union(biteGons)
#        if convexBite:
#            b = b.convex_hull
#        
#        if cutSimplify:
#            b.simplify(cut_simp_tol,preserve_topology=False)
#        if debugPlot:
#            patch = PolygonPatch(b)
#            PltBite.add_patch(patch)
#            
#    def makePolygons(self):
#        """
#        (1) loop through all fixtures of body and retrieve total outer geometry verticies. 
#            (if there is no polygon already created for this body)
#        """
#        needsPoly = True
#        if body.userData and 'Polygon' in body.userData.keys() and body.userData['Polygon']:
#            a = body.userData['Polygon']
#            needsPoly = False
#            print 'Polygon Found'
#        else:
#            polygons = []
#            for fixture in body.fixtures:
#                try:
#                    polygons.append(Polygon(fixture.shape.vertices))
#                except:
#                    print'fixture lost'
#            a = unary_union(polygons)
##        try:
#        if debugPlot:
#            patch = PolygonPatch(a)
#            PltOriginal.add_patch(patch)
#
#        c = a.difference(b)
#        if bodySimplify:
#            c = c.simplify(body_simp_tol,preserve_topology=False)
#        if cleaning:
#            c = c.buffer(0)
#        if scrubbing:
#            c = c.buffer(scrubbing_distance)
#            c = c.buffer(-scrubbing_distance)
#        #
#        d = a.intersection(b)
#        gridSpacing = r/10
#        #gX = range()
#        if debugPlot:
#            patch = PolygonPatch(c)
#            PltBitten.add_patch(patch)
#            
#    def segmentCheck(self):
#        """
#        Check to see if body has segmented
#        """
#        remeshBodies(c,body)
#        circR = r/packingFactor
#        boundary = d
#        world = body.world
#        noTry = False
#        #particleFill(circR,boundary,world, res = 4, parent = False)
#        if noTry:
#            particleFill(circR,boundary,world, res = 4, parent = body, biteGons=biteGons, VOCs=VOCs)
#        else:
#            
#            try:
#                isGood = boundary.centroid.xy
#                particleFill(circR,boundary,world, res = 4, parent = body, biteGons=biteGons, VOCs=VOCs)
#            except:
#                pass
#        if debugPlot:
#            plt.show()
#    
#    def particleFill(self):
#        circles = MultiPolygon([Point(x,y).buffer(circR,res) for x,y in gridXY])
#        contained = filter(boundary.contains,circles)
#        intersected = filter(boundary.intersects,circles)
#        impacted = []
#        allImp = []
#        for bite in biteGons:
#            cX,cY = bite.centroid.xy
#            center = (cX[0],cY[0])
#            bound = Point(center).buffer(circR)
#            within = filter(bound.intersects,contained)
#            try:
#                impacted.append(within)
#                allImp.extend(within)
#            except:
#                pass
#        #print"impacted: %d"%len(impacted)
#        #print "made circles"
#        gibDensity = 1
#        gibFriction = 1
#        if world:
#            for circle in contained:
#                cX,cY = circle.centroid.xy
#                center = (cX[0],cY[0])
#                if parenting:
#                    fxt = parent.fixtures[0]
#                    
#                    parent.CreateCircleFixture(pos=center, radius=circR,friction = fxt.friction, density = fxt.density)
#                else:
#                    gibFriction
#                    if circle in allImp:
#                        for group in impacted:
#                            if circle in group:
#                                velocity = VOCs[impacted.index(group)]
#                                break
#                    else:
#                        velocity = parent.linearVelocity
#                    body = world.CreateDynamicBody(position = parent.GetWorldPoint(center), fixedRotation = False,linearVelocity=velocity)
#                    body.CreateCircleFixture(radius=circR,friction = gibFriction, density = gibDensity)
#                    body.IsBullet = True
#        if debug:
#            fig = plt.figure(1, figsize = [6.4,4.8], dpi = 100)
#            ax = fig.add_subplot(111)
#            drawPatch(ax,boundary)
#            drawPoints(ax,gridXY)
#            for poly in contained:
#                drawPatch(ax,poly)
#            for poly in intersected:
#                drawPatch(ax,poly)
#            for poly in allImp:
#                drawPatch(ax,poly,alpha = 1)
#            
#            
#            ax.set_xlim(-40,40)
#            ax.set_ylim(-40,40)
#            ax.set_aspect(1)
#            ax.set_title('Circle Packing Test')
#            
#            plt.show()
        
     
def takeBites6(POCs,r,body,VOCs,packing=10):

        """
        Takes bites out of a body's polygon by compositing all POCs for this body.
        Also uses and stores the "bitten" polygon geometry within the b2body's userData
        so that the triangulated geometry is never used for biting and is regenerated
        everytime the body's geometry changes (at most once per simulation step).
        
        **Changes in v4:
        Modified to create new bodies if seperation occurs. 
            This will need to be modified eventually to handle player objects by retaining 
            control of the vehicle segment still attached to an indestructable player body
            and/or just a control point.
            
        **Changes in v5:
        Adds particles to the void created when biting.
        
        **Changes in v6:
        Code reoganization and functionalization for easieration of experimentation. Modularity.
        
        In the future: limit how often geometry can change (impose # of timesteps between bites)
                        or even make seperate biting frequency limits based on event type
                            (as in bullet strike, ground strike, vehicle/building strike)
        """
        #Tolerances for simplifying cut & body geometry
        cut_simp_tol = 1
        body_simp_tol = .25
        biteResolution = 2
        bodySimplify = True
        cutSimplify = False
        cleaning = False #uses a buffer(0) command to "clean" body polygon
        scrubbing = False #dilates and then errodes polygon to clean "small" appendages & holes
        scrubbing_distance = .1 #distance in meters to dilate & then errode
        debugPlot = False
        convexBite = False #limits bite geometry to a convex hull of bite circles
        packingFactor = packing # number of circles to pack into a single bite circle of radius r
        print"PackingFactor:%d"%packingFactor
        if debugPlot:
            fig = plt.figure(1, figsize = [10,10], dpi = 300)
            PltOriginal = fig.add_subplot(131)
            PltOriginal.set_xlim(-40,40)
            PltOriginal.set_ylim(-40,40)
            PltBite = fig.add_subplot(132)
            PltBite.set_xlim(-40,40)
            PltBite.set_ylim(-40,40)
            PltBitten = fig.add_subplot(133)
            PltBitten.set_xlim(-40,40)
            PltBitten.set_ylim(-40,40)
        """
        (0) loop through all points of contact & create bite geometry union
        """
#        N = 20
#        Dangle = (3.1415*2)/N
#        Sangle = .1 #starting angle of points
#        angles = [(Sangle + Dangle*i) for i in range(N)]
        biteGons = []
        #print"Contact Points:"
        #print POCs
        biteGons = [Point(POC).buffer(r,resolution = biteResolution) for POC in POCs]
        
        b = unary_union(biteGons)
        if convexBite:
            b = b.convex_hull
        
        #b = Point(POCs[0]).buffer(r,resolution = 2)
        if cutSimplify:
            b.simplify(cut_simp_tol,preserve_topology=False)
        if debugPlot:
            patch = PolygonPatch(b)
            PltBite.add_patch(patch)
        """
        (1) loop through all fixtures of body and retrieve total outer geometry verticies. 
            (if there is no polygon already created for this body)
        """
        needsPoly = True
        if body.userData and 'Polygon' in body.userData.keys() and body.userData['Polygon']:
            a = body.userData['Polygon']
            needsPoly = False
            print 'Polygon Found'
        else:
            polygons = []
            for fixture in body.fixtures:
                try:
                    polygons.append(Polygon(fixture.shape.vertices))
                except:
                    print'fixture lost'
            a = unary_union(polygons)
#        try:
        if debugPlot:
            patch = PolygonPatch(a)
            PltOriginal.add_patch(patch)

        c = a.difference(b)
        if bodySimplify:
            c = c.simplify(body_simp_tol,preserve_topology=False)
        if cleaning:
            c = c.buffer(0)
        if scrubbing:
            c = c.buffer(scrubbing_distance)
            c = c.buffer(-scrubbing_distance)
        #
        d = a.intersection(b)
        gridSpacing = r/10
        #gX = range()
        if debugPlot:
            patch = PolygonPatch(c)
            PltBitten.add_patch(patch)
        """
        Check to see if body has segmented
        """
        remeshBodies(c,body)
        circR = r/packingFactor
        boundary = d
        world = body.world
        noTry = False
        #particleFill(circR,boundary,world, res = 4, parent = False)
        if noTry:
            particleFill(circR,boundary,world, res = 4, parent = body, biteGons=biteGons, VOCs=VOCs)
        else:
            
            try:
                isGood = boundary.centroid.xy
                particleFill(circR,boundary,world, res = 4, parent = body, biteGons=biteGons, VOCs=VOCs)
            except:
                pass
        if debugPlot:
            plt.show()
 
def takeBites7(POCs,r,body,VOCs,packing=10):

       """
       Takes bites out of a body's polygon by compositing all POCs for this body.
       Also uses and stores the "bitten" polygon geometry within the b2body's userData
       so that the triangulated geometry is never used for biting and is regenerated
       everytime the body's geometry changes (at most once per simulation step).
       
       **Changes in v4:
       Modified to create new bodies if seperation occurs. 
           This will need to be modified eventually to handle player objects by retaining 
           control of the vehicle segment still attached to an indestructable player body
           and/or just a control point.
           
       **Changes in v5:
       Adds particles to the void created when biting.
       
       **Changes in v6:
       Code reoganization and functionalization for easieration of experimentation. Modularity.
       
       In the future: limit how often geometry can change (impose # of timesteps between bites)
                       or even make seperate biting frequency limits based on event type
                           (as in bullet strike, ground strike, vehicle/building strike)
                           
        **Changes in v7:
        Particles track the object they came from (this won't work after segmentation) and 
        Continuously calculate their relative velocity to the parent. If the velocity is below a
        threshold, then the particle will resolidified into it.
                           
        
       """
       #Tolerances for simplifying cut & body geometry
       cut_simp_tol = 1
       body_simp_tol = .25
       biteResolution = 2
       bodySimplify = True
       cutSimplify = False
       cleaning = False #uses a buffer(0) command to "clean" body polygon
       scrubbing = False #dilates and then errodes polygon to clean "small" appendages & holes
       scrubbing_distance = .1 #distance in meters to dilate & then errode
       debugPlot = False
       convexBite = False #limits bite geometry to a convex hull of bite circles
       packingFactor = packing # number of circles to pack into a single bite circle of radius r
       print"PackingFactor:%d"%packingFactor
       if debugPlot:
           fig = plt.figure(1, figsize = [10,10], dpi = 300)
           PltOriginal = fig.add_subplot(131)
           PltOriginal.set_xlim(-40,40)
           PltOriginal.set_ylim(-40,40)
           PltBite = fig.add_subplot(132)
           PltBite.set_xlim(-40,40)
           PltBite.set_ylim(-40,40)
           PltBitten = fig.add_subplot(133)
           PltBitten.set_xlim(-40,40)
           PltBitten.set_ylim(-40,40)
       """
       (0) loop through all points of contact & create bite geometry union
       """
#        N = 20
#        Dangle = (3.1415*2)/N
#        Sangle = .1 #starting angle of points
#        angles = [(Sangle + Dangle*i) for i in range(N)]
       biteGons = []
       #print"Contact Points:"
       #print POCs
       biteGons = [Point(POC).buffer(r,resolution = biteResolution) for POC in POCs]
       
       b = unary_union(biteGons)
       if convexBite:
           b = b.convex_hull
       
       #b = Point(POCs[0]).buffer(r,resolution = 2)
       if cutSimplify:
           b.simplify(cut_simp_tol,preserve_topology=False)
       if debugPlot:
           patch = PolygonPatch(b)
           PltBite.add_patch(patch)
       """
       (1) loop through all fixtures of body and retrieve total outer geometry verticies. 
           (if there is no polygon already created for this body)
       """
       needsPoly = True
       if body.userData and 'Polygon' in body.userData.keys() and body.userData['Polygon']:
           a = body.userData['Polygon']
           needsPoly = False
           print 'Polygon Found'
       else:
           polygons = []
           for fixture in body.fixtures:
               try:
                   polygons.append(Polygon(fixture.shape.vertices))
               except:
                   print'fixture lost'
           a = unary_union(polygons)
#        try:
       if debugPlot:
           patch = PolygonPatch(a)
           PltOriginal.add_patch(patch)

       c = a.difference(b)
       if bodySimplify:
           c = c.simplify(body_simp_tol,preserve_topology=False)
       if cleaning:
           c = c.buffer(0)
       if scrubbing:
           c = c.buffer(scrubbing_distance)
           c = c.buffer(-scrubbing_distance)
       #
       d = a.intersection(b)
       gridSpacing = r/10
       #gX = range()
       if debugPlot:
           patch = PolygonPatch(c)
           PltBitten.add_patch(patch)
       """
       Check to see if body has segmented
       """
       remeshBodies(c,body)
       circR = r/packingFactor
       boundary = d
       world = body.world
       noTry = False
       #particleFill(circR,boundary,world, res = 4, parent = False)
       if noTry:
           particleFill(circR,boundary,world, res = 4, parent = body, biteGons=biteGons, VOCs=VOCs)
       else:
           
           try:
               isGood = boundary.centroid.xy
               particleFill(circR,boundary,world, res = 4, parent = body, biteGons=biteGons, VOCs=VOCs)
           except:
               pass
       if debugPlot:
           plt.show()
 
            
def particleFill(circR,boundary,world, res = 4, parent = False, biteGons=False, VOCs=False):
    """
    takes in polygon, identifies circle centers for contained and intersecting circles.
    Adds circles to the physics world. Keeps parent body saved into particle's userdata.
    """
    debug = False
    parenting = False #True makes gibs attatch to parent
    if not world:
        debug = True
    """
    Portability code copy
    #gridXY = getHexGrid2(circR*2,10,10)
    """
    W = 40
    H = 40
    D = circR*2.0
    cols = int(W/(D))+1
    hexHeight = D*math.sin(math.pi/3)
    rows = H/(hexHeight)+1
    gridXY = []
    gridOffset = boundary.centroid.xy
    gO_X = gridOffset[0][0]-W/2.0
    gO_Y = gridOffset[1][0]-H/2.0
    for col in range(cols):
        x0 = (D*col)
        for row in range(rows):
            x00=x0+D/2.0*(row%2)
            y0 = hexHeight*row
            newxy = (x00+gO_X,y0+gO_Y)
            gridXY.append(newxy)
    
    gridPnts = MultiPoint([Point(xy) for xy in gridXY])
    """
    #circles = makeCircles(circR,gridXY)
    """
    circles = MultiPolygon([Point(x,y).buffer(circR,res) for x,y in gridXY])
    contained = filter(boundary.contains,circles)
    intersected = filter(boundary.intersects,circles)
    impacted = []
    allImp = []
    for bite in biteGons:
        cX,cY = bite.centroid.xy
        center = (cX[0],cY[0])
        bound = Point(center).buffer(circR)
        within = filter(bound.intersects,contained)
        try:
            impacted.append(within)
            allImp.extend(within)
        except:
            pass
    #print"impacted: %d"%len(impacted)
    #print "made circles"
    gibDensity = 1
    gibFriction = 1
    if world:
        localParticles = []
        for circle in contained:
            cX,cY = circle.centroid.xy
            center = (cX[0],cY[0])
            if parenting:
                fxt = parent.fixtures[0]
                
                parent.CreateCircleFixture(pos=center, radius=circR,friction = fxt.friction, density = fxt.density)
            else:
                gibFriction
                if circle in allImp:
                    for group in impacted:
                        if circle in group:
                            velocity = VOCs[impacted.index(group)]
                            break
                else:
                    velocity = parent.linearVelocity
                body = world.CreateDynamicBody(position = parent.GetWorldPoint(center), fixedRotation = False,linearVelocity=velocity)
                #body.userData = {'Parent':parent}
                body.CreateCircleFixture(radius=circR,friction = gibFriction, density = gibDensity)
                body.IsBullet = True
                #localParticles.append(body)
                #world.particleGroups.append(localParticles)
    if debug:
        fig = plt.figure(1, figsize = [6.4,4.8], dpi = 100)
        ax = fig.add_subplot(111)
        drawPatch(ax,boundary)
        drawPoints(ax,gridXY)
        for poly in contained:
            drawPatch(ax,poly)
        for poly in intersected:
            drawPatch(ax,poly)
        for poly in allImp:
            drawPatch(ax,poly,alpha = 1)
        
        
        ax.set_xlim(-40,40)
        ax.set_ylim(-40,40)
        ax.set_aspect(1)
        ax.set_title('Circle Packing Test')
        
        plt.show()
        
"""
Debug Drawing Functions
"""
circleResolution = 4
edgeLambda = math.sin(math.pi/(circleResolution*4))

def drawPoint(ax,x,y):
    drawCircle(ax,.05,x,y,alpha = .5)

def drawPoints(ax,xy):
    for x,y in xy:
        drawPoint(ax,x,y)
        
def drawCircle(ax,r,x,y,res=circleResolution,alpha = 1):
    """
    Draws a circle on an axis
    """
    circ = Point(x,y).buffer(r,res)
    patch = PolygonPatch(circ,alpha = alpha)
    ax.add_patch(patch)
    return circ

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

def drawPatch(ax,poly,alpha = .25):
    patch = PolygonPatch(poly, alpha = alpha)
    ax.add_patch(patch)
            
def remeshBodies(c,body):
    fixture = body.fixtures[0]
    density = fixture.density
    friction = fixture.friction
    if c and 'exterior' in dir(c):
        print"good"
        body.userData['Polygon'] = c
        reMeshBody4(c,body,density,friction)
    elif len(c)>0 and 'exterior' in dir(c[0]):
        print"body separated, new body creation"
        """
        for now, the first new body will be the vehicle/existing body
        """
        reMeshBody4(c[0],body,density,friction)
        body.userData['Polygon'] = c[0]
        world = body.world
        
        for polygon in c[1:]:
            newUserData = dict(body.userData)
            newUserData['ID'] = newUserData['ID'] + '_1'
            newUserData['Polygon'] = polygon
            new_body=world.CreateDynamicBody(
                userData = newUserData,                             
                position=body.position,
                angle=body.angle,
                linearVelocity=body.linearVelocity,
                angularVelocity=body.angularVelocity,
                )
            reMeshBody4(polygon,new_body,density,friction)
            
def reMeshBody4(c,body,density,friction):
    """
    recreates physics geometry fixture for a body given a polygon c.
    """   
    

    try:
        xy = c.exterior.coords.xy
        cVerts = [(xy[0][i],xy[1][i]) for i in range(len(xy[0])-1)]
        #(4b) triangulate C into triangles
        md = triMesh.makeMyMesh(cVerts)
    
        #(5) replace input body's fixture with new one made of multiple triangles
        for fixture in body.fixtures:
            body.DestroyFixture(fixture)
        for t in md.Triangles[0:md.triangleCount]:
            if t.inside:
                vertices = [(v.x,v.y) for v in t.v]
                x = [v.x for v in t.v]
                y = [v.y for v in t.v]
                testPoint = Point(sum(x)/3.0,sum(y)/3.0)
                if testPoint.within(c):
                    newTri=body.CreatePolygonFixture(vertices=vertices, density=density, friction=friction)
    except:
        print'mesh failure, geometry undisturbed'


class ElementBond(object):
    """One connection between two ElementNodes"""
    
    def __init__(self):
        pass

class ElementNode(object):
    """One node of a mesh for representing a discretized solid material"""
    
    neighbors = []
    def __init__(self):
        pass
    
def draw_Sprite_on_Body(screen, sprite, body, PPM, debug=False):
    #Draws an animated sprite onto a body (Need a better way 
    #to assign sprite to body)
    baseImage = sprite#spriteStrips[body.type].next()
    (x,y)= body.worldCenter*PPM
    im = baseImage.get_rect()
    #bounding box for sprite
    boxL_halfm = 2
    boxL = boxL_halfm
    boxH_halfm = boxL*328/1174.0
    boxH = boxH_halfm
    boxL_pix = boxL_halfm*2.0*PPM
    boxH_pix = boxH_halfm*2.0*PPM
    #pixel width, height of sprite on screen
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

def terrainTester(world,vertLimit,Method='Default'):
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
    if not world:
        debug = True
    else:
        debug = False
    
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
    """
    Horizontal Scale Factor is Pixels Per Meter
    """
    hMapHscaleFactor = 1.0
    """
    Vertical Scale Factor is Values Per Meter
    """
    hMapVscaleFactor = 1/100000.0
    
    hMapVoffset = 30
    numGrndVrts = vertLimit#100
    vStart = 3
    vEnd = scipy.minimum( numGrndVrts+vStart,zNew[middle2].shape[0])
    
    nGY = array(zNew[middle2][vStart:vEnd])
    
    nGY = nGY*hMapVscaleFactor-hMapVoffset
    if debug:
        print nGY[0]
        plt.plot(nGY)
        plt.show()
    #nGX = scipy.linspace(1,numGrndVrts)
    nGX = scipy.linspace(0,numGrndVrts)
    nGX = array(range(nGY.shape[0]))*hMapHscaleFactor
    if debug:
        print nGX
        #groundVrts = array([nGY,nGX])
        print nGY.shape
        print nGX.shape
    #firstEdge = b2EdgeShape(vertices=[[nGX[0],nGY[0]],[nGX[1],nGY[1]],[nGX[2],nGY[2]]])
#    lastEdge = b2EdgeShape(vertices=[[nGX[end-2],nGY[end-2]],[nGX[end-1],nGY[end-1]],[nGX[end],nGY[end]]])
#    middleEdges = b2EdgeShape(vertices=[[nGX[x-1],nGY[x-1]],[nGX[x],nGY[x]],[nGX[x+1],nGY[x+1]],[nGX[x+2],nGY[x+2]]])
    from Box2D import b2EdgeShape
    #middleEdges = [b2EdgeShape(vertices=[(nGX[xt],nGY[xt]),(nGX[xt+1],nGY[xt+1])])for xt in range(nGX.size-1)]#,[nGX[x+1],nGY[x+1]],[nGX[x+2],nGY[x+2]]])
    #print middleEdges.shape
    #middleEdges
    if world:
        if Method=='Default':
            middleEdges = [b2EdgeShape(vertices=[(nGX[xt],nGY[xt]),(nGX[xt+1],nGY[xt+1])])for xt in range(nGX.size-1)]
            newGround = world.CreateStaticBody(shapes = middleEdges)
        elif Method == 'New1':
            print "New1 ground generation method not yet implimented"
        elif Method == 'Simple':
            print "Generating simple terrain"
            newGround = world.CreateStaticBody(shapes = [b2EdgeShape(vertices=[(-1000,0),(1000,0)])])
        return newGround

if __name__ == "__main__":
    from Box2D import b2EdgeShape
    terrainTester(world=False,vertLimit=2000)