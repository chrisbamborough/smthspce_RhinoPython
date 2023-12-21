import Rhino
import rhinoscriptsyntax as rs
import SpceDisplay as DD

import System.Drawing as SD
from System import Guid as SG
import scriptcontext

"""
Hookes Law Spring System

Chris Bamborough // smthspce
Feb 2012
"""

# Global Values
globalRefreshRate = 50
globalDamping = 0.05
# When true points will be redrawn
globalDrawLinkPts = True
# Control from gui
# Lower the value the more tight.
globalSlackness = 0.5
globalTolerance = 1

# OBJECT TO CONTROL LINK - WILL BECOME AGENT
class Link (object):
    
    def __init__ (self, POS, MASS=2, ID=None):
        # ----> 4 called from System
        self.pos = POS
        self.connectedSprings = []
        #  5 ---->
        self.fixed = self.getFixedState()
        # Set up forces class global to be used to add all vectors
        self.totalForces = Rhino.Geometry.Vector3d(0,0,0)
        self.mass = MASS
        # setup self.geometry to pass into DP - Point(location: Point3d)
        self.radius = 0.5
        #self.centre = Rhino.Geometry.Point3d(self.pos.X, self.pos.Y, self.pos.Z)
        # *** Added for DP geometry
        self.sphere_geometry = Rhino.Geometry.Sphere(self.pos, self.radius)
        self.sphere_bbox = None
        self.displayColor = SD.Color.Red
        self.displayThickness = 1
        
        self.ID = ID
        if not self.ID:
            self.ID = len(linkPopulationList)
        
    # *** Add Bounding Box Function for use with DP
    def BoundingBox (self):
        # return the bounding box of the objects geometry
        return self.point_geometry.BoundingBox
        
    # *** Add a draw for use with DP
    def Draw (self, args,color,thickness):
        # need to add code to draw a point in Rhino Geometry
        # Setup colour and thickness
        color = self.displayColor
        thickness = self.displayThickness
        args.Display.DrawSphere(self.sphere_geometry,color,thickness)
        #print "the centre of the points", self.pos
        
    # ----> 5
    def getFixedState (self):
        check = False
        for fp in fixedPoints:
            if rs.Distance(self.pos, rs.PointCoordinates(fp)) < globalTolerance:
                check = True
        # ----> 5
        return check
        
    # 11 ----> add start and end links to connected springs list
    def addSpringToList (self, Spring):
        self.connectedSprings.append (Spring)
        
    # 10 ----> Run function called from System.run to take care of relax vector
    def UpdatePos (self):
        if self.fixed == False:
            # Set up a vector
            #totalForceVect = Rhino.Geometry.Vector3d(0,0,0)
            # Loop through connected springs for link
            for spring in self.connectedSprings:
                # call function from spring class 12 ---->
                forceVect = spring.getForceVect(self)
                # add this to the newly formed vector
                #totalForceVect = rs.VectorAdd(totalForceVect,forceVect)
                self.totalForces = self.totalForces + forceVect
            # Then use that totalForceVect but first damp
            #totalForceVectScaled = rs.VectorScale(totalForceVect,globalDamping)
            totalForceVectScaled = rs.VectorScale(self.totalForces,globalDamping)
            # update position by adding current position to new vector
            self.pos = rs.VectorAdd(self.pos,totalForceVectScaled)
            # *** Update the centre of the sphere
            self.sphere_geometry.Center = self.pos
            # *** update the bounding box of the self.sphere
            self.sphere_bbox = self.sphere_geometry.BoundingBox
            
            # loop through all springs in the connected springs list
            for spring in self.connectedSprings:
                # let the spring know about the change in link position
                spring.linkMove(self)
        else:
            return [0,0,0]
            
        # after each time return to 0
        self.totalForces *= 0
            
    # Function to add gravity force to the each link if not fixed
    def ApplyGravity(self, value = -0.98):
        gravityVec = Rhino.Geometry.Vector3d(0,0,value)
        # update overall totalforces before running.
        self.totalForces += gravityVec * self.mass
        
    # *** Function to draw final geometry
    def AddToDocument (self):
        layer = "Link_Spheres"
        if not rs.IsLayer(layer): rs.AddLayer(layer, SD.Color.Blue)
        geom = scriptcontext.doc.Objects.AddSphere(self.sphere_geometry)
        if not geom == SG.Empty:
            rs.ObjectName(geom,str(self.ID))
            rs.ObjectLayer(geom,layer)
     
# OBJECT TO CONTROL SPRING
class Spring (object):
    # 4 ----> called from System - init sets ups springs
    def __init__ (self, LINE, STLINK, ENDLINK, ID=None):
        self.line = LINE
        self.stLink = STLINK
        self.endLink = ENDLINK
        # set up springs from the two link objects 11 ---->
        self.stLink.addSpringToList (self)
        self.endLink.addSpringToList (self)
        # Obtain start and end points for the spring
        self.stPt = self.stLink.pos
        self.endPt = self.endLink.pos
        # calculate the slack length of the spring
        self.slackLength = globalSlackness * rs.Distance(self.stPt,self.endPt)
        
        # variables for geometry for use with DP
        self.line_geometry = Rhino.Geometry.Line(self.stPt,self.endPt)
        self.line_bbox = None
        self.displayColor = SD.Color.Aquamarine
        self.displayThickness = 1
        
        self.ID = ID
        if not self.ID:
            self.ID = len(springPopulationList)
        
    # BoundingBox def for DP
    def BoundingBox(self):
        # return the bounding box of the objects geometry
        return self.line_geometry.BoundingBox
        
    # To draw using the DP
    def Draw(self, args, color, thickness):
        # need to add code to draw a point in Rhino Geometry
        # Setup colour and thickness
        color = self.displayColor
        thickness = self.displayThickness
        # *** need to draw all lines from a line list each time
        #args.Display.DrawLine(self.line_geometry.From,self.line_geometry.To,color)
        args.Display.DrawLine(self.line_geometry, color, thickness)
        #print "the centre of the points", self.pos
        
    # ----> 9 called by System.Run
    def UpdateGeometry(self):
        #Delete the current line
        #rs.DeleteObject(self.line)
        #Draw new line based on new start / end points
        #self.line = rs.AddLine(self.stPt,self.endPt)
        
        # *** Update the start and end point of the line
        self.line_geometry.From = self.stPt
        self.line_geometry.To = self.endPt
        # *** update the bounding box of the self.sphere
        self.line_bbox = self.line_geometry.BoundingBox
        
       
    # called from within update geometry ----> 13
    def linkMove(self, LINK):
        if LINK == self.stLink:
            self.stPt = LINK.pos
        else:
            self.endPt = LINK.pos
        
    # ----> 12 called from within link.UpdatePos
    def getForceVect (self, LINK):
        # Apply hookes law to calculate the magnitude F = -k*x
        magnitude = ((rs.Distance(self.stPt,self.endPt)) - self.slackLength ) / self.slackLength
        # create a vector with the assumption of startpoint being the start
        vector = rs.VectorCreate(self.endPt, self.stPt)
        # check for exceptions, if so reverse vector
        if LINK == self.endLink:
            vector = rs.VectorReverse(vector)
        # Unitize vector
        unitizeVect = rs.VectorUnitize(vector)
        # Scale the vector
        forceVect = rs.VectorScale(unitizeVect,magnitude)
        return forceVect
        
    # *** Function to draw final geometry when 
    def AddToDocument (self):
        layer = "Spring_Lines"
        if not rs.IsLayer(layer): rs.AddLayer(layer, SD.Color.Blue)
        geom = scriptcontext.doc.Objects.AddLine(self.line_geometry)
        if not geom == SG.Empty:
            rs.ObjectName(geom,str(self.ID))
            rs.ObjectLayer(geom,layer)
    
# SYSTEM CLASS TO TAKE CONTROL OF EVRYTHING
class relaxSystem (object):
    
    # ----> 7 called from setup
    def __init__ (self):
        pass
        
    #  - Called by springTopology class initialisation
    def setupNetwork(self, GUIDS):
        #instantiates the topological network of nodes and edges
        self.network = sNetwork.Network(GUIDS)
        self.network.setup(MESHES=False,FACES=False)
        if verbose: print("Completed network setup", time.clock() - startTime)
        
        # delete initial geometry
        
        """
        SuperNetwork outputs 
        self.nodesDict 
        self.edgesDict 
        self.facesDict  
        Need to initialise geometry in DD pipeline
        """
        
        DP.AddObject(node)
        DP.AddObject(edge)
        DP.AddObject(face)

def main ():
     # Establish the Dynamic Display
    DP = DD.DynamicDisplay()
    # Toggle the display on
    DP.DisplayToggle(True)
        
    
    global startTime
    GUIDS = rs.GetObjects("Select the objects to make a network from", rs.filter.curve + rs.filter.point + rs.filter.mesh)
    if verbose: startTime = time.clock()
    system = relaxSystem(GUIDS, GRAVITY=2.0, PRESSURE=0.0, ITERATIONS=10000, SLACKMODE="ByLayerName_Proportional", SLACKVAL=0.4, DAMPING=0.5, DRAWFREQ=None, MINLENGTH=None, MAXLENGTH=None, SLACKUPDATEFREQ=None, PASSIVEMESH=True, TENSIONEDGE=False)
    
    
    # running script 
    for i in range (totalFrames):
        # Add escape test incase doesn't work
        if scriptcontext.escape_test(False): break
        if i%globalRefreshRate == 0:
            # loop through link list
            for spring in springPopulationList:
                # call run function from Link class 9 ----> this redraws the spring line
                spring.UpdateGeometry()
            rs.EnableRedraw(True)
            rs.EnableRedraw(False)
            
        #totalForceRemaining = [0,0,0]
        for link in linkPopulationList:
            # updates the position of the link ---> 10
            link.ApplyGravity()
            link.UpdatePos()
            
        # Redraw once per frame
        scriptcontext.doc.Views.Redraw()
        #Pauses to keep Windows message pump alive so views will update and windows will repaint.
        Rhino.RhinoApp.Wait()

    # *** At the end of the frames draw actual geometry using AddToDocument function
    rs.EnableRedraw(False)
    for link in linkPopulationList:
        link.AddToDocument()

    for spring in springPopulationList:
        spring.AddToDocument()
    rs.EnableRedraw(True)
    
    # Turn off the display pipeline
    DP.DisplayToggle(False)
    # One last redraw
    scriptcontext.doc.Views.Redraw()


# USER INPUT TO START 
if (__name__ == "__main__"):
    main()