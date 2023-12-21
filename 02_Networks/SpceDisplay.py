import rhinoscriptsyntax as rs
import math
import random
import Rhino
import scriptcontext
import System
from System.Drawing import Color

"""
Display Pipeline Class

Acknowledgements:
Steve Baer


"""

class DynamicDisplay():
    #Global BoundingBox
    BoundingBox = None
    
    #Deault Global Drawing Settings
    defaultColor = System.Drawing.Color.White
    defaultThickness = 1
    
    def __init__(self):
        self.Display = Rhino.Display.DisplayPipeline
        self.objects = []
        self.drawingList = []
        self.BoundingBox = None
        self.displayColor = DynamicDisplay.defaultColor
        self.displayThickness = DynamicDisplay.defaultThickness
        self.on = False
        
        
    
    
    def DisplayToggle(self, on):
        #To avoid multiple activations
        #Only apply these when the switch is changed, i.e. if ON -> OFF and OFF-> ON
        #Nothing happens if ON -> ON
        if on != self.on: 
            #turn the display pipeline on
            if on:
                self.Display.CalculateBoundingBox += self.OnCalcBoundingBox
                self.Display.PostDrawObjects += self.OnDraw
            else:
                self.Display.CalculateBoundingBox -= self.OnCalcBoundingBox
                self.Display.PostDrawObjects -= self.OnDraw
            self.on = on
        else:
            if on:
                msg = "ON"
            else:
                msg = "OFF"
            print ("Display is already Switched", msg)
    
    def OnCalcBoundingBox(self, sender, args):
        #Reset Bounding Box
        self.BoundingBox = DynamicDisplay.BoundingBox
        #called by the displaypipeline to recalculate the bounding box of the objects
        
        if not self.BoundingBox:
            bb = Rhino.Geometry.BoundingBox.Unset
            #Draw all object-based from from objects List
            if self.objects:
                for object in self.objects:
                    bb.Union(object.BoundingBox())
            #Draw all items from drawingList
            if len(self.drawingList) > 0:
                for item in self.drawingList:
                    geo = item[0]
                    geoBBX = None
                    #Get bounding box by two method, whichever works based on the Rhino.Geometry namespaces
                    #Some may have .BoundingBox and some may have .GetBoundingBox()
                    #Try BoundingBox as property
                    try:
                        result = geo.BoundingBox
                    except AttributeError:
                        #print "BoundingBox Not Found"
                        pass
                    else:
                        geoBBX = result
                        bb.Union(geoBBX)
                        pass
                    
                    #Try BoundingBox as Method
                    try:
                        result = geo.GetBoundingBox(False)
                    except AttributeError:
                        #print "BoundingBox Not Found in Method"
                        pass
                    else:
                        geoBBX = result
                        bb.Union(geoBBX)
                        pass
                        
            self.BoundingBox = bb
        args.IncludeBoundingBox(self.BoundingBox)
        
    def OnDraw(self, sender, args ):
        color = self.displayColor
        thickness = self.displayThickness
        #Draw class objects
        if self.objects:
            for object in self.objects:
                object.Draw(args, color, thickness)
        
        #Draw raw geometries with color and thickness
        if len(self.drawingList) > 0 :
            for item in self.drawingList:
                geo = item[0]
                color = item[1]
                thickness = item[2]
                self.DrawGeometry(args, geo, color, thickness)
        
    def AddObject(self, object):
        self.objects.append(object)
    
    def AddGeometry(self, geo, color=None, thickness=None):
        if not color:
            color = self.displayColor
        if not thickness:
            thickness = self.displayThickness
        self.drawingList.append([geo, color, thickness])
        
    def DrawGeometry(self, args, geo, color, thickness):
        
        #All types of draw-able geometry need to be defined here
        #Refer to:
        #Rhino.Display.DisplayPipeline
        #Members
        
        wiresDensity = 3
        if type(geo).__name__ == 'Line':
            args.Display.DrawLine(geo, color, thickness)
        if type(geo).__name__ == 'Sphere':
            args.Display.DrawSphere(geo, color, thickness)
        
        if type(geo).__name__ == 'Box':
            args.Display.DrawBox(geo, color, thickness)
        
        if type(geo).__name__ == 'Brep':
            args.Display.DrawBrepWires(geo, color, wiresDensity)

if (__name__ == "__main__"):
    totalFrames = 360    #defines duration of the script
    
    
    DP = DynamicDisplay() #Establish Display
    DP.DisplayToggle(True) #Display On
    
    #Change Display Settings
    DP.displayColor = System.Drawing.Color.Cyan
    DP.displayThickness = 1.5
    
    #Raw geometry
    
    #newSphere = Rhino.Geometry.Sphere(Rhino.Geometry.Point3d(50,0,0), 10)
    line = Rhino.Geometry.Line(0,0,0,100,100,100)
    DP.AddGeometry(line)
    scriptcontext.doc.Views.Redraw()
    """
    for i in range(totalFrames):
        Rhino.RhinoApp.CommandPrompt = "press escape to cancel   frame=" + str(i)
        
        #Geometry Update
        theta = math.radians(i*20)
        radius = newSphere.Radius
        newPos = Rhino.Geometry.Vector3d(math.sin(theta)*radius , math.cos(theta)*radius, 0)
        newSphere.Center = Rhino.Geometry.Point3d(newPos)
        
        #Redraw View
        scriptcontext.doc.Views.Redraw()
        #Escape Switch
        if scriptcontext.escape_test(False): break
        Rhino.RhinoApp.Wait() #use Wait to allow the window's message pump to keep up
    
    DP.DisplayToggle(False) #Display Off
    """
    scriptcontext.doc.Views.Redraw() #one final clean-up
