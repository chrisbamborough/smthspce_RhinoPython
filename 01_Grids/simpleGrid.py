#! python3

import rhinoscriptsyntax as rs
import rhinoscript.utility as rhutil
import scriptcontext
import Rhino
from System import Guid as GC
from System import Array


gridSize = 10
numX = 10
numY = 10
gridSpace = gridSize/numX

# Dictionary for grid points
gridList = []

# create points using a for loop

def setup():
    #set up the drawing
    # start at 0 in y and increase

    for i in range(0,numY):
        for j in range(0,numX):
            #gridPt = rs.AddPoint(i,j,0)
            x = i*gridSize
            y = j*gridSize
            z = 0

            point = Rhino.Geometry.Point3d(x,y,z)
            point = rhutil.coerce3dpoint(point, True)
            rc = scriptcontext.doc.Objects.AddPoint(point)
            if rc==GC.Empty: raise Exception("unable to add point to document")
            gridList.append(point)


def run():
    #animate the drawing 
    return()
    

# USER INPUT TO START 
if (__name__ == "__main__"):
    setup()
    scriptcontext.doc.Views.Redraw()

