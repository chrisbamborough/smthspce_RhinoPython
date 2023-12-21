#written by Dave Pigram for supermanoeuvre
#Code Module started on August 29, 2011

#import libraries
import rhinoscriptsyntax as rs
import math
import Rhino
import rhinoscript
import rhinoscript.utility as rhutil

def addDashedLines(startPt, endPt, dashLength, gapLength, startStyle=1):
    #start style = 0 : Lines start with a dash, is good for drawings
    #start style = 1 : Lines start with the gap, can be good for folding
    startPt = rhutil.coerce3dvector(startPt, True)
    endPt = rhutil.coerce3dvector(endPt, True)
    gapVect = vectorForceLength( [ endPt[0]-startPt[0],endPt[1]-startPt[1],endPt[2]-startPt[2] ], gapLength )
    if startStyle == 1:
        #offset the start and end points by the dash and then recall this function
        startPt = startPt + gapVect
        endPt = endPt - gapVect
        lines = addDashedLines(startPt, endPt, dashLength, gapLength, 0)
    else:
        length = rs.Distance(startPt, endPt)
        lineStep = dashLength + gapLength
        lines = []
    
        if lineStep < length:
            remainder = (length - dashLength)%lineStep
            numDivs = int((length - remainder - dashLength) / lineStep)
            #there is always one more line than there is gap
            dashLength = (length - numDivs*gapLength)/(numDivs+1)
            dashVect = vectorForceLength( gapVect, dashLength )
            gapVect = vectorForceLength( gapVect, gapLength/2)
            pt01 = startPt + dashVect #+ gapVect
            lines.append( rs.AddLine(startPt,pt01) )
            pt00 = pt01 + gapVect*2
            for i in range(numDivs)[1:]:
                pt01 = pt00 + dashVect
                lines.append( rs.AddLine(pt00,pt01) )
                pt00 = pt01 + gapVect*2
            lines.append( rs.AddLine(pt00,endPt) )
        else: 
            try: lines.append(rs.AddLine(startPt, endPt))
            except: print "???"
    return lines

def averageTwoPoints(self, PT1, PT2):
    sum = Rhino.Geometry.Point3d.Add(PT1,PT2)
    avg = Rhino.Geometry.Point3d.Divide(sum,2)
    return avg

def averageTwoPointsWeighted(PT1, PT2, WEIGHTING):
    #weighting must be beteen zero and one, zero will return the first point, one the second
    #0.5 will return the normal average and 0.25 will be on the quarterline closest the first point
    PT1 = Rhino.Geometry.Point3d(PT1[0], PT1[1], PT1[2])
    PT2 = Rhino.Geometry.Point3d(PT2[0], PT2[1], PT2[2])
    if WEIGHTING == 0: return PT1
    elif WEIGHTING == 1:  return PT2
    else:
        PT1Weight = Rhino.Geometry.Point3d.Multiply(1-WEIGHTING, PT1,)
        PT2Weight = Rhino.Geometry.Point3d.Multiply(WEIGHTING, PT2)
        return Rhino.Geometry.Point3d.Add(PT1Weight,PT2Weight)

def cullDuplicatePtOrVec3Ds(PTORVECLIST):
    culledList = []
    culledList.append(PTORVECLIST[0])
    for ptOrVec in PTORVECLIST:
        if (culledList[-1] - ptOrVec).Length > rs.UnitAbsoluteTolerance():
            culledList.append(ptOrVec)
    return culledList

def getMinDistBetweenTwoLines(line0, line1):
    
        line0Vec = line0.UnitTangent    #Vector
        line1Vec = line1.UnitTangent    #Vector
        w = line0.From - line1.From     #Vector
        a = line0Vec*line0Vec           #Float   (Dot Product)  Always >=0
        b = line0Vec*line1Vec           #Float   (Dot Product)
        c = line1Vec*line1Vec           #Float   (Dot Product)  Always >=0
        d = line0Vec*w                  #Float   (Dot Product)
        e = line1Vec*w                  #Float   (Dot Product)
        D = a*c - b*b                   #Float   (Dot Product)  Always >=0
    
        if D < rs.UnitAbsoluteTolerance():
            #the lines are parallel or very close to
            print "Parallel"
            return None
        else:
            sc = (b*e - c*d) / D
            tc = (a*e - b*d) / D
            
        point0 = line0.From + (sc * line0Vec)
        point1 = line1.From + (tc * line1Vec)
        dist = (point1 - point0).Length
        return dist, point0, point1

def lawCosinesAngle(a, b, c):
    #Uses law of cosines to solve the included angle between side a and side b
    return math.degrees(math.acos( ((a**2) + (b**2) - (c**2))/(2*a*b) ) )

def lawCosinesSide(a,b,C):
    #Uses law of cosines to solve the length of side c
    #the argument C is the included angle between side A and side B
    return math.sqrt( a**2 + b**2 -2*a*b*cos(C) )

def makeLinesDashed(lines, dashLength, gapLength, delete=True, startStyle=0):
    for line in lines:
        layer = rs.ObjectLayer(line)
        dashedLines = addDashedLines(rs.CurveStartPoint(line), rs.CurveEndPoint(line), dashLength, gapLength, startStyle)
        rs.ObjectLayer(dashedLines, layer)
        if delete: rs.DeleteObject(line)

def pointsAveragePos(points):
    points = rhutil.coerce3dpointlist(points)
    sum = Rhino.Geometry.Point3d(0.0,0.0,0.0)
    for point in points:
        sum += point
    return sum/len(points)

def unrollFaceWithLinetypes(targetLinePts, sourceFacePts, linetypes, dashLength, gapLength, startStyle=0):
    newFacePts = unrollFacePts(targetLinePts, sourceFacePts)
    #add the first point to the end of the array to allow curves to close
    newFacePts.append(newFacePts[0])
    #create a list to store the line guids that are produced
    unrollLines = []
    for i in range(len(newFacePts) - 1):
        if linetypes[i] == "S":
            #Line is solid
            try: unrollLines.append( rs.AddLine(newFacePts[i], newFacePts[i+1]) )
            except: pass
        elif linetypes[i] == "D":
            #Line is dashed
            unrollLines.extend( addDashedLines(newFacePts[i], newFacePts[i+1], dashLength, gapLength, startStyle) )
    list = [newFacePts, unrollLines]
    return list

def unrollFacePts(targetLinePts, sourceFacePts):
    #arguments
    #targetLinePts : [ [x,y,z], [x,y,z] ] Defines the line that the unrolled side will be based upon
    #sourceFacePts : [ [x,y,z], [x,y,z], [x,y,z], [x,y,z] ] The four points of the face to be unrolled

    sideLength00 = rs.Distance(sourceFacePts[0], sourceFacePts[1])
    sideLength01 = rs.Distance(sourceFacePts[1], sourceFacePts[2])
    if len(sourceFacePts) > 3:
        sideLength02 = rs.Distance(sourceFacePts[2], sourceFacePts[3])
        sideLength03 = rs.Distance(sourceFacePts[3], sourceFacePts[0])
    diagLength00 = rs.Distance(sourceFacePts[0], sourceFacePts[2])

    newFacePts = []

    #the new face point 00 is the first target point
    newFacePts.append( targetLinePts[0] )

    #find the new face point 01
    vect = rs.VectorCreate( targetLinePts[1], targetLinePts[0] )
    vect = vectorForceLength(vect, sideLength00)
    newFacePts.append( rs.VectorAdd( newFacePts[0], vect) )

    #find the new face point 02
    vect = vectorForceLength(vect, diagLength00)
    angle00 = lawCosinesAngle( sideLength00, diagLength00, sideLength01 )
    vect = rs.VectorRotate(vect, angle00, [0,0,1])
    newFacePts.append( rs.VectorAdd( newFacePts[0], vect) )

    if len(sourceFacePts) > 3:
        #find the new face point 03
        vect = vectorForceLength(vect, sideLength03)
        angle00 = lawCosinesAngle( sideLength03, diagLength00, sideLength02 )
        vect = rs.VectorRotate(vect, angle00, [0,0,1])
        newFacePts.append( rs.VectorAdd( newFacePts[0], vect) )

    return newFacePts

def vector3DFromLine(curveGUID):
    curve = rhutil.coercecurve(curveGUID, 0, True)
    startPt = curve.PointAtStart
    endPt = curve.PointAtEnd
    return Rhino.Geometry.Vector3d(endPt[0] - startPt[0], endPt[1] - startPt[1], endPt[2] - startPt[2])

def vectorForceLength(vect, length):
    rc = Rhino.Geometry.Vector3d(vect[0], vect[1], vect[2])
    rc.Unitize()
    return rc*length

def vectorsToVector3Ds(vectors):
    #converts one or more simple vectors into rhino commons vector3D instances
    try:
        test = vectors[0][0]
        #the input is a list of vectors
        vect3Ds = []
        for vect in vectors:
            vect3Ds.append( Rhino.Geometry.Vector3d(vect[0], vect[1], vect[2]) )
        return vect3Ds
    except:
        #the input is a single vector
        return Rhino.Geometry.Vector3d(vectors[0], vectors[1], vectors[2])

if (__name__ == "__main__"):
    pass
    """
    #test for unrollFaceWithLinetypes function
    facePts = [ [0,0,0], [10,0,0], [10,10,0], [0,10,0] ] 
    unrollFaceWithLinetypes( [ [0,-100,0], [100,-100,0] ], facePts, ["D","D","S","D"], 1.0, 0.5, 1)
    """