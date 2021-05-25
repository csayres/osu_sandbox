# NOTE: this has been tested with jaeger tag 0.4.2
# if things aren't working try ensuring you're using this jaeger version

import sys
import os
import asyncio
from jaeger import FPS, log
import matplotlib.pyplot as plt
import matplotlib
import numpy
from collections import OrderedDict

from kaiju import RobotGrid

from trajPlotter import plotTraj
from movieExample import plotMovie

from parseDesignRef import uwBench as assignmentTable

# I modified loic's code a bit, hacking it onto the path here
# so we can get to it.

matplotlib.use("TkAgg")

########### Kaiju Parameters #####################
##################################################
Speed = 3               # RPM at output
angStep = 0.05          # degrees per step in kaiju's rough path
smoothPts = 5           # width of velocity smoothing window
epsilon = angStep * 2   # max error (deg) allowed in kaiju's path simplification
collisionBuffer = 2.5     # effective *radius* of beta arm in mm effective beta arm width is 2*collisionBuffer
collisionShrink = 0.02  # amount to decrease collisionBuffer by when checking smoothed and simplified paths
##################################################

################ Coordinates ####################
#################################################

# Define the angle from the coordinate system in which the
# robot coordinates are given to Kaiju's coordinate system
# Kaiju's definition is:
# +x is in the direction of alpha = 0
# +y is in the direction of alpha = 90
AngleToKaiju = 90  # degrees.

c90 = numpy.cos(numpy.radians(AngleToKaiju))
s90 = numpy.sin(numpy.radians(AngleToKaiju))
rot2kaiju = numpy.array([
    [c90, s90],
    [-s90, c90]
])
rot2image = numpy.array([
    [c90, -s90],
    [s90, c90]
])


posDict = OrderedDict()
# modify the coordinates below with every new calibration
# these are calibration outputs measured in mm from top left
# looking at the robot grid
# (** NOTE THE NEGATIVE Y VALUES!)
# (** NOTE put positioners in ascending order of ID)

posDict[23] = numpy.array([50.17, -65.85])
posDict[242] = numpy.array([72.70, -65.80])
posDict[272] = numpy.array([83.99, -46.86])
posDict[308] = numpy.array([39.11, -46.09])
posDict[315] = numpy.array([50.48, -26.87])
posDict[359] = numpy.array([72.92, -27.10])
posDict[365] = numpy.array([61.61, -46.32])



centerXYMM = posDict[centerPositioner]
# rotate grid such that it is aligned with
# kaiju's definition (alpha=0 is aligned with +x)
for key in posDict.keys():
    fromMiddle = posDict[key] - centerXYMM
    posDict[key] = numpy.dot(fromMiddle, rot2kaiju)


## sort grid by positioner id (because the comments say to do that?)
assignmentTable.sort_by("assignment", inplace=True, ignore_index=True)
posTable = assignmentTable[assignmentTable.assignment != -1]
fidTable = assignmentTable[assignmentTable.assignment == -1]

posXY = posTable[["x", "y"]].to_numpy()
posID = posTable["assignment"].to_numpy(dtype=int)

fidXY = fidTable[["x", "y"]].to_numpy()

import pdb; pdb.set_trace()


###############################################################

def newGrid(seed=0, alphaLimit=None):
    """Return an initialized RobotGrid with positioners
    set to random target positions.  The same seed will produce the
    same grid always, as long as the kaiju parameters (eg collisionBuffer)
    are unchanged.  If you modify kaiju parameters you are not guarenteed
    to get the same grid (though there is a good chance you will).
    """
    hasApogee = True
    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)

    for posID, (xp, yp) in posDict.items():
        rg.addRobot(posID, xp, yp, hasApogee)
    rg.initGrid()

    while True:
        for ii in range(rg.nRobots):
            r = rg.getRobot(ii)
            r.setXYUniform()
        rg.decollide2()
        if alphaLimit is None:
            break
        # check all alphas are below limits
        alphaOK = True
        for r in rg.allRobots:
            if r.alpha > alphaLimit:
                print("alpha over limits, trying again")
                alphaOK = False
                break
        if alphaOK:
            break
    return rg


def homeGrid():
    """Return an initialized RobotGrid with target positions of
    alpha=0 beta=180 for all positioners.
    """
    alphaTarget = 0
    betaTarget = 180
    hasApogee = True
    seed = 0
    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)

    for posID, (xp, yp) in posDict.items():
        rg.addRobot(posID, xp, yp, hasApogee)
    rg.initGrid()

    for ii in range(rg.nRobots):
        r = rg.getRobot(ii)
        r.setAlphaBeta(alphaTarget, betaTarget)
    return rg


def safeGrid(alphaDeg, betaDeg):
    """Return a RobotGrid with all positioners set to the same alpha/beta
    provided as input.
    """
    hasApogee = True
    seed = 0
    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)

    for posID, (xp, yp) in posDict.items():
        rg.addRobot(posID, xp, yp, hasApogee)
    rg.initGrid()

    for ii in range(rg.nRobots):
        r = rg.getRobot(ii)
        r.setAlphaBeta(alphaDeg, betaDeg)
    return rg

def getTargetPositions(rg):
    """Return a dictionary of target positions for each
    positioner.
    """
    targetPositions = OrderedDict()
    for r in rg.allRobots:
        x, y, z = r.metFiberPos
        targetPositions[r.id] = [x, y]
    return targetPositions


def generatePath(rg, plot=False, movie=False, fileIndex=0):
    """Run the path generator for an initialized RobotGrid rg.
    Extra params are for plotting
    """
    rg.pathGen()
    if rg.didFail:
        print("path gen failed, either experienced a deadlock, or starting position is collided")
        raise(RuntimeError, "path gen failed, either experienced a deadlock, or starting position is collided")
    rg.smoothPaths(smoothPts)
    rg.simplifyPaths()
    rg.setCollisionBuffer(collisionBuffer - collisionShrink)
    rg.verifySmoothed()

    if rg.smoothCollisions:
        print("smoothing failed with %i collisions, try increasing the collisionBuffer and collisionShrink parameters", rg.smoothCollisions)
        raise(RuntimeError, "smoothing failed")

    if movie:
        plotMovie(rg, filename="movie_%i"%fileIndex)

    # find the positioner with the most interpolated steps
    forwardPath = {}
    reversePath = {}

    for robotID, r in zip(posDict.keys(), rg.allRobots):

        assert robotID == r.id
        if plot:
            plotTraj(r, "seed_%i_"%fileIndex, dpi=250)

        # bp = numpy.array(r.betaPath)
        # sbp = numpy.array(r.interpSmoothBetaPath)
        ibp = numpy.array(r.simplifiedBetaPath)

        # ap = numpy.array(r.alphaPath)
        # sap = numpy.array(r.interpSmoothAlphaPath)
        iap = numpy.array(r.simplifiedAlphaPath)

        # generate kaiju trajectory (for robot 23)
        # time = angStep * stepNum / speed

        alphaTimesR = iap[:,0] * angStep / (Speed*360/60.)
        alphaDegR = iap[:,1]
        betaTimesR = ibp[:,0] * angStep / (Speed*360/60.)
        betaDegR = ibp[:,1]


        # add time buffer for the reverse path, because we are
        # iterating final placement after the forward path
        # this ensures the robot first moves to the position
        # computed by kaiju before following the folding path.
        armPathR = {} # reverse path
        armPathR["alpha"] = [(pos, time+0.5) for pos, time in zip(alphaDegR, alphaTimesR)]
        armPathR["beta"] = [(pos, time+0.5) for pos, time in zip(betaDegR, betaTimesR)]

        reversePath[robotID] = armPathR

        # build forward path
        alphaTimesF = numpy.abs(alphaTimesR-alphaTimesR[-1])[::-1]
        alphaDegF = alphaDegR[::-1]
        betaTimesF = numpy.abs(betaTimesR-betaTimesR[-1])[::-1]
        betaDegF = betaDegR[::-1]

        armPathF = {}
        armPathF["alpha"] = [(pos, time) for pos, time in zip(alphaDegF, alphaTimesF)]
        armPathF["beta"] = [(pos, time) for pos, time in zip(betaDegF, betaTimesF)]


        forwardPath[robotID] = armPathF

    return forwardPath, reversePath

def centroid(imgData, positionerTargetsMM, plot=False):
    """Detect and measure centroids based on the input imageData and
    positionerTargetsMM (targets are given in MM in Kaiju's reference frame)

    (** Note Rows of the image are immediately inverted such that 0,0 corresponds
    to the lower left of the grid when looking at it)

    return a dictionary keyed by positioner ID of XY offsets to apply in
    mm in kaiju's coordinate system.
    """
    roiRadiusPx = int(numpy.floor(roiRadiusMM / scaleFac))
    imgData = imgData[::-1,:] # invert the rows of the image
    numCols, numRows = imgData.shape
    mask = numpy.zeros(imgData.shape) + 1
    # build the mask, draw squares around expected positions
    positionerTargetsPx = OrderedDict()
    for posID, xyKaijuMM in positionerTargetsMM.items():
        # take abs value of positioner because it's y axis is defined
        # negative (loic's positions are measured from top left)
        xyImageMM = numpy.dot(xyKaijuMM, rot2image) + numpy.abs(centerXYMM)
        xTargetPx, yTargetPx = xyImageMM / scaleFac
        positionerTargetsPx[posID] = numpy.array([xTargetPx, yTargetPx])
        # rotate into reference frame with 0,0 at bottom left
        xROI = numpy.int(numpy.floor(xTargetPx))
        yROI = numpy.int(numpy.floor(yTargetPx))
        startRow = xROI - roiRadiusPx
        if startRow < 0:
            startRow = 0
        endRow = xROI + roiRadiusPx
        if endRow > imgData.shape[1]:
            endRow = imgData.shape[1]

        startCol = yROI - roiRadiusPx
        if startCol < 0:
            startCol = 0
        endCol = yROI + roiRadiusPx
        if endCol > imgData.shape[0]:
            endCol = imgData.shape[0]

        mask[startCol:endCol, startRow:endRow] = 0

    # imshow defaults to -0.5, -0.5 for origin, set this to 0,0
    # plot the mask used too make it positive valued so it shows up
    if plot:
        plt.imshow(imgData + numpy.abs(mask-1)*200, origin="lower", extent=(0, numRows, 0, numCols))

    # find all the centroids, and loop through and plot them
    ctrDataList, imStats = PyGuide.findStars(
        data = imgData,
        mask = mask,
        satMask = None,
        thresh=detectThresh,
        ccdInfo = CCDInfo,
        verbosity = 0,
        doDS9 = False,
    )[0:2]
    centroidsPx = []
    for ctrData in ctrDataList:
        # need to index explicity because ctrData is actually an object
        centroidsPx.append(ctrData.xyCtr)
        xyCtr = ctrData.xyCtr
        rad = ctrData.rad
        counts = ctrData.counts
        if plot:
            plt.plot(xyCtr[0], xyCtr[1], 'or', markersize=10, fillstyle="none")#, alpha=0.2)
        # print("star xyCtr=%.2f, %.2f, radius=%s counts=%.2f" % (xyCtr[0], xyCtr[1], rad, counts))
    # plot the desired targets
    centroidsPx = numpy.asarray(centroidsPx)
    nTargs = len(positionerTargetsPx.values())
    nCentroids = len(centroidsPx)

    if plot:
        for posID, (xTargetPx, yTargetPx) in positionerTargetsPx.items():
            plt.plot(xTargetPx, yTargetPx, 'xr', markersize=10)
        plt.show()
        plt.close()

    # calculate distances between all targets and all centroids
    if nCentroids > nTargs:
        # don't allow false positives
        raise RuntimeError("more centroids than targets")
    if nCentroids < nTargs:
        #allow missing centroids
        print("warning: more targets than centroids")
    if nCentroids == 0:
        raise RuntimeError("didn't find any centroids")

    # print("distMat shappe", distMat.shape)
    targArrayPx = numpy.array(list(positionerTargetsPx.values()))
    targIdArray = list(positionerTargetsPx.keys())
    # for each centroid give it a target
    cent2target = [] # holds centroidIndex, targetIndex, and distance to target in px
    for centInd, cent in enumerate(centroidsPx):
        # a row of disntances for this target
        distArr = numpy.array([numpy.linalg.norm(targ-cent) for targ in targArrayPx])
        targInd = numpy.argmin(distArr)
        cent2target.append([centInd, targInd, distArr[targInd]])
    cent2target = numpy.array(cent2target)
    # for paranoia, remove any targets with distance greater than the ROI,
    # not sure this could happen but check anyways
    cent2target = cent2target[cent2target[:,2] < roiRadiusPx]

    # calculate the offsets (vector from centroid to target)
    # in kaiju's reference frame in mm
    positionerOffsets = OrderedDict()
    for cInd, tInd, dist in cent2target:
        tInd = int(tInd)
        cInd = int(cInd)
        posID = targIdArray[tInd]
        targPix = targArrayPx[tInd]
        centPix = centroidsPx[cInd]
        offPx = targPix - centPix
        offMM = numpy.dot(offPx * scaleFac, rot2kaiju)
        positionerOffsets[posID] = offMM

    return positionerOffsets

async def unwindGrid(fps):
    """Unwind the positioners from any starting point.
    Positioners are queried for position, kaiju builds a reverse path for
    them, the fps commands it.  A runtime error is raised if a path cannot
    be found.

    """

    rg = homeGrid() # get a grid
    # overwrite the positions to the positions that the robots
    # are reporting
    for ii in range(rg.nRobots):
        r = rg.getRobot(ii)
        await fps.positioners[r.id].update_position()
        alpha, beta = fps.positioners[r.id].position
        r.setAlphaBeta(alpha, beta)

    forwardPath, reversePath = generatePath(rg)

    await fps.send_trajectory(reversePath)

async def cameraFeedback(rg, fps, targetPositions):
    """Implement a sequence of correction moves based on camera feedback
    """
    positionerAlphaBeta = OrderedDict()
    for r in rg.allRobots:
        positionerAlphaBeta[r.id] = [r.alphaPath[0][1], r.betaPath[0][1]]

    for posIter in range(5):
        # measure the positions of all the guys use 4 images for centroid
        imgDataList = []
        for imageIter in range(nImgAvg):
            imgDataList.append(csCam.camera.getImage())
        # centroid on the average image
        imgData = numpy.sum(imgDataList, axis=0) / nImgAvg
        positionerOffsets = centroid(imgData, targetPositions, plot=False)
        # print the position errors in microns
        for r in rg.allRobots:
            try:
                xOffMM, yOffMM = positionerOffsets[r.id]
            except:
                print("no found offset for robot %i, skipping correction")
                continue

            print("iter %i: robot %i has error of %.2f microns"%(posIter, r.id, numpy.linalg.norm([xOffMM, yOffMM]) * 1000 ))
            # figure out alpha beta offset for this positioner
            alphaLast, betaLast = positionerAlphaBeta[r.id]
            r.setAlphaBeta(alphaLast, betaLast)
            xFiberLast, yFiberLast, zFiberLast = r.metFiberPos
            # add fiber offset
            xOff, yOff = positionerOffsets[r.id]
            nextFiberX = xFiberLast + xOff
            nextFiberY = yFiberLast + yOff
            metFiberID = 0
            nextAlpha, nextBeta = r.alphaBetaFromFiberXY(nextFiberX, nextFiberY, metFiberID)
            if numpy.isnan(nextAlpha) or numpy.isnan(nextBeta):
                # positioner can't reach desired offset don't do anything
                print("desired offset is out of range, not applying")
                continue
            alphaOff = nextAlpha - alphaLast
            betaOff = nextBeta - betaLast
            if numpy.max(numpy.abs([alphaOff, betaOff])) > 10:
                print("max alpha beta offsets are too high: %.2f, %.2f"%(alphaOff, betaOff))
                continue
            positionerAlphaBeta[r.id] = [nextAlpha, nextBeta]
            await fps[r.id].goto(alpha=nextAlpha, beta=nextBeta)

async def runPathPair(fps, seed=0, doComplicated=False, doCentroid=False, alphaLimit=None):
    """Command FPS to do 3 moves:
    1st move: all positioners to 0, 180 (warning, no check for safety is done!)
    2nd move: forward path to targets found based on seed input
    3rd move: reverse path from targets back to 0, 180

    If doCentroid is True, camera feedback is enabled at the target position
    If doComplicated is True, this function will run the path if its complicated.
    A complicated path is if at least one positioner has > 50 points.  Note that
    if you increase smoothing or epsilon, then you may not get any complicated
    paths.  Right now the complicated threshold is set to 50
    """
    # minimum number of trajectory points for a complicated path
    complicatedThreshold = 50

    rg = newGrid(seed, alphaLimit)
    targetPositions = getTargetPositions(rg)
    print("otachi, seed=%i, collisionBuffer=%.4f"%(seed, collisionBuffer))
    try:
        forwardPath, reversePath = generatePath(rg) # this will runtime error if failed
    except:
        print("skipping this path due to deadlock or initially collided orientation")
        return

    if doComplicated:
        maxSteps = 0
        for abDict in forwardPath.values():
            nPts = len(abDict["beta"])
            if nPts > maxSteps:
                maxSteps = nPts
        if maxSteps < complicatedThreshold:
            print("skipping un interesting path")
            # exit function here
            return

    # send all to 0 180
    gotoHome = [fps[rID].goto(alpha=0, beta=180) for rID in posDict.keys()]
    await asyncio.gather(*gotoHome)

    # command the forward path
    print("forward path going")
    await fps.send_trajectory(forwardPath)


    if doCentroid:
        await cameraFeedback(rg, fps, targetPositions)

    print("reverse path going")
    await fps.send_trajectory(reversePath)



async def main(
        seed=None,
        doCentroid=False,
        continuous=False,
        unwindOnly=False,
        doComplicated=False,
        alphaLimit=None,
        ):
    fps = FPS()
    await fps.initialise()

    # first unwind the grid, to ensure we start from a reasonable point
    print("unwinding grid")
    await unwindGrid(fps)

    print("unwind only", unwindOnly)
    if unwindOnly:
        print("unwind only, shutting down fps")
        await fps.shutdown()
        return

    trialNumber = 164323
    # if seed is None, pick a random one
    # if continuous is on, this will be the starting point
    # seeds will be incremented
    if seed is None:
        seed = numpy.random.randint(0, 30000)
    # Print the status of positioner 4
    # print("FPS status", fps[robotID].status)
    while True:
        print("running pair of paths seed = %i"%seed)
        await runPathPair(fps, seed, doComplicated, doCentroid, alphaLimit)
        if fps.locked:
            print("FPS is locked! exiting\n")
            break
        if not continuous:
            break
        trialNumber += 1
        seed += 1
        print(f"trial number: {trialNumber}")
        # Cleanly finish all pending tasks and exit
    await fps.shutdown()

# if __name__ == "__main__":
    # seed = None
    # doCentroid = False
    # continuous = True
    # unwindOnly = False
    # doComplicated = False
    # alphaLimit = 340 # degrees
    # asyncio.run(main(seed, doCentroid, continuous, unwindOnly, doComplicated, alphaLimit))



