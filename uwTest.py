# NOTE: this has been tested with jaeger tag 0.4.2
# if things aren't working try ensuring you're using this jaeger version
import matplotlib
matplotlib.use("Agg")

import sys
import os
import asyncio
from jaeger import FPS, log
import matplotlib.pyplot as plt
# import matplotlib
import numpy
from collections import OrderedDict

from kaiju import RobotGrid

from trajPlotter import plotTraj
from movieExample import plotMovie

# from parseDesignRef import uwBench as assignmentTable
from parseDesignRef import osuWok as assignmentTable

# I modified loic's code a bit, hacking it onto the path here
# so we can get to it.

# matplotlib.use("TkAgg")

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
AngleToKaiju = -90  # degrees.

c90 = numpy.cos(numpy.radians(AngleToKaiju))
s90 = numpy.sin(numpy.radians(AngleToKaiju))
rot2kaiju = numpy.array([
    [c90, s90],
    [-s90, c90]
])


## sort grid by positioner id (because the comments say to do that?)
assignmentTable.sort_values("assignment", inplace=True, ignore_index=True)
posTable = assignmentTable[assignmentTable.assignment != -1]
fidTable = assignmentTable[assignmentTable.assignment == -1]

posXY = posTable[["x", "y"]].to_numpy()
posXY = (rot2kaiju @ posXY.T).T
posID = posTable["assignment"].to_numpy(dtype=int)

fidXY = fidTable[["x", "y"]].to_numpy()
fidXY = (rot2kaiju @ fidXY.T).T

# plt.figure()
# for _posID, (x,y) in zip(posID, posXY):
#     # print(posID, x,y)
#     plt.plot(x,y,"x")
# for x,y in fidXY:
#     print(x,y)
#     plt.plot(x,y,"o")
# for _posID, (x,y) in zip(posID, posXY):
#     # print(posID, x,y)
#     plt.text(x,y,"%i"%_posID)
# plt.xlim([-100,100])
# plt.ylim([-100,100])
# # plt.axis("equal")
# plt.savefig("rot.png", dpi=350)
# plt.close()

# import pdb; pdb.set_trace()


def gridInit(seed):
    hasApogee = True
    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)

    for _posID, (xp, yp) in zip(posID, posXY):
        rg.addRobot(_posID, xp, yp, hasApogee)
    fid = 1
    for xf, yf in fidXY:
        rg.addFiducial(fid, xf, yf)
        fid += 1
    rg.initGrid()
    return rg


###############################################################

def newGrid(seed=0, alphaLimit=None):
    """Return an initialized RobotGrid with positioners
    set to random target positions.  The same seed will produce the
    same grid always, as long as the kaiju parameters (eg collisionBuffer)
    are unchanged.  If you modify kaiju parameters you are not guarenteed
    to get the same grid (though there is a good chance you will).
    """

    rg = gridInit(seed)

    while True:
        for ii in posID:
            r = rg.getRobot(ii)
            r.setXYUniform()
        rg.decollideGrid()
        if alphaLimit is None:
            break
        # check all alphas are below limits
        alphaOK = True
        for r in rg.robotDict.values():
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
    rg = gridInit(seed)

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

    rg = gridInit(seed)

    for ii in range(rg.nRobots):
        r = rg.getRobot(ii)
        r.setAlphaBeta(alphaDeg, betaDeg)
    return rg


def getTargetPositions(rg):
    """Return a dictionary of target positions for each
    positioner.
    """
    targetPositions = OrderedDict()
    for r in rg.robotDict.values():
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

    for robotID, r in zip(posID, rg.robotDict.values()):

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


async def runPathPair(fps, seed=0, doComplicated=False, alphaLimit=None):
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
    gotoHome = [fps[rID].goto(alpha=0, beta=180) for rID in posID]
    await asyncio.gather(*gotoHome)

    # command the forward path
    print("forward path going")
    await fps.send_trajectory(forwardPath)


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

if __name__ == "__main__":
    alphaLimit = None
    seed = 0
    complicatedThreshold = 70
    while True:
        seed += 1
        rg = newGrid(seed, alphaLimit)
        try:
            forwardPath, reversePath = generatePath(rg, plot=False, movie=False, fileIndex=0)
        except:
            print("skipping deadlocked path")
            continue
        maxSteps = 0
        for abDict in forwardPath.values():
            nPts = len(abDict["beta"])
            if nPts > maxSteps:
                maxSteps = nPts
        if maxSteps > complicatedThreshold:
            print("found complicated path!", maxSteps)
            # exit function here
            break
    rg = newGrid(seed, alphaLimit)
    forwardPath, reversePath = generatePath(rg, plot=True, movie=True, fileIndex=0)

    # seed = None
    # doCentroid = False
    # continuous = True
    # unwindOnly = False
    # doComplicated = False
    # alphaLimit = 340 # degrees
    # asyncio.run(main(seed, doCentroid, continuous, unwindOnly, doComplicated, alphaLimit))



