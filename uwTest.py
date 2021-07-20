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
from parseDesignRef import uwBench as assignmentTable
import datetime

import traceback
import json

import pandas as pd

from baslerCam import BaslerCamera, BaslerCameraSystem, config

# I modified loic's code a bit, hacking it onto the path here
# so we can get to it.

# matplotlib.use("TkAgg")

########### Kaiju Parameters #####################
##################################################
Speed = 3               # RPM at output
angStep = 0.05          # degrees per step in kaiju's rough path
smoothPts = 5           # width of velocity smoothing window
epsilon = angStep * 2   # max error (deg) allowed in kaiju's path simplification
collisionBuffer = 2.4    # effective *radius* of beta arm in mm effective beta arm width is 2*collisionBuffer
collisionShrink = 0.05  # amount to decrease collisionBuffer by when checking smoothed and simplified paths
exptime = 10000 # micro seconds
nAvgImg = 20 # number of stack
CONTINUOUS = True
UNWINDONLY = False
TAKE_IMGS = False
LED_VALUE = 52
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

    for r in rg.robotDict.values():
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

def getTargetAlphaBeta(rg):
    """Return a dictionary of target positions for each
    positioner.
    """
    targetPositions = OrderedDict()
    posIDs = []
    alphas = []
    betas = []
    for r in rg.robotDict.values():
        posIDs.append(r.id)
        alphas.append(r.alpha)
        betas.append(r.beta)
    return posIDs, alphas, betas


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

        reversePath[int(robotID)] = armPathR

        # build forward path
        alphaTimesF = numpy.abs(alphaTimesR-alphaTimesR[-1])[::-1]
        alphaDegF = alphaDegR[::-1]
        betaTimesF = numpy.abs(betaTimesR-betaTimesR[-1])[::-1]
        betaDegF = betaDegR[::-1]

        armPathF = {}
        armPathF["alpha"] = [(pos, time) for pos, time in zip(alphaDegF, alphaTimesF)]
        armPathF["beta"] = [(pos, time) for pos, time in zip(betaDegF, betaTimesF)]


        forwardPath[int(robotID)] = armPathF

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
    for r in rg.robotDict.values():
        await fps.positioners[r.id].update_position()
        alpha, beta = fps.positioners[r.id].position
        r.setAlphaBeta(alpha, beta)

    for r in rg.robotDict.values():
        if rg.isCollided(r.id):
            print("robot ", r.id, " is collided")
    forwardPath, reversePath = generatePath(rg)

    await fps.send_trajectory(reversePath)

    # if fps.locked:
        #  await fps.unlock(force=true)
        # try to u

async def expose(cam, fps, data, kind, seed):

    on_value = 32 * int(1023 * (LED_VALUE / 100))
    off_value = 0
    device = fps.ieb.get_device("led1")

    tnow = datetime.datetime.now().isoformat()
    expOn = tnow + "_%s_%i_on.fits"%(kind, seed)
    expOff = tnow + "_%s_%i_off.fits"%(kind, seed)
    dataname = tnow + "_%s_%i.csv"%(kind, seed)
    #  turn off ieb light
    await device.write(on_value)
    await asyncio.sleep(0.5)

    exp = await cam.expose(exptime * 1e-6, stack=nAvgImg, filename=expOn)
    await exp.write()

    print("max counts", numpy.max(exp.data))

    await device.write(off_value)
    await asyncio.sleep(0.5)

    exp = await cam.expose(exptime * 1e-6, stack=nAvgImg, filename=expOff)
    await exp.write()

    data.to_csv(dataname, index=False)

async def runPathPair(fps, seed=0, doComplicated=False, alphaLimit=None, logfile=None, cam=None):
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
    pathfile = None
    if logfile:
        pathfile = "failedPaths" + logfile.strip(".log").strip("log2") + ".json"

    # minimum number of trajectory points for a complicated path
    complicatedThreshold = 70

    rg = newGrid(seed, alphaLimit)
    targetPositions = getTargetPositions(rg)
    posIDs, targAlphas, targBetas = getTargetAlphaBeta(rg)
    print("otachi, seed=%i, collisionBuffer=%.4f"%(seed, collisionBuffer))
    try:
        pathPair = generatePath(rg) # this will runtime error if failed
        forwardPath, reversePath = pathPair
        totalPathPoints = 0
        longestPathPoint = -1
        for posID, p in forwardPath.items():
            for x in ["alpha", "beta"]:
                pp = len(p[x])
                if pp > longestPathPoint:
                    longestPathPoint = pp
                totalPathPoints += pp
    except:
        print("skipping this path due to deadlock or initially collided orientation")
        return

    # if doComplicated:
    #     maxSteps = 0
    #     for abDict in forwardPath.values():
    #         nPts = len(abDict["beta"])
    #         if nPts > maxSteps:
    #             maxSteps = nPts
    #     if maxSteps < complicatedThreshold:
    #         print("skipping un interesting path")
    #         # exit function here
    #         return

    # send all to 0 180
    # gotoHome = [fps[rID].goto(alpha=0, beta=180) for rID in posID]
    # await asyncio.gather(*gotoHome)

    # take image of grid before motion
    reportAlpha = []
    reportBeta = []
    cmdAlpha = []
    cmdBeta = []
    robotID = []
    for r in rg.robotDict.values():
        await fps.positioners[r.id].update_position()
        alpha, beta = fps.positioners[r.id].position
        robotID.append(r.id)
        reportAlpha.append(alpha)
        reportBeta.append(beta)
        cmdAlpha.append(0)
        cmdBeta.append(180)

    d = {}
    d["reportAlpha"] = reportAlpha
    d["reportBeta"] = reportBeta
    d["cmdAlpha"] = cmdAlpha
    d["cmdBeta"] = cmdBeta
    d["robotID"] = robotID
    df = pd.DataFrame(d)

    if TAKE_IMGS:
        await expose(cam, fps, df, "fold", seed)
        await asyncio.sleep(0.5)


    # command the forward path
    print("forward path going")
    if logfile is not None:
        with open(logfile, "a") as f:
            f.write("totalPathPoints=%i, longestPathPoint=%i\n"%(totalPathPoints, longestPathPoint))
            f.write("begin forward seed=%i\n"%seed)

    try:
        await fps.send_trajectory(forwardPath)
    except Exception as e:
        if logfile is not None:
            with open(logfile, "a") as f:
                f.write("Forward Path EXCEPTION!!!\n%s\n"%str(e))
                traceback.print_exc(file=f)
                f.write("\n\ntotalPathPoints=%i, longestPathPoint=%i\n"%(totalPathPoints, longestPathPoint))
                with open(pathfile, 'w') as outfile:
                    print(pathPair)
                    json.dump(pathPair, outfile)
        raise e

    await asyncio.sleep(0.5)
    # take image of grid after motion
    reportAlpha = []
    reportBeta = []
    cmdAlpha = []
    cmdBeta = []
    robotID = []
    for rid, targAlpha, targBeta in zip(posIDs, targAlphas, targBetas):
        await fps.positioners[rid].update_position()
        alpha, beta = fps.positioners[rid].position
        robotID.append(rid)
        reportAlpha.append(alpha)
        reportBeta.append(beta)
        cmdAlpha.append(targAlpha)
        cmdBeta.append(targBeta)

    d = {}
    d["reportAlpha"] = reportAlpha
    d["reportBeta"] = reportBeta
    d["cmdAlpha"] = cmdAlpha
    d["cmdBeta"] = cmdBeta
    d["robotID"] = robotID
    df = pd.DataFrame(d)

    if TAKE_IMGS:
        await expose(cam, fps, df, "targ", seed)
        await asyncio.sleep(0.5)

    # print("unwinding grid")
    # await unwindGrid(fps)

    print("reverse path going")
    if logfile is not None:
        with open(logfile, "a") as f:
            f.write("begin reverse seed=%i\n"%seed)
    try:
        await fps.send_trajectory(reversePath)
    except Exception as e:
        if logfile is not None:
            with open(logfile, "a") as f:
                f.write("Reverse Path EXCEPTION!!!\n%s\n"%str(e))
                traceback.print_exc(file=f)
                f.write("\n\ntotalPathPoints=%i, longestPathPoint=%i\n"%(totalPathPoints, longestPathPoint))
                with open(pathfile, 'w') as outfile:
                    json.dump(pathPair, outfile)

        raise e




async def main(
        seed=None,
        continuous=False,
        unwindOnly=True,
        doComplicated=False,
        alphaLimit=None,
        ):

    if TAKE_IMGS:
        camID = 0
        bcs = BaslerCameraSystem(BaslerCamera, camera_config=config)
        sids = bcs.list_available_cameras()
        cam = await bcs.add_camera(uid=sids[camID], autoconnect=True)
    else:
        cam = None

    logtime = datetime.datetime.now().isoformat()
    logFile = "log_" + logtime + ".log"
    with open(logFile, "w") as f:
        f.write("#begin sequence\n")
    reconfigIter = 0

    fps = FPS()
    await fps.initialise()

    print("wait for initialize")
    await asyncio.sleep(5)
    print("done waiting")

    # first unwind the grid, to ensure we start from a reasonable point
    print("unwinding grid")
    with open(logFile, "a") as f:
        f.write("unwinding grid\n")
    try:
        await unwindGrid(fps)
    except Exception as e:
        with open(logFile, "a") as f:
            f.write("EXCEPTION!!!\n%s"%str(e))
        raise e

    print("unwind only", unwindOnly)
    if unwindOnly:
        print("unwind only, shutting down fps")
        await fps.shutdown()
        return

    trialNumber = 0
    # if seed is None, pick a random one
    # if continuous is on, this will be the starting point
    # seeds will be incremented
    if seed is None:
        seed = numpy.random.randint(0, 30000)
    # Print the status of positioner 4
    # print("FPS status", fps[robotID].status)
    while True:
        print("running pair of paths seed = %i"%seed)
        with open(logFile, "a") as f:
            f.write("running pair of paths seed=%i iter=%i dt=%s\n"%(seed, reconfigIter, datetime.datetime.now().isoformat()))

        await runPathPair(fps, seed, doComplicated, alphaLimit, logFile, cam)
        if fps.locked:
            print("FPS is locked! exiting\n")
            with open(logFile, "a") as f:
                f.write("FPS is locked! exiting\n")
            break
        if not continuous:
            break
        trialNumber += 1
        seed += 1
        print(f"trial number: {trialNumber}")
        # Cleanly finish all pending tasks and exit
        reconfigIter += 1
    await fps.shutdown()

if __name__ == "__main__":
    seed = None
    unwindOnly = False
    doComplicated = False
    alphaLimit = None # degrees
    asyncio.run(main(seed, CONTINUOUS, UNWINDONLY, doComplicated, alphaLimit))

