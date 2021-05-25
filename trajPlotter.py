
# note for the moment must set to commit ae7c7e7 'mostly working assingment tools'
from subprocess import Popen
import time
from multiprocessing import Pool, cpu_count
import glob
import os
import numpy

import matplotlib.pyplot as plt
from shapely.geometry import LineString
from descartes import PolygonPatch

from kaiju import utils, RobotGrid

nDia = 3
pitch = 22.4

# angStep = 1
# smoothPts = 3 # 101 for 0.05
# epsilon = angStep * 2

angStep = .05
smoothPts = 5*5
epsilon = angStep * 2

collisionBuffer = 2
collisionShrink = 0.02
hasApogee = True

def plotTraj(r, figprefix="traj_", dpi=500):
    # r is a robot
    spa = numpy.array(r.smoothedAlphaPath)
    spb = numpy.array(r.smoothedBetaPath)
    rpa = numpy.array(r.alphaPath)
    rpb = numpy.array(r.betaPath)
    aRDP = numpy.array(r.simplifiedAlphaPath);
    bRDP = numpy.array(r.simplifiedBetaPath);


    av = numpy.array(r.alphaVel)
    bv = numpy.array(r.betaVel)
    vSteps = numpy.arange(len(av))
    sav = numpy.array(r.smoothAlphaVel)
    sbv = numpy.array(r.smoothBetaVel)
    ss = numpy.arange(len(sav))

    # print("plotting", r.id)
    # print("alpha start", rpa[0,:] - aRDP[0,:])
    # print("alpha end", rpa[-1,:] - aRDP[-1,:])
    # print("beta start", rpb[0,:] - bRDP[0,:])
    # print("beta end", rpb[-1,:] - bRDP[-1,:])

    fig, ax = plt.subplots(2,1, figsize=(10,10))


    ax[0].plot(rpa[:,0], rpa[:,1], linewidth=0.2, label="rough alpha", alpha=0.8)
    ax[0].plot(rpb[:,0], rpb[:,1], linewidth=0.2, label="rough beta", alpha=0.8)
    ax[0].plot(spa[:,0], spa[:,1], 'k-', linewidth=0.2, label="smooth alpha")
    ax[0].plot(spb[:,0], spb[:,1], 'k-', linewidth=0.2, label="smooth beta")
    ax[0].plot(aRDP[:,0], aRDP[:,1], 'oc-', linewidth=0.2, markeredgewidth=0.4, fillstyle="none", markersize=2, label="RDP alpha", alpha=0.7)
    ax[0].plot(bRDP[:,0], bRDP[:,1], 'oc-', linewidth=0.2, markeredgewidth=0.4, fillstyle="none", markersize=2, label="RDP beta", alpha=0.7)
    ax[0].legend()

    ax[1].plot(vSteps, av, linewidth=0.2, label="alphaVel", alpha=0.4)
    ax[1].plot(vSteps, bv, linewidth=0.2, label="betaVel", alpha=0.4)
    ax[1].plot(ss, sav, 'k-', linewidth=0.2, label="smoothAlpha")
    ax[1].plot(ss, sbv, 'k-', linewidth=0.2, label="smoothBeta")

    ax[1].legend()
    # plt.legend()


    plt.savefig(figprefix+"robot_%s.png"%r.id, dpi=dpi)
    plt.close()

# maxPathSteps = int(700.0/angStep)
if __name__ == "__main__":
    minPoints = 30
    cos = numpy.cos(numpy.radians(90))
    sin = numpy.sin(numpy.radians(90))
    seed = 5000

    while True:
        seed += 1
        print("trying seed", seed)
        rg = RobotGrid(angStep, collisionBuffer, epsilon, seed)
        xPos, yPos = utils.hexFromDia(nDia, pitch=pitch)
        for ii, (xp,yp) in enumerate(zip(xPos,yPos)):
            xrot = cos * xp + sin * yp
            yrot = sin * xp - cos * yp
            # print("%.8f, %.8f"%(xrot,yrot))
            rg.addRobot(ii, xrot, yrot, hasApogee)
        rg.initGrid()
        for ii in range(rg.nRobots):
            r = rg.getRobot(ii)
            r.setXYUniform()
        # set all positioners randomly (they are initialized at 0,0)
        rg.decollide2()
        rg.pathGen()
        rg.smoothPaths(smoothPts) # must be odd
        rg.simplifyPaths()
        rg.setCollisionBuffer(collisionBuffer-collisionShrink)
        rg.verifySmoothed()

        # find max beta path points
        # only plot if we've found minimum
        # amount of points
        maxPts = 0
        for r in rg.allRobots:
            nPts = len(r.simplifiedBetaPath)
            if nPts > maxPts:
                maxPts = nPts

        print("maxPts", maxPts)
        if maxPts < minPoints:
            continue

        for r in rg.allRobots:
            plotTraj(r)
        break


