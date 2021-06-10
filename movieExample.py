
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

from collections import OrderedDict
import functools

def plotFirst(stepInd, rg, stepsFirst, lims):
    step = int(stepsFirst[stepInd])
    fig = plt.figure(figsize=(9,10))
    ax = fig.add_axes([0,0,1,1])
    for robot in rg.robotDict.values():
        alphaX = robot.roughAlphaX[step][1]
        alphaY = robot.roughAlphaY[step][1]
        betaX = robot.roughBetaX[step][1]
        betaY = robot.roughBetaY[step][1]
        ax.plot([robot.xPos, alphaX], [robot.yPos, alphaY], color='black', linewidth=2, alpha=0.5)

        topCollideLine = LineString(
            [(alphaX, alphaY), (betaX, betaY)]
        ).buffer(rg.collisionBuffer, cap_style=1)
        topcolor = 'blue'
        edgecolor = 'black'
        patch = PolygonPatch(topCollideLine, fc=topcolor, ec=edgecolor, alpha=0.5, zorder=10)
        ax.add_patch(patch)
    ax.set_aspect('equal', 'box')
    ax.set_xlim([-lims, lims])
    ax.set_ylim([-lims, lims])
    ax.set_xticks([])
    ax.set_yticks([])
    ax.axis('off')
    print("fig", step)
    plt.savefig("step_%04d.png"%(stepInd), dpi=250)
    plt.close()


def plotMovie(rg, filename="movie"):
    steps1deg = numpy.floor(rg.nSteps * rg.angStep)
    stepsFirst = numpy.arange(steps1deg+1) / rg.angStep
    # partial = functools.partial(plotFirst, rg=rg, stepsFirst=stepsFirst)
    # p = Pool(cpu_count())
    # p.map(partial, range(len(stepsFirst)))
    xs = [r.xPos for r in rg.robotDict.values()]
    ys = [r.yPos for r in rg.robotDict.values()]
    maxX = numpy.max(numpy.abs(xs))
    maxY = numpy.max(numpy.abs(ys))
    lims = numpy.max([maxX, maxY]) + 22.4
    print("lims", lims)
    for ii in range(len(stepsFirst)):
        plotFirst(ii, rg, stepsFirst, lims)



    fps = 30 # frames per second
    args = ['ffmpeg', '-r', '%i'%fps, '-f', 'image2', '-i', 'step_%04d.png',
            '-pix_fmt', 'yuv420p', '%s.mp4'%filename]

    movie = Popen(args)
    movie.wait()

    # clean up imgs
    imgs = glob.glob("step*.png")
    for img in imgs:
        os.remove(img)

if __name__ == "__main__":
    nDia = 27
    pitch = 22.4
    angStep = .5
    collisionBuffer = 1.5
    epsilon = angStep * 2.2
    seed1 = 0
    seed2 = 1
    hasApogee = True
    figOffset = 0
    # maxPathSteps = int(700.0/angStep)
    posDict = OrderedDict()
    posDict[24] = [0, -22.4]
    posDict[17] = [0, 0]
    posDict[21] = [0, 22.4]
    posDict[19] = [19.39896904, -11.20000000]
    posDict[20] = [19.39896904, 11.20000000]
    posDict[16] = [-19.39896904, -11.20000000]
    posDict[25] = [-19.39896904, 11.20000000]

    rg = RobotGrid(angStep, collisionBuffer, epsilon, seed1)
    for posID, (xp, yp) in posDict.items():
        rg.addRobot(posID, xp, yp, hasApogee)

    # xPos, yPos = utils.hexFromDia(nDia, pitch=pitch)
    # for ii, (xp,yp) in enumerate(zip(xPos,yPos)):
    #     rg.addRobot(ii, xp, yp, hasApogee)
    rg.initGrid()
    for ii in range(rg.nRobots):
        r = rg.getRobot(ii)
        r.setXYUniform()
    # set all positioners randomly (they are initialized at 0,0)
    rg.decollide2()
    rg.pathGen()
    plotMovie(rg)
