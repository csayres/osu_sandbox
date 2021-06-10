import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt

gridFile = "fps_DesignReference.txt"

centerRow = 0
centerCol = 13


def parseDesignRef():
    holeType = []
    xPos = []
    yPos = []
    row = []
    col = []
    assign = []
    with open(gridFile, "r") as f:
        lines = f.readlines()
    for line in lines:
        line = line.strip()
        line = line.split("#")[0]
        if not line:
            continue
        _row, _col, x, y, fType = line.split()
        fType = fType.strip()
        if "Fiducial" in fType:
            fType = "Fiducial"
            assign.append(-1)  # fiducials get a -1 assignment
        else:
            assign.append(None)
        holeType.append(fType)
        xPos.append(float(x))
        yPos.append(float(y))
        row.append(int(_row))
        col.append(int(_col)+1)  #add 1 to all columns

    d = {}
    d["row"] = row
    d["column"] = col
    d["x"] = xPos
    d["y"] = yPos
    d["type"] = holeType
    d["assignment"] = assign

    df = pd.DataFrame(d)
    return df


designRef = parseDesignRef()

def updateAssignments(rows, cols, assignments):
    newDF = designRef.copy()
    for row, col, assign in zip(rows, cols, assignments):
        newDF["assignment"].loc[(newDF.row==row) & (newDF.column==col)] = assign
    newDF.dropna(inplace=True)
    newDF.reset_index(inplace=True)
    # with pd.option_context('display.max_rows', None, 'display.max_columns', None):  # more options can be specified also
    #     print(newDF)
    return newDF
    # import pdb; pdb.set_trace()


def getUWBench():
    rowAssign = []
    colAssign = []
    idAssign = []

    # top row
    rowAssign.append(2)
    colAssign.append(12)
    idAssign.append(-1) # FIF

    rowAssign.append(2)
    colAssign.append(13)
    idAssign.append(734) # FIF

    rowAssign.append(2)
    colAssign.append(14)
    idAssign.append(-1) # FIF

    # next row down
    rowAssign.append(1)
    colAssign.append(12)
    idAssign.append(428)

    rowAssign.append(1)
    colAssign.append(13)
    idAssign.append(561)

    rowAssign.append(1)
    colAssign.append(14)
    idAssign.append(594)

    rowAssign.append(1)
    colAssign.append(15)
    idAssign.append(649)

    #middle row
    rowAssign.append(0)
    colAssign.append(12)
    idAssign.append(-1) # FIF

    rowAssign.append(0)
    colAssign.append(13)
    idAssign.append(497)

    rowAssign.append(0)
    colAssign.append(14)
    idAssign.append(484)

    rowAssign.append(0)
    colAssign.append(15)
    idAssign.append(704)

    rowAssign.append(0)
    colAssign.append(16)
    idAssign.append(-1) # FIF

    # next row down
    rowAssign.append(-1)
    colAssign.append(12)
    idAssign.append(524)

    rowAssign.append(-1)
    colAssign.append(13)
    idAssign.append(645)

    rowAssign.append(-1)
    colAssign.append(14)
    idAssign.append(457)

    rowAssign.append(-1)
    colAssign.append(15)
    idAssign.append(705)

    # last row
    rowAssign.append(-2)
    colAssign.append(12)
    idAssign.append(-1) # FIF

    rowAssign.append(-2)
    colAssign.append(13)
    idAssign.append(566) # FIF

    rowAssign.append(-2)
    colAssign.append(14)
    idAssign.append(-1) # FIF

    return updateAssignments(rowAssign, colAssign, idAssign)

    print(uwBench)


uwBench = getUWBench()


def getOSU():
    osuAssignFile = "FPS_Sloan_Assignments_2021May14.csv"
    osuDF = pd.read_csv(osuAssignFile)
    rows = list(osuDF.Row)
    cols = list(osuDF.Column)
    dev = list(osuDF.DeviceID)
    idAssign = []

    rows = [int(x.strip("R")) for x in rows]
    cols = [int(x.strip("C")) for x in cols]
    for d in dev:
        if d.startswith("P"):
            idAssign.append(int(d.strip("P")))
        elif d.startswith("FTO"):
            idAssign.append(int(-1))
        else:
            raise RuntimeError("gah")

    return updateAssignments(rows, cols, idAssign)

# print(dR)

osuWok = getOSU()


def getOSUMini():
    osuAssignFile = "FPS_MiniWok_Assignments_2021Jun10.csv"
    osuDF = pd.read_csv(osuAssignFile)

    rows = list(osuDF.Row)
    cols = list(osuDF.Column)
    dev = list(osuDF.DeviceID)
    idAssign = []

    rows = [int(x.strip("R")) for x in rows]
    cols = [int(x.strip("C")) for x in cols]
    for d in dev:
        if d.startswith("P"):
            idAssign.append(int(d.strip("P")))
        elif d.startswith("FTO"):
            idAssign.append(int(-1))
        else:
            raise RuntimeError("gah")

    return updateAssignments(rows, cols, idAssign)

osuMini = getOSUMini()
#with pd.option_context('display.max_rows', None, 'display.max_columns', None):  # more options can be specified also
#    print(osuMini)


# sns.scatterplot(x="x", y="y", data=osuMini)
# osuWok.sort_values("assignment", ignore_index=True, inplace=True)
# posTable = osuWok[osuWok.assignment != -1]
# fidTable = osuWok[osuWok.assignment == -1]

# posXY = posTable[["x", "y"]].to_numpy()
# posID = posTable["assignment"].to_numpy(dtype=int)

# fidXY = fidTable[["x", "y"]].to_numpy()


# import pdb; pdb.set_trace()

#sns.scatterplot(x="x", y="y", style="type", data=osuMini)
#plt.axis("equal")
#plt.show()




