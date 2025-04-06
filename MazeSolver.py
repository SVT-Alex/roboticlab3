from irobot_edu_sdk.backend.bluetooth import Bluetooth
from irobot_edu_sdk.robots import event, hand_over, Color, Robot, Root, Create3
from irobot_edu_sdk.music import Note
from collections import deque

# robot is the instance of the robot that will allow us to call its methods and to define events with the @event decorator.
robot = Create3(Bluetooth())  # Will connect to the first robot found.

# === FLAG VARIABLES
HAS_COLLIDED = False
HAS_ARRIVED = False

# === MAZE DICTIONARY
N_X_CELLS = 3 # Size of maze (x dimension)
N_Y_CELLS = 3 # Size of maze (y dimension)
CELL_DIM = 50


# === DEFINING ORIGIN AND DESTINATION
PREV_CELL = None
START = (0,1)
CURR_CELL = START
DESTINATION = (1,0)
MAZE_DICT[CURR_CELL]["visited"] = True


# === PROXIMITY TOLERANCES
WALL_THRESHOLD = 45 #edit this number if you are having troubles seeing walls

# ==========================================================
# FAIL SAFE MECHANISMS

# EITHER BUTTON
@event(robot.when_touched, [True, True])  # User buttons: [(.), (..)]
async def when_either_button_touched(robot):
    global HAS_COLLIDED
    HAS_COLLIDED = True
    await robot.set_lights_rgb(255, 0, 0)
    await robot.stop()
    
# EITHER BUMPER
@event(robot.when_bumped, [True, True])  # [left, right]
async def when_either_bumped(robot):
    global HAS_COLLIDED
    HAS_COLLIDED = True
    await robot.set_lights_rgb(255, 0, 0)
    await robot.stop()
# ==========================================================
# Helper Functions

def createMazeDict(nXCells, nYCells, cellDim):
    mazeDict = {}
    for i in range(nXCells):
        for j in range(nYCells):
            mazeDict[(i, j)] = {
                'position': (i * cellDim, j * cellDim),
                'neighbors': [],
                'visited': False,
                'cost': 0
            }
    return mazeDict

def addAllNeighbors(mazeDict, nXCells, nYCells):
    for i in range(nXCells):
        for j in range(nYCells):
            neighbors = []

            if True:
                if i > 0:
                    neighbors.append((i - 1, j))
                if i + 1 < nXCells:
                    neighbors.append((i + 1, j))
                if j > 0:
                    neighbors.append((i, j - 1))
                if j + 1 < nYCells:
                    neighbors.append((i, j + 1))

            mazeDict[(i, j)]['neighbors'] = neighbors
    return mazeDict


def getRobotOrientation(heading):
    heading = heading % 360

    if heading >= 0 and heading < 45:
        return "E"
    if heading >= 315 and heading < 360:
        return "E"
    if heading >= 45 and heading < 135:
        return "N"
    if heading >= 135 and heading < 225:
        return "W"
    if heading >= 225 and heading < 315:
        return "S"

def getPotentialNeighbors(currentCell, orientation):
    x, y = currentCell
    if orientation == "N":
        return [(x - 1, y), (x, y + 1), (x + 1, y), (x, y - 1)]
    elif orientation == "S":
        return [(x + 1, y), (x, y - 1), (x - 1, y), (x, y + 1)]
    elif orientation == "E":
        return [(x, y + 1), (x + 1, y), (x, y - 1), (x - 1, y)]
    elif orientation == "W":
        return [(x, y - 1), (x - 1, y), (x, y + 1), (x + 1, y)]

def isValidCell(cellIndices, nXCells, nYCells):
    x, y = cellIndices

    if x < 0 or x >= nXCells:
        return False
    if y < 0 or y >= nYCells:
        return False

    return True

def getWallConfiguration(IR0, IR3, IR6, threshold):
    wallsDetected = []

    if IR0 > threshold:
        wallsDetected.append(True)
    else:
        wallsDetected.append(False)

    if IR3 > threshold:
        wallsDetected.append(True)
    else:
        wallsDetected.append(False)

    if IR6 > threshold:
        wallsDetected.append(True)
    else:
        wallsDetected.append(False)

    return wallsDetected

def getNavigableNeighbors(wallsAroundCell, potentialNeighbors, prevCell, nXCells, nYCells):
    navNeighbors = []

    for i in range(3):  # left, front, right
        if wallsAroundCell[i] == False:
            neighbor = potentialNeighbors[i]
            if isValidCell(neighbor, nXCells, nYCells):
                navNeighbors.append(neighbor)

    if prevCell != None:
        if isValidCell(prevCell, nXCells, nYCells):
            navNeighbors.append(prevCell)

    return navNeighbors


def updateMazeNeighbors(mazeDict, currentCell, navNeighbors):
    for cell in mazeDict:
        if currentCell in mazeDict[cell]['neighbors']:
            if cell not in navNeighbors:
                mazeDict[cell]['neighbors'].remove(currentCell)

    mazeDict[currentCell]['neighbors'] = navNeighbors
    return mazeDict

    
def getNextCell(mazeDict, currentCell):
    neighbors = mazeDict[currentCell]['neighbors']
    nextMove = None

    first = True
    for neighbor in neighbors:
        cost = mazeDict[neighbor]['cost']
        visited = mazeDict[neighbor]['visited']

        if visited == False:
            if first or cost < minCost:
                minCost = cost
                nextMove = neighbor
                first = False

    if nextMove == None:
        first = True
        for neighbor in neighbors:
            cost = mazeDict[neighbor]['cost']
            if first or cost < minCost:
                minCost = cost
                nextMove = neighbor
                first = False

    return nextMove


def checkCellArrived(currentCell, destination):
    return currentCell == destination

def printMazeGrid(mazeDict, nXCells, nYCells, attribute):
    for y in range(nYCells - 1, -1, -1):
        row = '| '
        for x in range(nXCells):
            cell_value = mazeDict[(x, y)][attribute]
            row += '{} | '.format(cell_value)
        print(row[:-1])

def updateMazeCost(mazeDict, start, goal):
    for (i,j) in mazeDict.keys():
        mazeDict[(i,j)]["flooded"] = False

    queue = deque([goal])
    mazeDict[goal]['cost'] = 0
    mazeDict[goal]['flooded'] = True

    while queue:
        current = queue.popleft()
        current_cost = mazeDict[current]['cost']

        for neighbor in mazeDict[current]['neighbors']:
            if not mazeDict[neighbor]['flooded']:
                mazeDict[neighbor]['flooded'] = True
                mazeDict[neighbor]['cost'] = current_cost + 1
                queue.append(neighbor)

    return mazeDict

# === BUILD MAZE DICTIONARY

MAZE_DICT = createMazeDict(N_X_CELLS, N_Y_CELLS, CELL_DIM)
MAZE_DICT = addAllNeighbors(MAZE_DICT, N_X_CELLS, N_Y_CELLS)

# ==========================================================
# EXPLORATION AND NAVIGATION

# === EXPLORE MAZE
async def navigateToNextCell(robot, nextCell, orientation):
    global MAZE_DICT, PREV_CELL, CURR_CELL, CELL_DIM

    x1, y1 = CURR_CELL
    x2, y2 = nextCell
    dx = x2 - x1
    dy = y2 - y1

    if orientation == "N":
        if dx == -1:
            await robot.turn_left(90)
        elif dx == 1:
            await robot.turn_right(90)
        elif dy == -1:
            await robot.turn_right(180)
    elif orientation == "S":
        if dx == -1:
            await robot.turn_right(90)
        elif dx == 1:
            await robot.turn_left(90)
        elif dy == 1:
            await robot.turn_right(180)
    elif orientation == "E":
        if dy == -1:
            await robot.turn_right(180)
        elif dx == -1:
            await robot.turn_right(90)
        elif dx == 1:
            await robot.turn_left(90)
    elif orientation == "W":
        if dy == 1:
            await robot.turn_right(180)
        elif dx == -1:
            await robot.turn_left(90)
        elif dx == 1:
            await robot.turn_right(90)

    await robot.move(CELL_DIM)
    MAZE_DICT[CURR_CELL]['visited'] = True
    PREV_CELL = CURR_CELL
    CURR_CELL = nextCell


@event(robot.when_play)
async def navigateMaze(robot):
    global HAS_COLLIDED, HAS_ARRIVED, CURR_CELL, PREV_CELL
    global START, DESTINATION, MAZE_DICT
    global N_X_CELLS, N_Y_CELLS, CELL_DIM, WALL_THRESHOLD

    while HAS_COLLIDED == False:
        if HAS_ARRIVED == True:
            break

        position = await robot.get_position()
        heading = await robot.get_heading()
        ir = await robot.get_ir_proximity()

        orientation = getRobotOrientation(heading)
        potentialNeighbors = getPotentialNeighbors(CURR_CELL, orientation)

        IR0 = ir["IR0"]
        IR3 = ir["IR3"]
        IR6 = ir["IR6"]

        walls = getWallConfiguration(IR0, IR3, IR6, WALL_THRESHOLD)

        navNeighbors = getNavigableNeighbors(
            walls,
            potentialNeighbors,
            PREV_CELL,
            N_X_CELLS,
            N_Y_CELLS
        )

        MAZE_DICT = updateMazeNeighbors(MAZE_DICT, CURR_CELL, navNeighbors)
        MAZE_DICT = updateMazeCost(MAZE_DICT, DESTINATION, CURR_CELL)

        if checkCellArrived(CURR_CELL, DESTINATION):
            HAS_ARRIVED = True
            await robot.set_lights_spinning(0, 255, 0)
            await robot.stop()
        else:
            nextCell = getNextCell(MAZE_DICT, CURR_CELL)
            await navigateToNextCell(robot, nextCell, orientation)


robot.play()
