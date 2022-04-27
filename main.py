from graphics import *
from tkinter import messagebox
import math
resx = 22 #cells on x axis
resy = 22 #cells on y axis
pixW = 25 #width/height of cells
win = GraphWin("test", resx*pixW,resy*pixW) #initiate window
w = win.getWidth() #width in pixels
h = win.getHeight() #height in pixels
control = tk.Tk() #initiate control window
state = "walls" #containing what to do when a cell is clicked on
control.geometry("200x250") #dimensions of control window
waypoints = {} #containing waypoints, name and coordinate
paths = [] #list of paths between waypoints (elements of type Path)
selectedNode = [-1,-1] #used for selecting nodes
wp1 = '' #holds name of first selected waypoint
gridlist = [] #the map - 0 is free, 1 is wall

#holds information about the path between two waypoints
class Path():
    def __init__(self, w1, namew1, w2, namew2, helpnodes):
        self.w1 = w1 #coordinate of first waypoint
        self.namew1 = namew1 #name of first waypoint
        self.w2 = w2 #coordinate of second waypoint
        self.namew2 = namew2 #name of second waypoint
        self.helpnodes = helpnodes #list of helpnodes that makes the path a series of straight lines

#holds information about a node in the pathfinding
class Searchnode():
    def __init__(self,parent, pos):
        self.pos = pos #coordinate
        self.parent = parent #which node this node is expanded from
        #cost values
        self.g=0
        self.h=0
        self.f=0
        
def getPos(nodelist): #takes a list of Searchnode objects and returns a list of their coordinates
    poslist = []
    for item in nodelist:
        poslist.append(item.pos)
    return poslist

def testPath(start, end, path, simple): #checks if two nodes, start and end, are connected by a straight line in path
    #start and end are the start and end nodes we wish to check
    #path is an ordered list of Searchnodes
    #if simple==x, then it checks if start and end are connected by a straight vertical line
    #if simple==y, then it checks if start and end are connected by a straight horizontal line
    pathlist = getPos(path)
    startindex = pathlist.index(start.pos)
    endindex = pathlist.index(end.pos)
    onSimple = False
    if simple == "x":
        for i in range(startindex, endindex):
            if path[startindex].pos[0] == path[i].pos[0]:
                onSimple = True
            else:
                return False
    elif simple == "y":
        for i in range(startindex, endindex):
            if path[startindex].pos[1] == path[i].pos[1]:
                onSimple = True
            else:
                return False
def astar(start, end, gridlist):
    global resx
    global resy
    neighbourindex = [[0,1], [1,0], [0,-1],[-1,0]] #list of directions to expand
    openlist = [] #nodes that need to be expanded
    closedlist = [] #nodes that have been expanded
    startnode = Searchnode(None, start) #define start node
    endnode = Searchnode(None, end) #define end node
    startnode.g=startnode.h=startnode.f=0 #define cost for startnode
    endnode.g=endnode.h=endnode.f=0 #define cost for endnode
    openlist.append(startnode) #add startnode to openlist
    
    while(len(openlist)>0): #while openilist not empty
        #find the node with the lowest fcost and set it to currentnode
        lowf = 10000
        highg = 0
        for node in openlist:
            openlistpos = getPos(openlist)
            if (node.f < lowf):
                lowf = node.f
                highg = node.g
                currentnode = node
            elif(node.f == lowf and node.g < highg):
                lowf = node.f
                highg = node.g
                currentnode = node
        #remove currentnode from openlist and add it to closedlist        
        openlist.pop(openlist.index(currentnode))
        closedlist.append(currentnode)
        
        if(currentnode.pos == endnode.pos): # if end node is found
            path = [currentnode]
            currenttrace = currentnode
            #generate path by backtracking parentnodes (going from parent to parent)
            while(currenttrace.pos != startnode.pos):
                currenttrace=currenttrace.parent
                path.append(currenttrace)

            path.reverse() #backtracked path is in reverse order
            optimizedpath = optimizePath(path)#get the optimized path
            return optimizedpath #return the optimized path and end the loop
            break
     
        neighbours=[] #generate empty list to store neighbours
        for cell in neighbourindex:
            newx = currentnode.pos[0] + cell[0]
            newy = currentnode.pos[1] + cell[1]
            #generate neighbours of currentnode and add them to neighbours
            if(gridlist[newx][newy] == 0 and newx>=0 and newy>=0 and newx<=resx and newy<=resy):
                if(gridlist[currentnode.pos[0]][currentnode.pos[1] + cell[1]]==0 or gridlist[currentnode.pos[0] + cell[0]][currentnode.pos[1]]==0):
                    neighbour = Searchnode(currentnode, [newx, newy])
                    if neighbour.pos not in getPos(openlist) and neighbour.pos not in getPos(closedlist):
                        neighbours.append(neighbour)

        for neighbour in neighbours: #calculate g, h, and f for each neighbour
            neighbour.g = (abs(neighbour.pos[0]-currentnode.pos[0]))+(abs(neighbour.pos[1]-currentnode.pos[1]))
            neighbour.h = (abs(neighbour.pos[0]-endnode.pos[0]))+(abs(neighbour.pos[1]-endnode.pos[1]))
            neighbour.f = neighbour.g + neighbour.h

            if(neighbour not in openlist): #add neighbour to openlist if not already there
                openlist.append(neighbour)
def optimizePath(path): #takes a list of nodes of type Searchnode
    i = 0
    optimizedpath = []#contains a list of coordinates corresponding to the optimized path
    while i<len(path): #starts looping through the path
        currentnode = path[i] #sets currentnode
        for testnode in path[path.index(currentnode):len(path)]: #loops through the path again, starting from testnode
            able = False #if able=false, the new path is blocked by a wall
            if testnode == currentnode and currentnode.pos not in optimizedpath:
                optimizedpath.append([currentnode.pos[0], currentnode.pos[1]])
                
            #x simple
            #checks if currentnode and testnode are not connected by a straight path on x
            #else it sees if it can connect currentnode and testnode, and adds the new path to optimizedpath
            elif testnode.pos[0] == currentnode.pos[0] and testPath(currentnode, testnode, path, "x") == False:
                if currentnode.pos[1] > testnode.pos[1]:
                    s = testnode.pos[1]
                    e = currentnode.pos[1]
                    ystep = range(e,s, -1)
                else:
                    e = testnode.pos[1]
                    s = currentnode.pos[1]
                    ystep = range(s, e)
                for y in ystep:
                    if gridlist[currentnode.pos[0]][y] == 0:
                        able = True
                    else:
                        able=False
                        break
                if able:
                    for y in ystep:
                        if [currentnode.pos[0],y] not in optimizedpath:
                            #adds new path to optimizedpath
                            optimizedpath.append([currentnode.pos[0],y])
                    i = path.index(testnode)-1 #continues loop from testnode, skipping the unoptimized route
                continue
            #x simple end
                    
            #y simple
            #checks if currentnode and testnode are not connected by a straight path on y
            #else it sees if it can connect currentnode and testnode, and adds the new path to optimizedpath
            elif testnode.pos[1] == currentnode.pos[1] and testPath(currentnode, testnode, path, "y") == False:
                if currentnode.pos[0] > testnode.pos[0]:
                    s = testnode.pos[0]
                    e = currentnode.pos[0]
                    xstep = range(e,s, -1)
                else:
                    e = testnode.pos[0]
                    s = currentnode.pos[0]
                    xstep = range(s,e)

                for x in xstep:
                    if gridlist[x][currentnode.pos[1]] == 0:
                        able = True
                    else:
                        able=False
                        break
                if able:
                    for x in xstep:
                        if [x,currentnode.pos[1]] not in optimizedpath:
                            #adds new path to optimizedpath
                            optimizedpath.append([x,currentnode.pos[1]])
                    i = path.index(testnode)-1#continues loop from testnode, skipping the unoptimized route
                continue
            #y simple end
        i = i+1 #if no optimization is possible, increment i to next node on the path
    #optimize end
    return optimizedpath #return the optimized path as a list of coordinates
#makes a straight line path between waypoints using helpnodes
#for 90 degree turns, no diagonals. takes a list of coordinates
def drawdiag2(path):
    global pixW
    global gridlist
    helpnodes = []
    i=0
    prev = path[0]
    for i in range(1, len(path)-1):
        #if the path has a corner, set a helpnode and connect previous waypoint/helpnode with a straight line
        if(path[i-1][0] != path[i+1][0] and path[i-1][1] != path[i+1][1]):
            drawCell(path[i][0], path[i][1], "red", "")
            l = Line(Point(prev[0]*pixW+int(pixW/2), prev[1]*pixW+int(pixW/2)), Point(path[i][0]*pixW+int(pixW/2), path[i][1]*pixW+int(pixW/2)))    
            l.draw(win)
            prev=path[i]
            helpnodes.append(path[i])
    l = Line(Point(prev[0]*pixW+int(pixW/2), prev[1]*pixW+int(pixW/2)), Point(path[len(path)-1][0]*pixW+int(pixW/2), path[len(path)-1][1]*pixW+int(pixW/2)))    
    l.draw(win)        
    return helpnodes
def printPath(): #prints information about paths for Arduino implementation
    global paths
    #determining MAX\_HELPNODES (largest number of help nodes)
    maxHelpnodes = 0   
    for path in paths:
        if(len(path.helpnodes) > maxHelpnodes):
            maxHelpnodes = len(path.helpnodes)

    print("/*MAX\_HELPNODES must be " + str(maxHelpnodes) + " - CHANGE IN Path.h!*/")        
    print("byte pathsnumber = " + str(len(paths)) + ";")
    for path in paths:
        helpnodesOut=path.helpnodes
        #adding [0,0] to paths with a smaller number of helpnodes, so arrays are equal in size
        if(len(path.helpnodes)<maxHelpnodes):
            missing = maxHelpnodes-len(path.helpnodes)
            for i in range(0,missing):
                helpnodesOut.append([0,0])
                
        helpnodesOut = str(helpnodesOut).replace("[", "{")
        helpnodesOut = helpnodesOut.replace("]", "}")
        print("Coord " + path.namew1 + path.namew2 + "[] = " + str(helpnodesOut) + ";")

    objectOut = "Path paths[] = {"
    for path in paths:
        objectOut = objectOut + 'Path({"' + path.namew1 + '", ' + str(path.w1[0]) + ', ' + str(path.w1[1]) + '}, {"' + path.namew2 + '", ' + str(path.w2[0]) + ', ' + str(path.w2[1]) + '}, ' + path.namew1 + path.namew2 + '), '
    objectOut = objectOut[:len(objectOut)-2]
    objectOut = objectOut + '};'
    print(objectOut)
    #notifying user about MAX\_HELPNODES
    messagebox.showwarning("Attention", "Update MAX\_HELPNODES in Path.h to " + str(maxHelpnodes))