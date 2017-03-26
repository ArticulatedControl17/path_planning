import matplotlib.pyplot as plt
from pathPlanning import *
from model import *
import time
import numpy as np
import cv2
from vehicleState import vehicleState

optimalFile = "optimal_path3.txt"
mapName = 'map3.png'
mapp = np.asarray(cv2.imread(mapName, 0), dtype=np.bool).tolist()
#optimalFile = "optimal_path_rondell3.txt"
#mapName = 'rondell_4.png'

dirpath = os.path.dirname(os.path.abspath(__file__))
f = open(dirpath+'/'+optimalFile, 'r')
lines = [line.rstrip('\n') for line in f.readlines()]
posL = [s.split(' ', 1 ) for s in lines]
optimalPath = []
for l in posL:
    optimalPath.append(Point(float(l[0]),float(l[1])))

pf = graphFinder(mapp)
pf.setOptimalpath(optimalPath)

#startPoint = Point(100, 530) # for rondell
#endPoint = Point(523, 60) #for rondell
startPoint = Point(400, 900) #for map3
endPoint = Point(90,130) #for map3

start_time = time.time()
vehicleState = vehicleState(startPoint.x, startPoint.y, radians(180), radians(180))
#path = pf.creategraph(startPoint, endPoint, 30, radians(0), radians(0))
path = pf.getPath(vehicleState, endPoint)
#37 does many laps

run_time = ("--- %s seconds ---" % (time.time() - start_time))
model = truck()

lx= []
ly=[]
lt1= []
lt2=[]


#for ((xa,ya),ta1, ta2) in reversed(path):
#    print "angle", ta1
#    print xa,ya

for vs in path:
    print "x, y: ", vs.x, vs.y
    lx.append(vs.x)
    ly.append(vs.y)
    lt1.append(vs.theta1)
    lt2.append(vs.theta2)

li= []
for i in range(len(lx)-1):
    li = li + [(model.calculateCorners(Point(lx[i],ly[i]), lt1[i],lt2[i]))]

headerFrontLeftx = []
headerFrontLefty = []

headerFrontRightx = []
headerFrontRighty = []

headerBacktLeftx = []
headerBacktLefty = []

headerBackRightx = []
headerBackRighty = []

trailerBackLeftx = []
trailerBackLefty = []

trailerBackRightx = []
trailerBackRighty = []
count =0

for ((x1,y1),(x2,y2),(x3,y3),(x4,y4),(x5,y5),(x6,y6)) in reversed(li):
    #if count >4:
    #    break
    #count= count+1
    headerFrontLeftx.append(x1)
    headerFrontLefty.append(y1)
    headerFrontRightx.append(x2)
    headerFrontRighty.append(y2)
    headerBacktLeftx.append(x3)
    headerBacktLefty.append(y3)
    headerBackRightx.append(x4)
    headerBackRighty.append(y4)
    trailerBackLeftx.append(x5)
    trailerBackLefty.append(y5)
    trailerBackRightx.append(x6)
    trailerBackRighty.append(y6)
matrix = np.asarray(cv2.imread(mapName, 0), dtype=np.bool).tolist()
plt.imshow(matrix)
#wall,
#wallx1 = [500,80,80,80,500,500]
#wally1 = [842,842,842,540,540,0]

#wallx2 = [0,0,420,420,0]
#wally2 = [1000,460,460,166,166]

plt.plot(headerFrontLeftx, headerFrontLefty, 'blue')
plt.plot(headerFrontRightx, headerFrontRighty, 'blue')

plt.plot(headerBacktLeftx, headerBacktLefty, 'red')
plt.plot(headerBackRightx, headerBackRighty, 'red')

plt.plot(trailerBackRightx, trailerBackRighty, 'green')
plt.plot(trailerBackLeftx, trailerBackLefty, 'green')

#plt.plot(wallx1, wally1, 'yellow')
#plt.plot(wallx2, wally2, 'yellow')

plt.plot([startPoint.x, endPoint.x], [startPoint.y, endPoint.y], 'ro')

plt.plot(lx, ly, 'go')

optimal_x =  []
optimal_y =  []
for l in optimalPath:
    optimal_x.append(l.x)
    optimal_y.append(l.y)

plt.plot(optimal_x, optimal_y, 'yellow')

print run_time

plt.axis([-10, 1000, 1000, -10])
plt.show()
