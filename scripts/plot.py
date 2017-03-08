import matplotlib.pyplot as plt
from pathPlanning import *
from model import *

pf = graphFinder()

startPoint = Point(60,800)
endPoint = Point(480,100)

path = pf.creategraph(startPoint, endPoint, 40)
model = truck()

lx= []
ly=[]
lt1= []
lt2=[]


for ((xa,ya),ta1, ta2) in reversed(path[1]):
    print xa,ya

for ((x,y),t1,t2) in path[1]:
    lx.append(x)
    ly.append(y)
    lt1.append(t1)
    lt2.append(t2)

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

#wall,
wallx1 = [80,80,500,500]
wally1 = [1000,540,540,0]

wallx2 = [0,0,420,420]
wally2 = [1000,460,460,0]

plt.plot(headerFrontLeftx, headerFrontLefty, 'blue')
plt.plot(headerFrontRightx, headerFrontRighty, 'blue')

plt.plot(headerBacktLeftx, headerBacktLefty, 'red')
plt.plot(headerBackRightx, headerBackRighty, 'red')

plt.plot(trailerBackRightx, trailerBackRighty, 'green')
plt.plot(trailerBackLeftx, trailerBackLefty, 'green')

plt.plot(wallx1, wally1, 'yellow')
plt.plot(wallx2, wally2, 'yellow')

plt.plot([startPoint.x, endPoint.x], [startPoint.y, endPoint.y], 'ro')

plt.plot(lx, ly, 'go')



plt.axis([-10, 1000, 1000, -10])
plt.show()
