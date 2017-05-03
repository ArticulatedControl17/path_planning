from pathPlanning import *
from vehicleState import *
from Point import *
from pp_wrapper import *
import time

testPy = False
#testPy = True

#FOR TEST: change DT and grid sizes, information to save: amount of nodes, error after layer 1, lowest error, amount of nodes after layer 1, AVG error, runtime

p = os.path.abspath(os.path.dirname(__file__))
lib_path = os.path.abspath(os.path.join(p, '..', '..', 'truck_map', 'scripts'))
sys.path.append(lib_path)
from map_func import *


map_obj = Map()
(maps, scale) = map_obj.getMapAndScale()


#start vehicleState
start_vs = VehicleState(323.2, 748.4, 1.57999992371, 1.57999992371)
#hardocded ref path, hard turn
#refpath =  [(323.4, 765.0), (323.0, 785.0), (321.1, 799.8), (318.3, 809.7), (314.5, 819.3), (309.9, 828.5), (304.4, 837.2), (298.1, 845.4), (291.1, 852.9), (283.4, 859.8), (275.1, 865.8), (266.3, 871.1), (257.0, 875.0), (245.9, 878.4), (234.7, 880.6), (223.2, 881.8), (211.7, 881.8), (200.3, 880.8), (189.0, 878.6), (178.0, 875.0), (166.4, 870.5), (155.6, 864.4), (145.7, 856.9), (137.0, 848.0), (129.6, 838.1), (123.7, 827.1), (119.4, 815.5), (116.8, 803.3), (116.0, 797.0), (116.0, 776.3), (116.0, 755.7), (116.1, 735.1), (116.1, 714.5), (116.1, 693.9), (116.2, 673.3), (116.0, 669.0), (137.0, 669.2), (158.0, 669.4), (179.0, 669.6), (200.0, 669.8), (221.0, 670.0), (242.0, 670.2), (263.0, 670.4), (284.0, 670.6), (305.0, 670.8)]

#ref path
start_vs = VehicleState(168.2, 671.1, -0.000796440232079, -0.000796440232079)
refpath = [(179.0, 669.6), (200.0, 669.8), (221.0, 670.0), (242.0, 670.2), (263.0, 670.4), (284.0, 670.6), (305.0, 670.8), (326.0, 671.0), (325.3, 685.0), (324.8, 705.0), (324.4, 725.0), (323.9, 745.0), (323.4, 765.0), (323.0, 785.0), (321.1, 799.8), (318.3, 809.7), (314.5, 819.3), (309.9, 828.5), (304.4, 837.2), (298.1, 845.4), (291.1, 852.9), (283.4, 859.8), (275.1, 865.8), (266.3, 871.1), (257.0, 875.0), (245.9, 878.4), (234.7, 880.6), (223.2, 881.8), (211.7, 881.8)]


g = refpath[-1]
g2 = refpath[-2]

if testPy:
    pathplanner = PathPlanner(maps)
    pathplanner.setOptimalpath(refpath)

else:
    pathplanner = PathPlannerCPP(maps)
    pathplanner.setOptimalPath(refpath)

start_time = time.time()
path = pathplanner.getPath(start_vs, g, g2, 6, 6, 0.3)

run_time = ("--- %s seconds ---" % (time.time() - start_time))
print "runtime", run_time

print "size: ", len(path)

lx= []
ly=[]
lt1= []
lt2=[]


for vs in path:
    print "x: ", vs.x, "y: ", vs.y, "th1: ", vs.theta1, "th2: ", vs.theta2
    lx.append(vs.x)
    ly.append(vs.y)
    lt1.append(vs.theta1)
    lt2.append(vs.theta2)

plt.imshow(maps)

#plt.plot([startPoint.x, endPoint[0]], [startPoint.y, endPoint[1]], 'ro')

plt.plot(lx, ly, 'green')

plt.axis([-10, 1000, 1000, -10])
plt.show()
