
import math
import sys
import decimal

class Point:
    def __init__(self, x , y):
        self.x = x
        self.y = y

class Vector:
    def __init__(self, x , y):
        self.values = (x,y)

    def magnitude(self):
        #magnitude of the vector
        (x,y)= self.values
        return math.sqrt(x*x + y*y)

    def rotate(self, theta):
        #rotate the vector theta degrees
        theta = math.radians(theta)
        dc, ds = math.cos(theta), math.sin(theta)
        (x, y) = self.values
        (x, y) = dc*x - ds*y, ds*x + dc*y
        return Vector(x, y)


class graphFinder:

    def vectorToPoint(self, vector , prevToPoint ):
        (x,y) = vector.values
        toPoint = Point(prevToPoint.x + x, prevToPoint.y + y)
        return toPoint

    def checkIfInTrack(self, (fromPoint, toPoint)):
        p0 = Point(100,100) #corner position
        if fromPoint.x <=0 or toPoint.x <=0 :
            #truck to the left of track
            return False
        if fromPoint.y <=0 or toPoint.y <=0 :
            #truck above track
            return False
        if fromPoint.y >= 250 or toPoint.y >= 250:
            #truck is belove the track
            return False
        if fromPoint.x >=250 or toPoint.x >=250 :
            #truck to the right of track
            return False
        #check if p0 is to the left of the line, that means that the line enter forbidden area
        isLeft = ((toPoint.x - fromPoint.x)*(p0.y - fromPoint.y) - (toPoint.y - fromPoint.y)*(p0.x - fromPoint.x)) <=0
        dist = math.sqrt( (100 - toPoint.x)**2 + (100 - toPoint.y)**2 )
        if isLeft and dist <10:
            return False
        if fromPoint.y<100 and toPoint.y>100:
            #going back down again
            return False
        if fromPoint.x>100 and fromPoint.y>100:
            return False
        if toPoint.x>100 and toPoint.y>100:
            return False
        return True

    def creategraph(self, startPoint, endPoint):
        self.distance = 10 #distance for each vector
        self.alpha = 10 #the amount of degrees to rotate in each direction
        #10, 6 inresting case

        #calculate the first vector
        firstV = Vector(75 - 75 , -self.distance) #first vector, the direction is the important part
        mag = firstV.magnitude()
        nFirstV = tuple( comp/mag for comp in firstV.values )
        (newx,newy) = tuple(comp*self.distance for comp in nFirstV) #the vector from startPoint towards firstV
        toPoint = pf.vectorToPoint(Vector(newx,newy), startPoint) # new strait point

        #initiate fields
        self.graph = {(toPoint.x, toPoint.y) : []} #the graph of all the points
        self.visited = set([]) #the points we have visited
        self.fromPoints ={(toPoint.x, toPoint.y) : (startPoint.x , startPoint.y)} #a Map which hold the frompoints given a toPoint
        self.toVisit = [(toPoint.x,toPoint.y)] #a stack of points to visit,
        self.path= [] #the path we are taking TODO: might not be the correct path.
        prevFromPoint = 0 #the previous point
        prevToPoint = startPoint
        count =0 #for debugging


        while len(self.toVisit)>0:
            #loop until all vectors have been checked for adjacent vectors
            count= count+1
            while True:
                (x,y) = self.toVisit.pop()
                if (x,y) not in self.visited:
                    break
                else:
                    #pass
                    self.path.pop() #TODO: Crashes sometimes
            self.path.append((x,y))
            (a,b) = self.fromPoints[x, y]
            prevFromPoint = Point(a,b)
            prevToPoint = Point(x, y) # get the toPoint
            dist = math.sqrt( (200 - x)**2 + (75 - y)**2 )
            if dist <20:
                #reached end
                print prevToPoint.x, prevToPoint.y
                print count
                print self.path
                return self.visited

            #looking for adjacent vectors

            #Find the strait new vector from the previous direction
            prevV = Vector(prevToPoint.x - prevFromPoint.x , prevToPoint.y - prevFromPoint.y)
            mag = prevV.magnitude()
            nPrevV = tuple( comp/mag for comp in prevV.values )
            (x,y) = tuple(comp*self.distance for comp in nPrevV) #the vector from prevToPoint to newPoint

            #Get the rotated vectors
            vectorStrait = (prevToPoint, self.vectorToPoint(Vector(x,y), prevToPoint))
            vectorRight = (prevToPoint, self.vectorToPoint(Vector(x,y).rotate(self.alpha), prevToPoint))
            vectorLeft = (prevToPoint, self.vectorToPoint(Vector(x,y).rotate(-self.alpha), prevToPoint))

            #Values of the vectors
            vectorStraitVal = (round(vectorStrait[1].x, 1), round(vectorStrait[1].y, 1))
            vectorRightVal = (round(vectorRight[1].x, 1), round(vectorRight[1].y, 1))
            vectorLeftVal = (round(vectorLeft[1].x, 1), round(vectorLeft[1].y, 1))

            #see if the endpoint is to the right/left/strait and go correct direction from there
            goRightOrLeft = ((prevToPoint.x - prevFromPoint.x)*(endPoint.y - prevFromPoint.y) -
                (prevToPoint.y - prevFromPoint.y)*(endPoint.x - prevFromPoint.x))
            #TODO: Never goes strait


            if goRightOrLeft==0:
                #Go strait
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(vectorLeft):
                    self.addVector(vectorLeftVal, prevToPoint)
                if self.checkIfInTrack(vectorRight):
                    self.addVector(vectorRightVal, prevToPoint)
                if self.checkIfInTrack(vectorStrait):
                    self.addVector(vectorStraitVal, prevToPoint)

            elif goRightOrLeft>0:
                #Go right
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(vectorLeft):
                    self.addVector(vectorLeftVal, prevToPoint)
                if self.checkIfInTrack(vectorStrait):
                    self.addVector(vectorStraitVal, prevToPoint)
                if self.checkIfInTrack(vectorRight):
                    self.addVector(vectorRightVal, prevToPoint)

            else:
                #Go left
                #check if the vectors are within the allowed track
                if self.checkIfInTrack(vectorRight):
                    self.addVector(vectorRightVal, prevToPoint)
                if self.checkIfInTrack(vectorStrait):
                    self.addVector(vectorStraitVal, prevToPoint)
                if self.checkIfInTrack(vectorLeft):
                    self.addVector(vectorLeftVal, prevToPoint)

            #mark the previous vector as visited
            self.visited.add((prevToPoint.x, prevToPoint.y))

        print "reached End, no solution found"
        return self.path

    def addVector(self, vectorVal, prevToPoint):
        #add the vector as an adjacent vector to the previous vector in the graph
        newList = self.graph[(prevToPoint.x, prevToPoint.y)]
        newList.append(vectorVal)
        self.graph[(prevToPoint.x, prevToPoint.y)] = newList
        self.graph[vectorVal] = []
        self.fromPoints[vectorVal] = (prevToPoint.x, prevToPoint.y)
        self.toVisit.append(vectorVal)

if __name__ == '__main__':

    pf = graphFinder()
    pf.creategraph(Point(75,200), Point(200,75))

#    **************************Tests****************************

#    pf = graphFinder()
#    prevToPoint=Point(0,5)
#    prevFromPoint=Point(0,10)
#    distance = 5
#    prevV = Vector(prevToPoint.x - prevFromPoint.x , prevToPoint.y - prevFromPoint.y)
#    mag = prevV.magnitude()
#    print "mag :", mag
#    nPrevV = tuple( comp/mag for comp in prevV.values )
#    (newx,newy) = tuple(comp*distance for comp in nPrevV) #the vector from prevToPoint to newPoint
#    toPoint = pf.vectorToPoint(Vector(newx,newy), prevToPoint) # new strait point
#    print "x: " , toPoint.x, "y: " , toPoint.y
#    newV = Vector(newx,newy).rotate(90)
#    toPoint = pf.vectorToPoint(newV, prevToPoint) # new strait point 45 degrees to the right
#    print "x: " , toPoint.x, "y: " , toPoint.y


#    graphf = graphFinder()
#    print "true :"
#    print graphf.checkIfInTrack(Point(75,200),Point(82,192))
#    print graphf.checkIfInTrack(Point(75,100),Point(25,50))
#    print graphf.checkIfInTrack(Point(25,50),Point(50,25))
#    print graphf.checkIfInTrack(Point(50,25),Point(75,50))
#    print graphf.checkIfInTrack(Point(75,50),Point(200,75))
#
#    print "false:"
#    print graphf.checkIfInTrack(Point(100,50),Point(150,150))
#    print graphf.checkIfInTrack(Point(50,50),Point(200,125))
