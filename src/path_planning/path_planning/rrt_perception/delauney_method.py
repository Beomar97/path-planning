import time
from path_planning.model.tag import Tag
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import numpy as np

from .edge import Edge
from .ConeEdge import ConeEdge

class Delauney_Method():

    def __init__(self):

        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []

        self.coneEdges = []
        self.middleEdges = []
        self.middlePointsX = []
        self.middlePointsY = []
        self.blue_counter = 0
        self.yellow_counter = 0

    
    def delaunay_method(self,coordinate,blue_cones,yellow_cones):
        self.blue_cones = blue_cones
        self.yellow_cones = yellow_cones

        frontConesBlue = []
        frontConesYellow = []
        blueConesLength = len(self.blue_cones)
        yellowConesLength = len(self.yellow_cones)

        if coordinate.tag == Tag.BLUE.value:
            self.blue_counter = self.blue_counter + 1
        elif coordinate.tag == Tag.YELLOW.value:
            self.yellow_counter = self.yellow_counter+ 1
        
        print("Yellow: " + str(self.yellow_counter))
        print("Blue: " + str(self.blue_counter))

        if (self.blue_counter + self.yellow_counter) >= 4:
            
            if self.blue_counter - self.yellow_counter >= 3:
                frontConesBlue.append(Cone(self.blue_cones[blueConesLength-1]))
                frontConesBlue.append(Cone(self.blue_cones[blueConesLength-2]))
                frontConesBlue.append(Cone(self.blue_cones[blueConesLength-3]))
                frontConesYellow.append(Cone(self.yellow_cones[yellowConesLength-1]))
                
                self.getDelauneyMiddlePoints(frontConesBlue + frontConesYellow)
            elif self.yellow_counter - self.blue_counter >= 3:
                

                frontConesYellow.append(Cone(self.yellow_cones[yellowConesLength-1]))
                frontConesYellow.append(Cone(self.yellow_cones[yellowConesLength-2]))
                frontConesYellow.append(Cone(self.yellow_cones[yellowConesLength-3]))
                frontConesBlue.append(Cone(self.blue_cones[blueConesLength-1]))

                self.getDelauneyMiddlePoints(frontConesBlue + frontConesYellow)
            elif self.blue_counter % 2 == 0 and self.yellow_counter % 2 == 0:
                frontConesBlue.append(Cone(self.blue_cones[blueConesLength-1]))
                frontConesBlue.append(Cone(self.blue_cones[blueConesLength-2]))
                frontConesYellow.append(Cone(self.yellow_cones[yellowConesLength-1]))
                frontConesYellow.append(Cone(self.yellow_cones[yellowConesLength-2]))
                self.getDelauneyMiddlePoints(frontConesBlue + frontConesYellow)

        if self.yellow_counter == 4 and self.blue_counter == 4:
            self.yellow_counter = 0
            self.blue_counter = 0
        #if self.blue_counter == 4:
        #    self.blue_counter = 0

    def getDelauneyMiddlePoints(self,cones):
            startTime = time.time()
            #delaunayEdgesBlue = self.getDelaunayEdges(frontConesBlue)
            #delaunayEdgesYellow = self.getDelaunayEdges(frontConesYellow)
            delaunayEdges = self.getDelaunayEdges(cones)

            #print(delaunayEdges)
            
            print("Edges:")
            for edge in delaunayEdges:
                print(edge.x1)
                #if edge not in delaunayEdgesBlue and edge not in delaunayEdgesYellow:
                self.middleEdges.append(edge)
                #middlePoint = edge.getMiddlePoint()
                #plt.plot(middlePoint.x,middlePoint.y,'o', c='red')
            
            for middleEdge in self.middleEdges:
                middlePoint = middleEdge.getMiddlePoint()
                plt.plot(middlePoint.x,middlePoint.y,'o', c='red')
            print("Delaunay Middle: {0} ms".format((time.time() - startTime) * 1000))
    
    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 4: # no sense to calculate delaunay
            return

        conePoints = np.zeros((len(frontCones), 2))

        for i in range(len(frontCones)):
            cone = frontCones[i]
            conePoints[i] = ([cone.x, cone.y])

        # print conePoints
        tri = Delaunay(conePoints)
        # print "len(tri.simplices):", len(tri.simplices)

        plt.triplot(conePoints[:,0], conePoints[:,1], tri.simplices)
        
        delaunayEdges = []
        for simp in tri.simplices:

            for i in range(3):
                j = i + 1
                if j == 3:
                    j = 0
                edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

                if edge not in delaunayEdges:
                    delaunayEdges.append(edge)
        
        return delaunayEdges

class Cone():
    def __init__(self, coordinate):
        self.x = coordinate[0]
        self.y = coordinate[1]