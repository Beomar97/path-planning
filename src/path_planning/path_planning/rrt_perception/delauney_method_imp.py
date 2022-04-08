
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import numpy as np

from .edge import Edge

class Delauney_Method_Imp:
    def __init__(self):
        self.blue_cones = []
        self.yellow_cones = []
        self.treshhold = 7

        self.middle_points = []

    def delauney_method_improved(self,coordinate,blue_cones,yellow_cones):
        self.blue_cones = blue_cones
        self.yellow_cones = yellow_cones

        blueConesLength = len(self.blue_cones)
        yellowConesLength = len(self.yellow_cones)

        front_cones = []

        if blueConesLength == 3 and yellowConesLength == 2:
            yellowCone = Cone(self.yellow_cones[0])
            print(yellowCone)
            front_cones.append(yellowCone)

            for coneCoordinate in self.blue_cones:
                blueCone = Cone(coneCoordinate)
                distance = self.cone_distance(yellowCone,blueCone)
                if distance < 7:
                    front_cones.append(blueCone)

                print("distance: " + str(distance))

            if len(front_cones) == 3:
                yellowCone2 = Cone(yellow_cones[1])
                front_cones.append(yellowCone2)
            else:
                blueCone2 = Cone(blue_cones[2])
                front_cones.append(blueCone2)
            
            delauneyEdges = self.getDelaunayEdges(front_cones)


    def cone_distance(self,cone1,cone2):
        edge = Edge(cone1.x,cone1.y,cone2.x,cone2.y)

        return edge.length()


    
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

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"