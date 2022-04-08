
from scipy.spatial import Delaunay
import matplotlib.pyplot as plt
import numpy as np


#from .edge import Edge
from path_planning.model.tag import Tag
from path_planning.model.edge import Edge
from interfaces.msg import Coordinate


class Delauney_Method_Imp:
    def __init__(self):
        self.blue_cones = []
        self.yellow_cones = []
        self.treshholdCar = 7

        self.middle_points = []

        self.carCoordinate = ""

    def delauney_method_improved(self,coneCoordinate: Coordinate, carCoordinate: Coordinate):
        self.carCoordinate = carCoordinate

        if coneCoordinate.tag == Tag.BLUE.value:
            self.blue_cones.append(coneCoordinate)
            plt.plot(coneCoordinate.x, coneCoordinate.y, 'o', c='blue')
        elif coneCoordinate.tag == Tag.YELLOW.value:
            self.yellow_cones.append(coneCoordinate)
            plt.plot(coneCoordinate.x, coneCoordinate.y, 'o', c='yellow')

        if self.carCoordinate != None:
            #print(carCoordinate)
            plt.plot(self.carCoordinate.x,self.carCoordinate.y,'o',c='black')
            carDistanceToCone = self.coordinate_distance(coneCoordinate,self.carCoordinate)
            print("Car distance to cone: " + str(carDistanceToCone))
            if self.treshholdCar > carDistanceToCone:
                print("Under treshhold")

        blueConesLength = len(self.blue_cones)
        yellowConesLength = len(self.yellow_cones)

        front_cones = []
        middle_points = []

        if blueConesLength == 3 and yellowConesLength == 2:
            yellowCone = self.yellow_cones[0]
            print(yellowCone)
            front_cones.append(yellowCone)

            for blueCone in self.blue_cones:
                distance = self.coordinate_distance(yellowCone,blueCone)

                if distance < 7:
                    front_cones.append(blueCone)

                print("distance: " + str(distance))

            if len(front_cones) == 3:
                front_cones.append(self.yellow_cones[1])
            else:
                front_cones.append(self.blue_cones[2])
            
            delauneyEdges = self.getDelaunayEdges(front_cones)

            #print(delauneyEdges)
            middle_points = []
            for edge in delauneyEdges:
                middle_point = edge.getMiddlePoint()
                middle_points.append(middle_point)
                plt.plot(middle_point.x, middle_point.y, 'o', c='red')

        if len(middle_points) > 0 and carCoordinate != None:
            middle_points.sort(key=lambda middle_point: Edge(middle_point,carCoordinate).length())
            
        return middle_points

    #def quick_sort_middle_points(self,middle_points,carCoordinate):
        


    def coordinate_distance(self,coordinate1,coordinate2):
        edge = Edge(coordinate1,coordinate2)

        return edge.length()

    def getDelaunayEdges(self, frontCones):
        if len(frontCones) < 4: # no sense to calculate delaunay
            return
        
        conePointsDel = np.zeros((len(frontCones), 2))
        conePoints = np.zeros((len(frontCones), 3))

        for i in range(len(frontCones)):
            cone = frontCones[i]
            conePointsDel[i] = ([cone.x, cone.y])
            if frontCones[i].tag == Tag.YELLOW.value:
                conePoints[i] = ([cone.x, cone.y, 0])
            elif frontCones[i].tag == Tag.BLUE.value:
                conePoints[i] = ([cone.x, cone.y, 1])

        tri = Delaunay(conePointsDel)

        plt.triplot(conePointsDel[:,0], conePointsDel[:,1], tri.simplices)

        delaunayEdges = []
        for simp in tri.simplices:

            for i in range(3):
                j = i + 1
                if j == 3:
                    j = 0
                
                cone1 = Coordinate()
                cone1.x = conePoints[simp[i]][0]
                cone1.y = conePoints[simp[i]][1]
                if conePoints[simp[i]][2] == 0:
                    cone1.tag = Tag.YELLOW.value
                else:
                    cone1.tag = Tag.BLUE.value

                cone2 = Coordinate()
                cone2.x = conePoints[simp[j]][0]
                cone2.y = conePoints[simp[j]][1]
                if conePoints[simp[j]][2] == 0:
                    cone2.tag = Tag.YELLOW.value
                elif conePoints[simp[j]][2] == 1:
                    cone2.tag = Tag.BLUE.value

                edge = Edge(cone1,cone2)
                #edge = Edge(conePoints[simp[i]][0], conePoints[simp[i]][1], conePoints[simp[j]][0], conePoints[simp[j]][1])

                if edge not in delaunayEdges and cone1.tag != cone2.tag:
                    delaunayEdges.append(edge)

        return delaunayEdges

"""
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


"""

class Cone():
    def __init__(self, coordinate):
        self.x = coordinate[0]
        self.y = coordinate[1]

    def __str__(self):
        return "(" + str(self.x) + "," + str(self.y) + ")"