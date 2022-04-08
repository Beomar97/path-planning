from sympy import false, true
import matplotlib.pyplot as plt
from interfaces.msg import Coordinate
from path_planning.model.tag import Tag
from .ConeEdge import ConeEdge
from statsmodels.nonparametric.kernel_regression import KernelReg

class Middle_Point_Method():

    def __init__(self):

        self.blue_cones = []
        self.yellow_cones = []
        self.orange_cones = []

        self.coneEdges = []
        self.middleEdges = []
        self.middlePointsX = []
        self.middlePointsY = []


    def middle_point_method(self, coordinate, blue_cones, yellow_cones):
        self.blue_cones = blue_cones
        self.yellow_cones = yellow_cones

        blueConesLength = len(self.blue_cones)
        yellowConesLength = len(self.yellow_cones)

        if (blueConesLength + yellowConesLength) > 1:
            foundValidEdge = false

            if coordinate.tag == Tag.YELLOW.value:
                index = blueConesLength-1
            elif coordinate.tag == Tag.BLUE.value:
                index = yellowConesLength-1
            else:
                index = 0
            
            if index <= 1:
                initIndex = 2
            else:
                initIndex = index
            
            while foundValidEdge == false and index >= (initIndex - 2):
                
                if coordinate.tag == Tag.YELLOW.value:
                    blueCone = Coordinate()
                    blueCone.x  = self.blue_cones[index][0]
                    blueCone.y = self.blue_cones[index][1]

                    #blueCone = Coordinate(Tag.BLUE.value,self.blue_cones[index][0],self.blue_cones[index][1])
                    newEdge = ConeEdge(coordinate,blueCone)

                    foundValidEdge = true
                elif coordinate.tag == Tag.BLUE.value:
                    yellowCone = Coordinate()
                    yellowCone.x = self.yellow_cones[index][0]
                    yellowCone.y = self.yellow_cones[index][1]

                    #yellowCone = Coordinate(Tag.YELLOW.value,self.yellow_cones[index][0],self.yellow_cones[index][1])
                    newEdge = ConeEdge(coordinate,yellowCone)

                    foundValidEdge = true
                
                index = index-1
                
            if foundValidEdge == true:
                self.coneEdges.append(newEdge)

                middlePoint = newEdge.getMiddlePoint()
                self.middlePointsX.append(middlePoint.x)
                self.middlePointsY.append(middlePoint.y)
                #print("x: " + middlePoint.x + " y: " + middlePoint.y)
                plt.plot(middlePoint.x,middlePoint.y,'o', c='red')
                if len(self.middlePointsX) % 7 == 0 and len(self.middlePointsY) % 7 == 0:
                    #spl = UnivariateSpline(self.middlePointsX, self.middlePointsY)
                    #xs = np.linspace(-20, 20, 1000)
                    kr = KernelReg(self.middlePointsY,self.middlePointsX,'c')
                    y_pred, y_std = kr.fit(self.middlePointsX)
                    plt.plot(self.middlePointsX, y_pred)
                    self.middlePointsX = []
                    self.middlePointsY = []
                    self.middlePointsX.append(middlePoint.x)
                    self.middlePointsY.append(middlePoint.y)