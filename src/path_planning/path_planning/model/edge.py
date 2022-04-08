
import math
from interfaces.msg import Coordinate

class Edge():
    
    def __init__(self,coordinate1: Coordinate, coordinate2: Coordinate):
        self.coordinate1 = coordinate1
        self.coordinate2 = coordinate2

        self.intersection = None

    def getMiddlePoint(self):
        middlePoint = Coordinate()
        middlePoint.tag = 'red'
        middlePoint.x = (self.coordinate1.x + self.coordinate2.x) / 2
        middlePoint.y = (self.coordinate1.y + self.coordinate2.y) / 2
        return middlePoint

    def length(self):
        return math.sqrt((self.coordinate1.x - self.coordinate2.x) ** 2 + (self.coordinate1.y - self.coordinate2.y) ** 2)

    def getPartsLengthRatio(self):
        part1Length = math.sqrt((self.coordinate1.x - self.intersection[0]) ** 2 + (self.coordinate1.y - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.coordinate2.x) ** 2 + (self.intersection[1] - self.coordinate2.y) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)
    
    def __eq__(self, other):
        return (self.coordinate1.x == other.coordinate1.x and self.coordinate1.y == other.coordinate1.y and self.coordinate2.x == other.coordinate2.x and self.coordinate2.y == other.coordinate2.y
             or self.coordinate1.x == other.coordinate2.x and self.coordinate1.y == other.coordinate2.y and self.coordinate2.x == other.coordinate1.x and self.coordinate2.y == other.coordinate1.y)

    def __str__(self):
        return "(" + str(round(self.coordinate1.x, 2)) + "," + str(round(self.coordinate1.y,2)) + "," + self.coordinate1.tag +"),(" + str(round(self.coordinate2.x, 2)) + "," + str(round(self.coordinate2.y,2)) + "," + self.coordinate2.tag +")"

    def __repr__(self):
        return str(self)
