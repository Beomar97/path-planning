import math

from interfaces.msg import Coordinate


class ConeEdge():
    def __init__(self, coordinate1, coordinate2):
        self.x1 = coordinate1.x
        self.y1 = coordinate1.y
        self.x2 = coordinate2.x
        self.y2 = coordinate2.y
        self.intersection = None

    def getMiddlePoint(self):
        middlePoint = Coordinate()
        middlePoint.tag = 'red'
        middlePoint.x = (self.x1 + self.x2) / 2
        middlePoint.y = (self.y1 + self.y2) / 2
        return middlePoint
        #return (self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2

    def length(self):
        return math.sqrt((self.x1 - self.x2) ** 2 + (self.y1 - self.y2) ** 2)

    def getPartsLengthRatio(self):
        import math

        part1Length = math.sqrt((self.x1 - self.intersection[0]) ** 2 + (self.y1 - self.intersection[1]) ** 2)
        part2Length = math.sqrt((self.intersection[0] - self.x2) ** 2 + (self.intersection[1] - self.y2) ** 2)

        return max(part1Length, part2Length) / min(part1Length, part2Length)

    def __eq__(self, other):
        return (self.x1 == other.x1 and self.y1 == other.y1 and self.x2 == other.x2 and self.y2 == other.y2
             or self.x1 == other.x2 and self.y1 == other.y2 and self.x2 == other.x1 and self.y2 == other.y1)

    def __str__(self):
        return "(" + str(round(self.x1, 2)) + "," + str(round(self.y1,2)) + "),(" + str(round(self.x2, 2)) + "," + str(round(self.y2,2)) + ")"

    def __repr__(self):
        return str(self)