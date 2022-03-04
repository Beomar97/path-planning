import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial import Delaunay

def main():
    print('Hi from my_package.')

    blue_cones_coordinates = [
        (0,0),
        (0,1)
        ]

    yellow_cones_coordinates = [
        (1,0),
        (1,1)
        ]

    # Input Map
    coordinates = blue_cones_coordinates + yellow_cones_coordinates

    # Delaunay Triangulation
    tri = Delaunay(coordinates)
    print(tri.simplices)
    print(tri.simplices[0])
    test = calculate_midpoints_of_edges(coordinates, tri.simplices[0])
    print(test)
    
    x, y = zip(*coordinates)

    plt.triplot(x, y, tri.simplices)
    plt.plot(x, y, 'o')
    plt.show()

def calculate_midpoints_of_edges(coordinates, simplicy):
    point1 = coordinates[simplicy[0]]
    point2 = coordinates[simplicy[1]]
    point3 = coordinates[simplicy[2]]

    edge1 = ((point1[0] + point2[0])/2, (point1[1] + point2[1])/2)
    edge2 = ((point1[0] + point3[0])/2, (point1[1] + point3[1])/2)
    edge3 = ((point2[0] + point3[0])/2, (point2[1] + point3[1])/2)

    return [edge1, edge2, edge3]



def midpoint(x1, y1, x2, y2):
    return ((x1 + x2)/2, (y1 + y2)/2)

if __name__ == '__main__':
    main()
