import math
from collections import defaultdict

import matplotlib.pyplot as plt
import numpy as np
from scipy.spatial import Delaunay


def main():

    blue_cones = np.array([
        [1152.27115094, 1281.52899302],
        [1155.53345506, 1295.30515742],
        [1163.56506781, 1318.41642169],
        [1168.03497425, 1330.03181319],
        [1173.26135672, 1341.30559949],
        [1184.07110925, 1356.54121651],
        [1194.88086178, 1371.77683353],
        [1202.58908737, 1381.41765447],
        [1210.72465255, 1390.65097106],
        [1227.81309742, 1403.29046460],
        [1244.90154229, 1415.92995815],
        [1261.98998716, 1428.56945169],
        [1275.89219696, 1438.21626352],
        [1289.79440676, 1447.86307535],
        [1303.69661656, 1457.50988719],
        [1323.80994319, 1470.41028655],
        [1343.92326983, 1488.31068591],
        [1354.31738934, 1499.33260989],
        [1374.48879779, 1516.93734053],
        [1394.66020624, 1534.54207116]
    ])

    yellow_cones = np.array([
        [1233.87375018, 1230.07095987],
        [1237.63559365, 1253.90749041],
        [1240.87500801, 1264.43925132],
        [1245.30875975, 1274.63795396],
        [1256.14493570, 1294.48254424],
        [1264.33600095, 1304.47893299],
        [1273.38192911, 1313.71468591],
        [1283.12411536, 1322.35942538],
        [1293.25593880, 1330.55873344],
        [1309.48170020, 1342.53074698],
        [1325.70746160, 1354.50276051],
        [1341.93322301, 1366.47477405],
        [1358.15898441, 1378.44678759],
        [1394.38474581, 1390.41880113]
    ])

    # Delaunay Triangulation
    triangulation = Delaunay(cones)

    x, y = zip(*cones)
    plt.triplot(x, y, triangulation.simplices)
    plt.plot(x, y, 'o')
    plt.show()

    # Midpoints of edges become simplices (vertices)
    for simplicy in triangulation.simplices:
        for i in range(3):
            j = i + 1
            if j == 3:
                j = 0
            midpoint = calculate_midpoint(cones[simplicy[i]], cones[simplicy[j]])
            if midpoint not in cones:
                cones = np.append(cones, midpoint)

    triangulation = Delaunay(cones)

    edges = []
    for simplicy in triangulation.simplices:
        for i in range(3):
            j = i + 1
            if j == 3:
                j = 0
            edge = (coordinates[simplicy[i]], coordinates[simplicy[j]])
            edgeMirrored = (coordinates[simplicy[j]], coordinates[simplicy[i]])
            if edge and edgeMirrored not in edges:
                edges.append(edge)

    graph = defaultdict(list)

    for edge in edges:
        a, b = edge[0], edge[1]
        graph[a].append(b)
        graph[b].append(a)

    # Graph Search
    # TODO

    # Fit straight line & extrapolate
    # TODO

    x, y = zip(*coordinates)

    plt.triplot(x, y, triangulation.simplices)
    plt.plot(x, y, 'o')
    plt.show()


def calculate_midpoint(a, b):
    return ((a[0] + b[0])/2, (a[1] + b[1])/2)

def calculate_distance(a, b):
    return math.sqrt((b[0] - a[0])**2 + (b[1] - a[1])**2)

if __name__ == '__main__':
    main()
