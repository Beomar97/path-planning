import csv
import os
import sys

import matplotlib.pyplot as plt
from path_planning.model.tag import Tag


def main():
    """
    Plot a track.

    Plot the given track displaying all blue, yellow, orange and big orange cones.
    """
    cones = []
    blue_cones = []
    yellow_cones = []
    orange_cones = []
    big_orange_cones = []
    car_start = []

    with open(os.getcwd() + '/src/path_planning/resource/maps/' + sys.argv[1]) as csv_file:
        csv_reader = csv.DictReader(csv_file, delimiter=',')
        for row in csv_reader:
            cones.append([float(row['x']), float(row['y'])])
            if row['tag'] == Tag.BLUE.value:
                blue_cones.append([float(row['x']), float(row['y'])])
            elif row['tag'] == Tag.YELLOW.value:
                yellow_cones.append([float(row['x']), float(row['y'])])
            elif row['tag'] == Tag.ORANGE.value:
                orange_cones.append([float(row['x']), float(row['y'])])
            elif row['tag'] == Tag.BIG_ORANGE.value:
                big_orange_cones.append([float(row['x']), float(row['y'])])
            elif row['tag'] == Tag.CAR_START.value:
                car_start = [float(row['x']), float(row['y'])]

    # Plot
    if blue_cones:
        blue_cones_x, blue_cones_y = zip(*blue_cones)
        plt.plot(blue_cones_x, blue_cones_y, 'o', c='blue')

    if yellow_cones:
        yellow_cones_x, yellow_cones_y = zip(*yellow_cones)
        plt.plot(yellow_cones_x, yellow_cones_y, 'o', c='yellow')

    if orange_cones:
        orange_cones_x, orange_cones_y = zip(*orange_cones)
        plt.plot(orange_cones_x, orange_cones_y, 'o', c='orange')

    if big_orange_cones:
        big_orange_cones_x, big_orange_cones_y = zip(*big_orange_cones)
        plt.plot(big_orange_cones_x, big_orange_cones_y, 'o', c='red')

    if car_start:
        plt.plot(car_start[0], car_start[1], 'o', c='black')

    plt.show()


if __name__ == '__main__':
    main()
