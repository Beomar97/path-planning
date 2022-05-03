import matplotlib.pyplot as plt
from path_planning._testing.test_data import TestData
from path_planning.algorithm.optimization import main_globaltraj
from path_planning.util.optimization_input_transformer import \
    OptimizationInputTransformer


def main():

    blue_cones = TestData.SmallTrack.get_blue_cones()
    yellow_cones = TestData.SmallTrack.get_yellow_cones()
    refline = TestData.SmallTrack.get_refline()

    reftrack = OptimizationInputTransformer.transform(
        blue_cones, yellow_cones, [], [], refline)

    # Plot cones and refline
    blue_cones_x, blue_cones_y = zip(*blue_cones)
    yellow_cones_x, yellow_cones_y = zip(*yellow_cones)
    refline_x, refline_y = zip(*refline)

    plt.plot(blue_cones_x, blue_cones_y, 'o', c='blue')
    plt.plot(yellow_cones_x, yellow_cones_y, 'o', c='yellow')
    plt.plot(refline_x, refline_y, 'o', c='black')

    plt.show()

    main_globaltraj.optimize_path(reftrack)


if __name__ == '__main__':
    main()
