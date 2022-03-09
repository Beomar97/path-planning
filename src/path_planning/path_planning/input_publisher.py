import rclpy
from rclpy.node import Node
from interfaces.msg import Input


class InputPublisher(Node):

    blue_cones_x = [
        1152.27115094,
        1155.53345506,
        1163.56506781,
        1168.03497425,
        1173.26135672,
        1184.07110925,
        1194.88086178,
        1202.58908737,
        1210.72465255,
        1227.81309742,
        1244.90154229,
        1261.98998716,
        1275.89219696,
        1289.79440676,
        1303.69661656,
        1323.80994319,
        1343.92326983,
        1354.31738934,
        1374.48879779,
        1394.66020624
    ]

    blue_cones_y = [
        1281.52899302,
        1295.30515742,
        1318.41642169,
        1330.03181319,
        1341.30559949,
        1356.54121651,
        1371.77683353,
        1381.41765447,
        1390.65097106,
        1403.29046460,
        1415.92995815,
        1428.56945169,
        1438.21626352,
        1447.86307535,
        1457.50988719,
        1470.41028655,
        1488.31068591,
        1499.33260989,
        1516.93734053,
        1534.54207116
    ]

    yellow_cones_x = [
        1233.87375018,
        1237.63559365,
        1240.87500801,
        1245.30875975,
        1256.14493570,
        1264.33600095,
        1273.38192911,
        1283.12411536,
        1293.25593880,
        1309.48170020,
        1325.70746160,
        1341.93322301,
        1358.15898441,
        1394.38474581
    ]

    yellow_cones_y = [
        1230.07095987,
        1253.90749041,
        1264.43925132,
        1274.63795396,
        1294.48254424,
        1304.47893299,
        1313.71468591,
        1322.35942538,
        1330.55873344,
        1342.53074698,
        1354.50276051,
        1366.47477405,
        1378.44678759,
        1390.41880113
    ]

    def __init__(self):
        super().__init__('input_publisher')
        self.publisher_ = self.create_publisher(Input, 'input', 10)
        timer_period = 1.0  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        input = Input()
        input.blue_cones_x = self.blue_cones_x
        input.blue_cones_y = self.blue_cones_y
        input.yellow_cones_x = self.yellow_cones_x
        input.yellow_cones_y = self.yellow_cones_y
        self.publisher_.publish(input)
        self.get_logger().info('Publishing')
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    input_publisher = InputPublisher()

    rclpy.spin(input_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    input_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()