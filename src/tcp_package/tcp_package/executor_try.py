import rclpy
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class OdomSub(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        odom_data = {
            "msg_type": "ODOMETRY",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "w": msg.pose.pose.orientation.w,
            "end": "**********end***********\n"
        }
        print(odom_data)
        time.sleep(2)

class MapSub(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)

    def map_callback(self, msg):
        map_data = list(msg.data)

        map_json = {
            "msg_type": "MAP",
            "width": msg.info.width,
            "height": msg.info.height,
            "resolution": msg.info.resolution,
            "data": map_data,
            "end": "**********end***********\n"
        }
        print("map height: ",map_json["height"])
class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            1)

    def cmd_vel_callback(self, msg):
        vel_data = {
            "msg_type": "VELOCITY",
            "linear_x": msg.linear.x,
            "angular_z": msg.angular.z,
            "end": "**********end***********\n"
        }
        print(vel_data)


def main(args=None):
    rclpy.init(args=args)
    Odom=OdomSub()
    map=MapSub()
    cmd_vel=VelocitySubscriber()

    executor = MultiThreadedExecutor()
    executor.add_node(Odom)
    executor.add_node(map)
    executor.add_node(cmd_vel)

    executor.spin()

    executor.shutdown()

if __name__ == '__main__':
    main()