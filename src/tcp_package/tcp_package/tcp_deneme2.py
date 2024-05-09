import rclpy
import json
import time
import socket
import threading
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class Netcat():
    def __init__(self):
        self.target_host = "192.168.41.174"
        self.target_port = 8910
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected = False

    def connect(self):
        try:
            self.client.connect((self.target_host, self.target_port))
            self.connected = True
        except ConnectionRefusedError:
            print("Bağlantı hatası: Sunucuya bağlanılamadı.")

    def data_transfer(self,msg):
        if not self.connected:
            self.connect()

        if self.connected:
            try:
                self.client.send(msg)
                response = self.client.recv(4096)
                print("\nRESPONSE:" + response.decode() + "\n")
            except ConnectionError:
                print("Bağlantı hatası: Veri gönderilirken bir hata oluştu.")

    def close(self):
        if self.connected:
            self.client.close()
            self.connected = False


class Odom_sub(Node):
    def __init__(self):
        super().__init__('pose_subscriber')
        self.netcat = Netcat
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        odom_data = {
            "\nmsg_type": "ODOMETRY",
            "\nx": msg.pose.pose.position.x,
            "\ny": msg.pose.pose.position.y,
            "\nw_": msg.pose.pose.orientation.w,
            "\nend": "**********end***********\n"
        }
        odom_msg_str = json.dumps(odom_data)
        self.netcat.data_transfer(odom_msg_str.encode()) 


class VelocitySubscriber(Node):
    def __init__(self):
        super().__init__('velocity_subscriber')
        self.netcat = Netcat
        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

    def cmd_vel_callback(self, msg):
        vel_data = {
            "msg_type": "VELOCITY",
            "linear_x": msg.linear.x,
            "angular_z": msg.angular.z,
            "end": "**********end***********\n"
        }
        vel_msg_str = json.dumps(vel_data)
        self.netcat.data_transfer(vel_msg_str.encode()) 


class Map_Sub(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.netcat = Netcat

    def map_callback(self, msg):
        map_data = list(msg.data)

        map_json = {
            "\nmsg_type": "MAP",
            "\nwidth": msg.info.width,
            "\nheight": msg.info.height,
            "\nresolution": msg.info.resolution,
            "\ndata": map_data,
            "\nend": "**********end***********\n"
        }
        print("data: " + map_json["\ndata"])
        map_msg_str = json.dumps(map_json)
        self.netcat.data_transfer(map_msg_str.encode()) 


def main(args=None):

    rclpy.init(args=args)
    netcat = Netcat()

    odom = Odom_sub()
    map = Map_Sub()
    velocity = VelocitySubscriber()

    # Her bir düğüm için ayrı döngülerde spin çağrıları yapılır
    odom_thread = threading.Thread(target=lambda: rclpy.spin(odom))
    map_thread = threading.Thread(target=lambda: rclpy.spin(map))
    velocity_thread = threading.Thread(target=lambda: rclpy.spin(velocity))

    odom_thread.start()
    map_thread.start()
    velocity_thread.start()

    odom_thread.join()
    map_thread.join()
    velocity_thread.join()

    rclpy.shutdown()

    
def main2(args=None):
    rclpy.init(args=args)
    NetcaT_sample=Netcat()

    Odom = Odom_sub()
    Map  = Map_Sub()
    Velocity = VelocitySubscriber()
    
    rclpy.spin_once(Map)
    rclpy.spin_once(Odom)
    rclpy.spin_once(Velocity)

    rclpy.shutdown()
    
if __name__ == '__main__':
    main()