import rclpy
import json
import time
import socket
import threading
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry

class Netcat:
    def __init__(self):
        self.target_host = "192.168.72.214"
        self.target_port = 8910
        self.client = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.connected = False

    def connect(self):
        try:
            self.client.connect((self.target_host, self.target_port))
            self.connected = True
        except ConnectionRefusedError:
            print("Bağlantı hatası: Sunucuya bağlanılamadı.")

    def data_transfer(self, msg):
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

class OdomSub(Node):
    def __init__(self, netcat):
        super().__init__('odom_subscriber')
        self.netcat = netcat
        self.subscription = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

    def odom_callback(self, msg):
        odom_data = {
            "msg_type": "ODOMETRY",
            "x": msg.pose.pose.position.x,
            "y": msg.pose.pose.position.y,
            "w": msg.pose.pose.orientation.w,
            "end": "**********end***********\n"
        }
        odom_msg_str = json.dumps(odom_data)
        self.netcat.data_transfer(odom_msg_str.encode()) 


class MapSub(Node):
    def __init__(self, netcat):
        super().__init__('map_subscriber')
        self.netcat = netcat
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
        map_msg_str = json.dumps(map_json)
        self.netcat.data_transfer(map_msg_str.encode()) 

def main(args=None):
    rclpy.init(args=args)
    netcat = Netcat()
    odom = OdomSub(netcat)
    map_sub = MapSub(netcat)

    executor = MultiThreadedExecutor()
    executor.add_node(odom)
    executor.add_node(map_sub)

    try:
      executor.spin()
    finally:
      executor.shutdown()

    odom.destroy_node()
    map_sub.destroy_node()
    
if __name__ == '__main__':
    main()
