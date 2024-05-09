import rclpy
import json
import time
import socket
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid

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

class Map_Sub(Node):
    def __init__(self):
        super().__init__('map_subscriber')
        self.subscription = self.create_subscription(OccupancyGrid, 'map', self.map_callback, 10)
        self.netcat = Netcat()

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
        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    Map = Map_Sub()
    rclpy.spin(Map)
    rclpy.shutdown()

if __name__ == '__main__':
    main()