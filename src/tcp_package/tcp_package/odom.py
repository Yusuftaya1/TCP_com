import rclpy
import json
import time
import socket
from rclpy.node import Node
from nav_msgs.msg import Odometry

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
        self.netcat = Netcat()
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
        time.sleep(2)


def main(args=None):
    rclpy.init(args=args)
    Odom = Odom_sub
    rclpy.spin(Odom)
    rclpy.shutdown()

if __name__ == '__main__':
    main()