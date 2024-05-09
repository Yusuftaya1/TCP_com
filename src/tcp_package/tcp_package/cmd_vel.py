import rclpy
import json
import time
import socket
from rclpy.node import Node
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
        time.sleep(2) 

def main(args=None):
    rclpy.init(args=args)
    cmd_vel= VelocitySubscriber()
    rclpy.spin(cmd_vel)
    rclpy.shutdown()

if __name__ == '__main__':
    main()