import socket
import json
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

    def conv_data(self,msg):
        #json_msg= json.dumps(msg)
        #self.data_transfer(json_msg)
        pass

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