import socket
import time


class Sender():
    def __init__(self, addr, port=31901):
        self.tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.tcp_client_socket.connect((addr, 31901))
        print("CONNECTED!")

    def sendvec(self, x, y):
        self.tcp_client_socket.send(('[' + str(x) + ',' + str(y) + "]").encode('UTF-8'))

    def close(self):
        self.tcp_client_socket.close()
        print("Socket CLOSED!")


if __name__ == '__main__':
    sd = Sender('192.168.0.22')
    count = 0
    x = 0
    y = 0
    while 1:
        count = count + 1
        if(count == 100):
            count = 0
        sd.sendvec(x, y)
        time.sleep(0.01)
    sd.close()
