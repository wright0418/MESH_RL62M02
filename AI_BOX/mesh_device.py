import _thread
from machine import UART
import binascii

class Mesh_Device():
    def __init__(self,uart_port,baudrate=115200):
        self.uart = UART(uart_port, baudrate)
        self.proved = None
        self.recv_callback= None
        mesh_task = _thread.start_new_thread(self.recv_tesk,())
        # reboot module to get the proved status
        self.uart.write(b'AT+REBOOT\r\n')

    def unprov(self):
        self.uart.write(b'AT+NR\r\n')

    def recv_tesk(self):
        while True:
            try:
                if self.uart.any():
                    recv_data = self.uart.readline().strip().split(b' ')
                    if len(recv_data) < 4:
                        if recv_data[0] == b'PROV-MSG' and recv_data[1] == b'SUCCESS' :
                            self.proved = True
                        if recv_data[0] == b'SYS-MSG' and recv_data[1] == b'DEVICE' and recv_data[2] == b'UNPROV' :
                            self.proved = False
                        continue
                    if recv_data[0] == b'SYS-MSG' and recv_data[1] == b'DEVICE':
                        if recv_data[2] == b'PROV-ED':
                            self.proved = True
                        continue
                    msg_type = recv_data[0]
                    msg_from = recv_data[1]
                    msg_data = binascii.unhexlify(recv_data[3])
                    if self.recv_callback:
                        self.recv_callback(type = msg_type , source = msg_from ,msg = msg_data)
            except Exception as e:
                print ('recv_tesk:',e)
                continue

    def send(self,msg_data):
        msg = b'AT+MDTS 0 ' + binascii.hexlify(msg_data) + b'\r\n'
        self.uart.write(msg)


if __name__ == '__main__':
    def mesh_callback(data):
        print ('recv_data==' , data)

    mesh = Mesh_Device(1)
    mesh.recv_callback = mesh_callback  

    mesh.send(b'\x82\x76\x00\x00')
    while True:

        pass 