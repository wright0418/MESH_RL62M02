
from machine import UART,Pin
from utime import sleep_ms,ticks_ms,ticks_diff
import ubinascii as binascii

import _thread
class Rs485_Agent():
    
    def __init__ (self, port, baudrate,ctl_pin=None):
        self.ctl_pin = Pin(ctl_pin, Pin.OUT)
        self.uart = UART(port, baudrate,timeout=10)
        self.char_3p5_time_ms = 3.5*(8+1+2)/baudrate *1000
        self.ctrl_timebase_ms = (8+1+3)/baudrate *1000

    def send(self, data):
        self.uart.read(self.uart.any())
        self.ctl_pin.value(1)
        self.uart.write(data)
        sleep_ms(int(len(data)*self.ctrl_timebase_ms))
        self.ctl_pin.value(0)
    
    def receive(self,timeout = 50):
        start = ticks_ms()
        while ticks_diff(ticks_ms(),start) < timeout:
            if self.uart.any():
                return self.uart.readline()

class Mesh_Device():
    def __init__(self,uart_port,baudrate=115200):
        self.uart = UART(uart_port, baudrate)
        self.proved = None
        self.setup_recv_callback = None
        mesh_task = _thread.start_new_thread(self.recv_tesk,())
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
                    # print ('recv_data[3]:',recv_data[3])
                    msg_data = binascii.unhexlify(recv_data[3])
                    if self.recv_callback:
                        self.recv_callback(msg_data)
            except Exception as e:
                print ('recv_tesk:',e)
                continue

    def set_recv_callback(self,callback):
        self.recv_callback = callback   
    
    def send(self,msg_data):
        msg = b'AT+MDTS 0 ' + binascii.hexlify(msg_data) + b'\r\n'
        self.uart.write(msg)

if __name__ == '__main__':
    from machine import Pin,LED
    import utime as time
    from utime import ticks_diff,ticks_ms
    key_pushed_time = 0
    key_state = "release"

    def check_key_time(pin):
        global key_state,key_pushed_time
        if key_state =='release' and pin.value() == 0: #pushed
            key_pushed_time = time.ticks_ms()
            key_state = 'pushed'
        elif key_state == 'pushed' and pin.value() == 1: #release
            key_state = 'release'
        elif key_state =='pushed' and pin.value() == 0: #release
            return time.ticks_diff(time.ticks_ms(),key_pushed_time)
        return 0
        
        
    g_led = LED('ledg')
    unprov_KEY = Pin(Pin.epy.KEYA,Pin.IN,Pin.PULL_UP)

    def mesh_callback(data):
        modbus.send(data)
        recv_data = modbus.receive(timeout = 200)
        # print (recv_data)
        mesh.send(recv_data)

    uart_port = 0
    modbus = Rs485_Agent(uart_port, baudrate = 4800 ,ctl_pin=Pin.epy.KEYB)
    mesh = Mesh_Device(1)
    mesh.set_recv_callback(mesh_callback)

    while True:
        if mesh.proved:
            g_led.on()
        else:
            g_led.toggle()
        if check_key_time(unprov_KEY) > 5000:
            g_led.off()
            mesh.unprov()
        time.sleep(0.5)

