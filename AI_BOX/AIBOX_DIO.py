
from machine import UART,Pin
from utime import sleep_ms,ticks_ms,ticks_diff
import ubinascii as binascii

import _thread
class DigitalOut():
    
    def __init__ (self, io_port):
        self.io_port = Pin(io_port, Pin.OUT)
        self.io_port.value(0)
        
    def set(self, value):
        self.io_port.value(value)
    
    def get(self):
        return self.io_port.value()
    
class DigitalIN():
    
    def __init__ (self, io_port):
        self.io_port = Pin(io_port, Pin.IN, Pin.PULL_UP)
        self.io_callback = None
        self.io_port.irq(trigger=Pin.IRQ_FALLING | Pin.IRQ_RISING, handler=self.IO_call_back)
   
    def get(self):
        return self.io_port.value()
    def IO_call_back(self, io_port):
        # print('IO_call_back:',io_port.value())
        if self.io_callback:
            self.io_callback(io_port.value())

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
        # print (data)
        if data[0:3] == bytes([0x82,0x76,0x00]): # DO type
            if data[3] == 0:
                DO.set(0)
            else:
                DO.set(1)
        
        if data[0:3] == bytes([0x82,0x76,0x81]): # get DI
            if DI.get():
                mesh.send(b'\x82\x76\x01\x00')
            else:
                mesh.send(b'\x82\x76\x01\x01')

    def DI_callback(value):
        # print('DI_callback:',value)
        if value == 1:
            mesh.send(b'\x82\x76\x01\x00')
        else:
            mesh.send(b'\x82\x76\x01\x01')

    #P10 is Relay control
    DO= DigitalOut(Pin.epy.P10)
    
    DI = DigitalIN(Pin.epy.P19)
    DI.io_callback = DI_callback

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

