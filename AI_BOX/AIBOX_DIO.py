
from machine import UART,Pin
from utime import sleep_ms,ticks_ms,ticks_diff
import ubinascii as binascii
from mesh_device import Mesh_Device
from micropython import const

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
class Rs485_Agent():
    
    def __init__ (self, port, baudrate,ctl_pin=None):
        self.ctl_pin = Pin(ctl_pin, Pin.OUT)
        self.uart = UART(port, baudrate,timeout=500)
        self.char_3p5_time_ms = 3.5*(8+1+2)/baudrate *1000
        self.ctrl_timebase_ms = (8+1+3)/baudrate *1000

    def send(self, data):
        self.uart.read(self.uart.any())
        self.ctl_pin.value(1)
        self.uart.write(data)
        sleep_ms(int(len(data)*self.ctrl_timebase_ms))
        self.ctl_pin.value(0)
    
    def receive(self,timeout = 500):
        start = ticks_ms()
        while ticks_diff(ticks_ms(),start) < timeout:
            if self.uart.any():
                return self.uart.readline()

if __name__ == '__main__':
    from machine import Pin,LED
    import utime as time
    from utime import ticks_diff,ticks_ms

    # mesh device define packet format
    #          Header(2) + Type(1) + Addr(1) + Length(1) + Data(n)
    # Response Header(2) + Type(1) + Status(1) + Length(1) + Data(n)

    HEADER   = const (b'\x82\x76')
    GET_TYPE = const (b'\x00') 
    SET_TYPE = const (b'\x01')
    RTU_TYPE = const (b'\x02')
    ADDR     = const (b'\x00')
    LENGTH   = const (b'\x01')
    STATUS_OK= const (b'\x80')
    STATUS_ERROR = const (b'\xFE')

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

    def mesh_callback(**msg):
        data = msg['msg']
        if data[0:2] != HEADER:
            return
        if data[2] in (GET_TYPE,SET_TYPE):
            if data[3:4] != ADDR+LENGTH: 
                mesh.send(HEADER + data[2] + STATUS_ERROR)
                return
            if data[2] == GET_TYPE:
                DI_data = b'\x01' if DI.get() else b'\x00'
                mesh.send(HEADER + data[2]+ STATUS_OK +LENGTH+ DI_data)
                return
            if data[2] == SET_TYPE:
                if data[4] == 0:
                    DO.set(0)
                else:
                    DO.set(1)
                mesh.send(HEADER + data[2]+ STATUS_OK)
                return
        elif data[2] == RTU_TYPE:
            # RTU agent
            modbus.send(data[3:0])
            recv_data = modbus.receive(timeout = 200)
            # print (recv_data)
            if recv_data:
                mesh.send(recv_data)

            return
        else :
            mesh.send(HEADER + data[2] + STATUS_ERROR)

    def DI_callback(value):
        # print('DI_callback:',value)
        if value == 1:
            mesh.send(HEADER + GET_TYPE + STATUS_OK + LENGTH + b'\x01')
        else:
            mesh.send(HEADER + GET_TYPE + STATUS_OK + LENGTH + b'\x00')

    #P10 -- Relay control
    DO= DigitalOut(Pin.epy.P10)
    DI = DigitalIN(Pin.epy.P19)
    DI.io_callback = DI_callback
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

