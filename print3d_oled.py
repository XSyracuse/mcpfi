import time
from ustruct import unpack, unpack_from
from array import array
from micropython import const
import ssd1306
import framebuf
import micropython
micropython.alloc_emergency_exception_buf(64)

def net():
    import network
    import usocket as socket
    import uselect

    wlan = network.WLAN(network.STA_IF)

    def client_mode():
        
        wlan.active(True)
        if not wlan.isconnected():
            print('connecting to network...')
            wlan.connect('rohr2', '0394747aHeKs')
            while not wlan.isconnected():
                pass
        print('network config:', wlan.ifconfig())
    
    def ap_mode():
        ap = network.WLAN(network.AP_IF)
        ap.active(True)
        ap.config(essid='ESP-3D',password='4242424242')

    client_mode()
    #send socket
    add = ("192.168.128.29",4445)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)
    #rx socket
    rsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rsock.bind(('0.0.0.0',4440))
    rsock.setblocking(False)

    print('listening on 4440')

    _c=0

    poller=uselect.poll()
    poller.register(rsock,uselect.POLLIN)
    def sendudp(data):
        print ('send')
       
        sock.sendto(data,add)

    def rxudp(buf):
        try:
            rsock.readinto(buf)
            #print('rx:')

        except OSError:
            pass

        return

    def get_add():
      return wlan.ifconfig()

    def pole():
         e = poller.poll(1)
         return e

    return rxudp,sendudp,pole,get_add

def sc16is750(uspi,cs):
    # array
    b = bytearray(16)
    # fifo control
    # first byte control
    # w = 0
    # a[3:0] = 0
    c = b'\x80'
     
    # register address byte
    # bit 7 - r=1 w=0
    # bit 6 - a3
    # bit 5 - a2
    # bit 4 - a1
    # bit 3 - a0
    # bit 2 - ch1
    # bit 1 - ch0
    # bit 0 - x
    def read_reg(a):
        v = 0x80 | a<<3
        cs.value(0)
        uspi.write(v.to_bytes(1,'big'))
        reg = uspi.read(1)
        cs.value(1)
        return reg

    def write_reg(a,d):
        v = a<<3
        cs.value(0)
        uspi.write(v.to_bytes(1,'big'))
        uspi.write(d.to_bytes(1,'big'))
        cs.value(1)
        return
    
    def read_fifo():
        rxlvl = read_reg(9)
        rxlvl = int.from_bytes(rxlvl,'big')
        if rxlvl>0:
            cs.value(0)
            uspi.write(c)
            b=uspi.read(rxlvl)
            cs.value(1)
        return rxlvl,b

    def write_fifo(d):
        ack=1
        txlvl = read_reg(8)
        txlvl = int.from_bytes(txlvl,'big')
        if txlvl >= len(d):
            cs.value(0)
            uspi.write(b'\x00')
            uspi.write(d)
            cs.value(1)
        else:
            ack=0
        return ack

    def rd_uart():
        d=[]
        n = 0
        l = True
        while l:
            lsr = read_reg(5)
            lsr = int.from_bytes(lsr,'big') & 0x01
            if lsr==0x01:
                cs.value(0)
                uspi.write(c)
                v=uspi.read(1)
                cs.value(1)
                d.append(int.from_bytes(v,'big'))
                n=n+1
            else:
                l=False
        return n,d

    def uart_init():
        print('init sc16is750')
        #write SPR readback
        write_reg(7,0x86)
        rb = read_reg(7)
        print(rb[0])
        if rb[0] == 0x86:
            print('init ok')
        else:
            print('init err')
        #software reset
        write_reg(0x0e,0x08)
        print(read_reg(0x0e))
        #enhance
        #write_reg(3,0xbf)
        #write_reg(2,0x10) 
        #LCR divisor latch enable
        write_reg(3,0x80)
        #DLL/DLH, 115200
        write_reg(0,0x08)
        write_reg(1,0x00)
        #LCR 8 bits
        write_reg(3,0x03)
        #FCR reset tx and rx fifo
        write_reg(2,0x06)
        #FCR rx fifo 8 char,enable fifo
        write_reg(2,0x01)
        #IER, thr and rhr
        #write_reg(1,0x03)
        print('ier: ')
        print(read_reg(1))

    return uart_init,read_reg,write_reg,write_fifo,read_fifo,rd_uart


def spi():
    from machine import SPI
    from machine import Pin
    from utime import sleep

    def uart_handle(pin):
        print('irq')

    intr = Pin(2,Pin.IN)
    intr.irq(trigger=Pin.IRQ_RISING, handler=uart_handle)

    cspin = Pin(15,Pin.OUT)
    cspin.value(1)

    cs1pin = Pin(16,Pin.OUT)
    cs1pin.value(1)

    hspi = SPI(1,baudrate=4000000,polarity=0,phase=0)
    hspi.init()
    
    i,r,w,wfifo,rfifo,ruart = sc16is750(hspi,cspin)
    i()

    o = 0

    def wr():
        print('wfifo') 
        ack = wfifo(b'\x00\x01\x02\x03\x04\x05\x06\x07\x10\x11\x12\x13\x14\x15\x16\x17')
        if ack==0:
            print('no room')
        #wfifo(b'\xaa\x66\x55\x99')

    def rd():
        print('readback')

        lsr = r(5)
        print('x{0:x}'.format(int.from_bytes(lsr, 'big')) )
    
        for i in range(8):
            lsr1 = r(5)
            lsr1 = int.from_bytes(lsr1,'big')
            lsr = lsr1 & 0x01
            if lsr==1:
                print( '{:02x}'.format(lsr1) )
                print(r(0))

        lsr = r(5)
        print('x{0:x}'.format(int.from_bytes(lsr, 'big')) )

    def rd1():

        nonlocal o
        n,b = ruart()
        print( '{}:{}: {}'.format(o,n,b) )
        o=o+1

    def rd2():
        nonlocal o
        print('readback2')
        lsr = r(5)
        lsr1 = int.from_bytes(lsr,'big') & 0x01

        if lsr1==0x01:          
          print( '{}: {}'.format(o,rfifo()) )

        print('x{0:02x}'.format(int.from_bytes(lsr, 'big')) )
        o=o+1
        
    def write595(v):
        cs1pin.value(0)
        hspi.write(v.to_bytes(1,'big'))
        cs1pin.value(1)

    
    return write595,wr,rfifo

def statscreen(oled,c,d,e,myip):
    oled.fill_rect(0,29,42,32,0)
    oled.fill_rect(44,29,42,32,0)
    oled.fill_rect(88,29,40,32,0)
  
    oled.text(myip,0,0,1) 
    #oled.text('X',0,0,1)
    #oled.text('Y',40,0,1)
    #oled.text('Z',80,0,1)
    oled.text('    00:00:00  0%',0,10,1)

    oled.rect(0,20,128,8,1)
    
    oled.fill_rect(48,29,45,32,d)
    oled.fill_rect(96,29,45,32,e)
    oled.fill_rect(0,29,45,32,c)

    if c==1:
       c=0
    else:
       c=1

    if d==1:
       d=0
    else:
       d=1

    if e==1:
       e=0
    else:
       e=1

    #oled.text(' EXT   BED  FAN ',0,30,c)
    #oled.text(' 220    20  30% ',0,40,c)
    #oled.text('  30    18      ',0,50,c)
    oled.text(' EXT',0,30,c)
    oled.text(' 220',0,40,c)
    oled.text('  30',0,50,c)
    oled.text('BED',58,30,d)
    oled.text('  0',58,40,d)
    oled.text(' 20',58,50,d)
    oled.text('FAN',100,30,e)
    oled.text(' 30%',96,40,e)
      
def code_scr_enc():
    lines = ['','','','','']
    nl = 0

    def code_scr(oled,s):
        nonlocal lines
        lines.pop(0)
        lines.append(s)

        
        oled.fill(0)
        oled.fill_rect(0,0,128,10,1)
        oled.text('G Code',0,1,0)
        oled.text(lines[0],0,12)
        oled.text(lines[1],0,23)
        oled.text(lines[2],0,34)
        oled.text(lines[3],0,45)
        oled.text(lines[4],0,56) 

    return code_scr 
 
def print3d_oled():
    from machine import I2C
    from machine import Pin
    from utime import sleep
    from machine import SPI
    import math
    import utime

    machine.freq(160000000)    

    i2c=I2C(scl=Pin(4),sda=Pin(5),freq=400000)
    print(i2c.scan()) 
    oled = ssd1306.SSD1306_I2C(128,64,i2c)

    
    buf = bytearray(64)
    r,s,p,g = net()

    write595,wr,rd = spi()
 
   
    code_scr = code_scr_enc()
    code_scr(oled,'') 
    
    oled.fill_rect(0,0,128,64,1)
    oled.show()
    sleep(1)
    oled.fill_rect(0,0,128,64,0)
    oled.show()
    sleep(1)
    oled.contrast(100)

    statscreen(oled,1,1,1,'')
           
    oled.show()

    s=''
    v=127
    ext = 0
    scrmode = 0
    while True:
        
        wr()
        wr()
        wr()
        wr()
        sleep(0.05)
        wr()
        wr()
        n,d = rd()
        print(n,d)

        write595(v)
        v=3
       
        
        if ext==1:
          ext=0
        else:
          ext=1

        if scrmode==0:
            my_ip = g()[0]
            
            statscreen(oled,ext,ext,ext,my_ip)
        
        oled.show()
        
        events = p()
        for e in events:
            r(buf)
            t = buf.decode().find('\n')
            print(buf.decode()[0:t])
            s=buf.decode()[0:t]
            code_scr(oled,s)
            scrmode=1

        sleep(0.1)
        
        write595(v)
        v=0
        
        sleep(0.1)


if __name__=='__main__':
    print3d_oled()

