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
    add = ("192.168.128.60",49000)
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

    def get_send_ip():
      return add

    def pole():
         e = poller.poll(1)
         return e

    return rxudp,sendudp,pole,get_add,get_send_ip

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
        else:
            b=bytearray(1)
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
        #DLL/DLH, 115200, 08, 38400, 24
        write_reg(0,24)
        write_reg(1,0x00)
        #LCR 8 bits
        write_reg(3,0x03)
        #FCR reset tx and rx fifo
        write_reg(2,0x06)
        #FCR rx fifo 8 char,enable fifo
        write_reg(2,0x01)
        #IER, thr and rhr
        write_reg(1,0x01)
        print('ier: ')
        print(read_reg(1))

    return uart_init,read_reg,write_reg,write_fifo,read_fifo,rd_uart


def spi(mcp_cmd):
    from machine import SPI
    from machine import Pin
    from utime import sleep

    intr = Pin(2,Pin.IN)
    
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

        
    def write595(v):
        cs1pin.value(0)
        hspi.write(v.to_bytes(1,'big'))
        cs1pin.value(1)

    def clr_irq():
        iir = r(2)
        #print(iir)
        n,d = rfifo()
        #print(n,d)
        mcp_cmd(d)
        
    def uart_handle(pin):
        nonlocal rfifo
        nonlocal r
        #print('irq')
        clr_irq()

    intr.irq(trigger=Pin.IRQ_FALLING, handler=uart_handle)


    return write595,wr,rfifo,clr_irq


def mcp():
    cmnd={'0+':'NAV1_COURSE+1','0-':'NAV1_COURSE-1',
          '8+':'NAV1_COURSE+1','8-':'NAV1_COURSE-1', 
          '1+':'MCPSPD+1','1-':'MCPSPD-1',
          '9+':'MCPSPD+10','9-':'MCPSPD-10', 
          '2+':'HDG+1','2-':'HDG-1',
          'A+':'HDG+6','A-':'HDG-6', 
          '3+':'VVI+100','3-':'VVI-100',
          'B+':'VVI+500','B-':'VVI-500',
          '4+':'ALTSEL+100','4-':'ALTSEL-100',
          'C+':'ALTSEL+1000','C-':'ALTSEL-1000',
          '0B':'MCPSPD_MODE_TOGGLE','1B':'HDGSEL_TOGGLE',
          '2B':'VS_TOGGLE'
         }

    cmd_list = []

    def add(b):
        cmd_list.append(b)
        print(cmd_list)

    def rm():
        try:
            d = cmd_list[0]
            cmd_list.pop(0)
        except:
            d=bytearray(1)
           
        return d

    def mcp_cmd(buf):
        k = buf[0:2].decode('utf-8')
        try:    
            c = cmnd[k]
        except:
            print('!')
            c='' 
        d = 'CMND0x737/mcp/'+c
        return d

    return mcp_cmd,add,rm

def statscreen(oled,c,d,e,myip,x_ip,xport):
    oled.fill_rect(0,0,160,60,0)

    y=0 
    if c>6:
        y=16
    elif c>2:
        y=8

    oled.text(myip,0,y+0,1)   
    oled.text(x_ip,0,y+16,1)
    oled.text(xport,0,y+32,1)
 
      
def code_scr_enc():
    lines = ['','','','','']
    nl = 0

    def code_scr(oled,s):
        nonlocal lines
        lines.pop(0)
        lines.append(s)

        
        oled.fill(0)
        oled.fill_rect(0,0,128,10,1)
        oled.text('MCP',0,1,0)
        oled.text(lines[0],0,12)
        oled.text(lines[1],0,23)
        oled.text(lines[2],0,34)
        oled.text(lines[3],0,45)
        oled.text(lines[4],0,56) 

    return code_scr 
 
def mcpfi_oled():
    import machine
    from machine import I2C
    from machine import Pin
    from utime import sleep
    import math
    import utime

    machine.freq(160000000)    

    i2c=I2C(scl=Pin(4),sda=Pin(5),freq=400000)
    print(i2c.scan()) 
    oled = ssd1306.SSD1306_I2C(128,64,i2c)

    buf = bytearray(64)
    r,s,p,g,x = net()

    mcp_cmd,add,rm = mcp()
    write595,wr,rd,clr_irq = spi(add)
  
    
    code_scr = code_scr_enc()
    code_scr(oled,'') 
    
    oled.fill_rect(0,0,128,64,1)
    oled.show()
    sleep(1)
    oled.fill_rect(0,0,128,64,0)
    oled.show()
    sleep(1)
    oled.contrast(100)

    statscreen(oled,1,1,1,'','','')
           
    oled.show()
    t=0
    msg=''
    v=127
    ext = 0
    scrmode = 0
    while True:
  
        #make mcp command

        r=rm()
        while len(r)>2:
            d=mcp_cmd(r)
            print(d)
            s(d)
            r=rm()

        t=t+1
        if t==10:
            print('clr irq')
            clr_irq()
            t=0

        write595(v)
        v=3
       
        
        if ext==1:
          ext=0
        else:
          ext=1

        if scrmode==0:
            my_ip = g()[0]
            xip = x()[0]
            xport = x()[1]

            statscreen(oled,t,ext,ext,my_ip,xip,str(xport))
        
        oled.show()
        
        events = p()
        for e in events:
            r(buf)
            t = buf.decode().find('\n')
            print(buf.decode()[0:t])
            s=buf.decode()[0:t]
            code_scr(oled,msg)
            scrmode=1

        sleep(0.1)
        
        write595(v)
        v=0
        
        sleep(0.1)


if __name__=='__main__':
    mcpfi_oled()

