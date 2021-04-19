import time
import machine
import network
import usocket as socket
import usys as sys
import uselect
import uos

def do_connect():

    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    if not wlan.isconnected():
        print('connecting to network...')
        wlan.connect('rohr2', '0394747aHeKs')
        while not wlan.isconnected():
            pass
    print('network config:', wlan.ifconfig())

def sendudp(ssock, add, i):
    data = 'CMND0x737/mcp/'+i
    print ('send')
    ssock.sendto(data,add)
 
def rxudp(_c,buf,rsock):
    try:
        buf,ad = rsock.recvfrom(64)
        print('rx:')
        print(_c)
        print(buf)
    except OSError:
        _c += 1
    return _c

def ledEnclosed(blue):
  p2 = machine.Pin(2, machine.Pin.OUT)
  on = p2.on
  off = p2.off

  def toggleLED():
    nonlocal blue
    if blue:
      off()
      blue = False
    else:
      on()
      blue = True

  return toggleLED

    

def pin_main():
    
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
          '1B':'MCPSPD_MODE_TOGGLE','2B':'HDGSEL_TOGGLE',
          '3B':'VS_TOGGLE'
         }
          
    machine.freq(160000000)

    print(machine.freq())

    led = ledEnclosed(True)
    tim = machine.Timer(-1)
    tim.init(period=300,mode=machine.Timer.PERIODIC,callback=lambda t:led())
     
    #this mode pin is to select uart or repl mode for uart0
    #nodemcu D6
    modepin = machine.Pin(12,machine.Pin.IN,machine.Pin.PULL_UP)

    if modepin.value()==0:
      print('uart')
      uos.dupterm(None,1)
      uart = machine.UART(0,baudrate=115200)
      gog = uselect.poll()
      gog.register(uart,uselect.POLLIN)
    else:
      print('not in gog')
 
    buf = bytearray(64)

    do_connect()

    #send socket
    address = ("192.168.128.29",49000)
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

    #rx socket
    rsock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rsock.bind(('0.0.0.0',49000))
    rsock.setblocking(False)
     
  
    adc = machine.ADC(0)

    cnt = 0
    _c = 0 
    while True:
        
        events = gog.poll(10)
        for e in events:
            uart.readinto(buf)
            k = buf[0:2].decode('utf-8')
            uart.write(k)
            try:    
                c = cmnd[k]
            except:
                uart.write('!')
                c='' 
            sendudp(sock,address,c) 
        uart.write('.')

        #print('.')
        if cnt==5:
            
            cnt=0
        else:
            cnt += 1
    
        _c = rxudp(_c,buf,rsock)

        time.sleep(0.5)  # Delay for 0.5 second.

   
if __name__ == "__main__":
    pin_main()

