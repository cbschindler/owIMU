import socket
import struct

import datetime
from math import sin,cos,tan,radians,atan,sqrt

# open socket
socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.bind(('',2018))

while True:
    
    # wait for a request
    request,dist_addr = socket_handler.recvfrom(1024)
    
    hisAddress     = dist_addr[0]
    hisPort        = dist_addr[1]
    
    #asn  = struct.unpack('<HHB',request[-14:-9])
    #counter  = struct.unpack('<h',request[-9:-7])


    Xaccel  = struct.unpack('<h',request[-9:-7])[0]
    Xaccel = Xaccel / 16000.0
    Yaccel  = struct.unpack('<h',request[-11:-9])[0]
    Yaccel = Yaccel / 16000.0
    Zaccel  = struct.unpack('<h',request[-13:-11])[0]
    Zaccel = Zaccel / 16000.0
    try:
    	roll = atan(Yaccel/Zaccel)*180.0/3.14
    except:
    	roll = 999
    try:
    	pitch = atan(-Xaccel/sqrt(Yaccel**2 + Zaccel**2))*180.0/3.14
    except:
    	pitch = 999
    myAddr  = struct.unpack('<H',request[-15:-13])
    temperature    = struct.unpack('<h',request[-17:-15])
    print Xaccel
    print Yaccel
    print Zaccel
    print "\n"
    #print 'received "{0}" from [{1}]:{2}'.format(counter,hisAddress,hisPort)

