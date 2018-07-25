import socket
import struct

import datetime, keyboard, argparse
from math import sin,cos,tan,radians,atan,sqrt,degrees,radians
#from visualization import *

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--log", action='store_true', help='a logging flag')
parser.add_argument("-v", "--verbose", action='store_true', help='a printing flag')
args = parser.parse_args()

# open socket
socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.bind(('',2018))

#now = datetime.datetime.now()
#filename = str(now.year)+"-"+str(now.month)+"-"+str(now.day)+"-"+str(now.hour)+"-"+str(now.minute)+"-"+str(now.second)+".csv"
#file = open(filename,"w")

#init_angles, reset_flag, reset_buf, run = {}, True, [], True

'''def reset(event):
    global reset_flag
    global reset_buf
    if event.event_type == 'down':
        reset_flag = True
        reset_buf = []'''

'''def brk(event):
    global run
    run = False'''

#network = Network.initialize('setup.txt', args.testing)
#keyboard.hook_key('r', lambda event: reset(event), suppress=False)
#keyboard.hook_key('q', lambda event: brk(event), suppress=False)
run = True
while run:
    try:
        # wait for a request
        message, dist_addr = socket_handler.recvfrom(1024)

        hisAddress     = dist_addr[0]
        hisPort        = dist_addr[1]

        ASN = struct.unpack('<HHB',message[0:5])
        addr = format(struct.unpack('<H',message[5:7])[0], 'x')

        accelX  = struct.unpack('<h',message[7:9])[0]
        accelY  = struct.unpack('<h',message[9:11])[0]
        accelZ  = struct.unpack('<h',message[11:13])[0]
        gyro0 = struct.unpack('<h',message[13:15])[0]
        gyro1 = struct.unpack('<h',message[15:17])[0]
        gyro2 = struct.unpack('<h',message[17:19])[0]
        roll = degrees(struct.unpack('<f',message[19:23])[0])
        pitch = degrees(struct.unpack('<f',message[23:27])[0])
        yaw = degrees(struct.unpack('<f',message[27:31])[0])
        
        '''print "time [s]: " + str()
            print "address: " + str(addr)
            print "accelX: " + str(accelX)
            print "accelY: " + str(accelY)
            print "accelZ: " + str(accelZ)
            print "pitch: " + str(degrees(pitch))
            print "roll: " + str(degrees(roll))
            print "yaw: " + str(degrees(yaw))
            print "\n"'''
        
        '''if reset_flag and not reset_buf.contains(addr):
            init_angles[addr] = (roll, pitch)
            reset_buf.append(addr)
            if len(reset_buf) == len(network): # have we updated every mimsy
                reset_flag = False
                reset_buf = []'''
                

        #network.update(data=(roll-init_angles[addr][0], pitch-init_angles[addr][1]), addr=formattedAddr)

        data_accelx = 'g_x: ' + str(accelX)
        data_accely = 'g_y: ' + str(accelY)
        data_accelz = 'g_z: ' + str(accelZ)
        data_r = 'roll: ' + str(roll)
        data_p = 'pitch: ' + str(pitch)
        data_y = 'yaw: ' + str(yaw)
        data_addr = 'addr: ' + str(addr)
        data_time = 'time[s]: ' + str(0.01*(ASN[0] + ASN[1]*(2**16) + ASN[2]*(2**16)))

        sep = "\n"

        data = data_accelx + sep + data_accely + sep + data_accelz + sep + data_r + sep + \
                data_p + sep + data_y + sep + data_addr + sep + \
                data_time
        print data + "\n"

        '''if args.verbose:
            print(data)
        if args.log:
            file.write(data + '\n')'''
    except:
        print('Receive failed.')
