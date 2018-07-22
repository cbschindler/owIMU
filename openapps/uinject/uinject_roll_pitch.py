import socket
import struct

import datetime, keyboard, argparse
from math import sin,cos,tan,radians,atan,sqrt
from visualization import *

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--log", action='store_true', help='a logging flag')
parser.add_argument("-v", "--verbose", action='store_true', help='a printing flag')
args = parser.parse_args()

# open socket
socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.bind(('',2018))

now = datetime.datetime.now()
filename = str(now.year)+"-"+str(now.month)+"-"+str(now.day)+"-"+str(now.hour)+"-"+str(now.minute)+"-"+str(now.second)+".csv"
file = open(filename,"w")

init_angles, reset_flag, reset_buf, run = {}, True, [], True

def reset(event):
    global reset_flag
    global reset_buf
    if event.event_type == 'down':
        reset_flag = True
        reset_buf = []

def brk(event):
    global run
    run = False

network = Network.initialize('setup.txt', args.testing)
keyboard.hook_key('r', lambda event: reset(event), suppress=False)
keyboard.hook_key('q', lambda event: brk(event), suppress=False)

while run:
    try:
        # wait for a request
        message, dist_addr = socket_handler.recvfrom(1024)

        hisAddress     = dist_addr[0]
        hisPort        = dist_addr[1]

        ASN = struct.unpack('<HHB',message[0:5])
        addr = format(struct.unpack('<H',message[5:7])[0], 'x')

        accelX  = struct.unpack('<h',message[7:9])
        accelY  = struct.unpack('<h',message[9:11])
        accelZ  = struct.unpack('<h',message[11:13])
        gyro0 = struct.unpack('<h',message[13:15])
        gyro1 = struct.unpack('<h',message[15:17])
        gyro2 = struct.unpack('<h',message[17:19])
        roll = struct.unpack('<f',message[19:23])
        pitch = struct.unpack('<f',message[23:27])
        yaw = struct.unpack('<f',message[27:31])

        if reset_flag and not reset_buf.contains(addr):
            init_angles[addr] = (roll, pitch)
            reset_buf.append(addr)
            if len(reset_buf) == len(network): # have we updated every mimsy
                reset_flag = False
                reset_buf = []

        network.update(data=(roll-init_angles[addr][0], pitch-init_angles[addr][1]), addr=formattedAddr)

        data_x = 'g_x: ' + str(accelX)
        data_y = 'g_y: ' + str(accelY)
        data_z = 'g_z: ' + str(accelZ)
        data_r = 'roll: ' + roll
        data_p = 'pitch: ' + pitch
        data_y = 'yaw: ' + yaw
        data_addr = 'addr: ' + addr
        data_asn = 'ASN: ' + str(ASN)

        data = data_x + ', ' + data_y + ', ' + data_z + ', ' + data_r + ', ' + \
                data_r + ', ' + data_p + ', ' + data_y + ', ' + data_addr + ', ' + \
                data_asn

        if args.verbose:
            print(data)
        if args.log:
            file.write(data + '\n')
    except:
        print('Receive failed.')
