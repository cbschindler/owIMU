import socket
import struct

import datetime, keyboard, argparse, traceback
from math import sin,cos,tan,radians,atan,sqrt,degrees,radians
from visualization import *
from time import time

parser = argparse.ArgumentParser()
parser.add_argument("-l", "--log", action='store_true', help='a logging flag')
parser.add_argument("-v", "--verbose", action='store_true', help='a printing flag')
args = parser.parse_args()

# open socket
socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.bind(('',2018))

if args.log:
    now = datetime.datetime.now()
    filename = str(now.year)+"-"+str(now.month)+"-"+str(now.day)+"-"+str(now.hour)+"-"+str(now.minute)+"-"+str(now.second)+".csv"
    file = open(filename,"w")

init_angles, reset_flag, reset_buf, run, show_raw  = {}, True, [], True, True
time_mappings = {}

network = Network.initialize('setup.txt', False)

def reset(event):
    global reset_flag
    global reset_buf
    if event.event_type == 'down':
        reset_flag = True
        reset_buf = []

def brk(event):
    global run
    run = False

def toggleRaw(event):
    global show_raw
    if event.event_type == 'down':
        show_raw = not show_raw

def consoleUpdate(event):
    global time_mappings
    global show_raw
    global reset_flag
    global reset_buf
    now = time()
    if event.event_type == 'down':
        print("")
        for mimsy in time_mappings.keys():
            print(mimsy + ": " + str(now - time_mappings.get(mimsy)) + " seconds since last update")
        if show_raw:
            print("Showing NATIVE angle data.")
        else:
            print("Showing RELATIVE angle data.")
            for mimsy in init_angles.keys():
                print("Offset Angles for " + mimsy + ": roll=" + str(init_angles[mimsy][0]) + " | pitch=" + str(init_angles[mimsy][1]))  
        if reset_flag:
            print("Reset Buffer (size=" + str(len(reset_buf)) + ", missing=" + str(len(network)-len(reset_buf)) +  "): " + str(reset_buf))
        else:
            print("Reset flag not set.")
        print("")

keyboard.hook_key('r', lambda event: reset(event), suppress=False)
keyboard.hook_key('n', lambda event: toggleRaw(event), suppress=False)
keyboard.hook_key('q', lambda event: brk(event), suppress=False)
keyboard.hook_key('u', lambda event: consoleUpdate(event), suppress=False)

while run:
    try:
        # wait for a request
        message, dist_addr = socket_handler.recvfrom(1024)

        timestamp = time()

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
        
        if reset_flag and addr not in reset_buf:
            init_angles[addr] = (roll, pitch)
            reset_buf.append(addr)
            if len(reset_buf) == len(network): # have we updated every mimsy
                reset_flag = False
                reset_buf = []
                
        if not show_raw:
            network.update(data=(roll-init_angles[addr][0], pitch-init_angles[addr][1]), addr=addr)
        else:
            network.update(data=(roll, pitch), addr=addr)

        time_mappings[addr] = timestamp

        data_ax = 'g_x: ' + str(accelX)
        data_ay = 'g_y: ' + str(accelY)
        data_az = 'g_z: ' + str(accelZ)
        data_r = 'roll: ' + str(roll)
        data_p = 'pitch: ' + str(pitch)
        data_y = 'yaw: ' + str(yaw)
        data_addr = 'addr: ' + str(addr)
        data_asn = 'time[s]: ' + str(0.01*(ASN[0] + ASN[1]*(2**16) + ASN[2]*(2**16)))
        data_roff = 'roll_offset: ' + str(init_angles[addr][0])
        data_poff = 'pitch_offset: ' + str(init_angles[addr][1])

        sep = ", "

        data = data_ax + sep + data_ay + sep + data_az + sep + data_r + sep + \
                data_p + sep + data_y + sep + data_addr + sep + data_asn + sep + \
                data_roff + sep + data_poff

        if args.verbose:
            print(data)
        if args.log:
            file.write(data + '\n')
    except Exception, err:
        print('Receive failed.')
        print('Printing traceback...')
        traceback.print_exc()
