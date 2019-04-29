import socket
import struct

# open socket
socket_handler = socket.socket(socket.AF_INET6,socket.SOCK_DGRAM)
socket_handler.bind(('',2018))

run = True
while run:
    try:
        # wait for a request
        message, dist_addr = socket_handler.recvfrom(1024)

        hisAddress     = dist_addr[0]
        hisPort        = dist_addr[1]

        ASN = struct.unpack('<HHB',message[0:5])
        addr = format(struct.unpack('<H',message[5:7])[0], 'x')

        temp  = struct.unpack('<h',message[7:9])[0]

        print("({} : {}) temperature = {}".format(ASN, addr, temp))
    except:
        print('Receive failed.')
