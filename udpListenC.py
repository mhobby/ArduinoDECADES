import socket
import struct
import sys
import StringIO
import csv
from collections import namedtuple
data_tuple=namedtuple('data', 'id pktLen flightNo udpNo rtcRunning rsvd time sync temp field')

#multicast group & port
C_GRP="239.1.4.6"
C_PORT=50001

#create multicast socket and join group
sC=socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sC.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
sC.bind(('', C_PORT))
mreq = struct.pack("5sl", socket.inet_aton(C_GRP), socket.INADDR_ANY)
sC.setsockopt(socket.IPPROTO_IP, socket.IP_ADD_MEMBERSHIP, mreq)

print >>sys.stderr, '\nlistening on %s:%s' % (C_GRP, C_PORT)
while True:
    dataC, address = sC.recvfrom(1024) #receive data
    print dataC
