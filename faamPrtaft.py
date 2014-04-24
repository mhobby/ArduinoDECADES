import socket
import struct
from time import sleep
import sys
import StringIO
import csv
import numpy as np
from collections import namedtuple

#udp broadcast packet
with open('prtaftUdp.csv', 'rb') as file:
   udpPkt=file.readlines(1)[0][:-1]

#multicast group & port
C_GRP="239.1.4.6"
C_PORT=50001

#create multicast socket and join group
sC=socket.socket(socket.AF_INET, socket.SOCK_DGRAM, socket.IPPROTO_UDP)
sC.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL,2)

noisyUdp=False
if len(sys.argv)>1:
   if sys.argv[1]=='-n':
      print 'Noisy UDP Multicast packets'
      noisyUdp=True

while True:
   if noisyUdp:
      myRnd=np.random.rand()
      if myRnd<=0.1:
         tmp=list(udpPkt)
         tmp[92]='*'
         txPkt="".join(tmp)
      elif myRnd>0.1 and myRnd<=0.2:
         txPkt=udpPkt[:54]
      elif myRnd>0.2 and myRnd<=0.3:
         tmp=list(udpPkt)
         tmp[10:19]='BADPACKET,'
         txPkt="".join(tmp[:33])      
      else:
         tmp=list(udpPkt)
         tmp[92]='1'
         txPkt="".join(tmp)
   else:
      txPkt=udpPkt
   print >>sys.stderr, '%s:%s < ' % (C_GRP, C_PORT)+txPkt
   sC.sendto(txPkt, (C_GRP, C_PORT))
   sleep(1)
