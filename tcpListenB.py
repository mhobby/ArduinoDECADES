import socket
import struct
import sys
from collections import namedtuple
data_tuple=namedtuple('data', 'id pktLen time sync temp flightNo rtcRunning udpNo mem field')

B_IP="192.168.1.73"
B_PORT=10002

#open TCP/IP socket and bind.
sB=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sB.bind((B_IP, B_PORT))
sB.listen(5) #listen for any connections including backlog

print >>sys.stderr, '\nlistening on %s:%s' % (B_IP, B_PORT)

conB, addrB=sB.accept() #wait for incoming TCP/IP connections

while True:
   dataB = conB.recv(1024) #receive data
   #read DECADES data packet
   try: 
      data=data_tuple._make(struct.unpack('!9siIBH4shIBh', dataB))
      #print DECADES data packet
      print >>sys.stderr,\
        'ID: %s  pktLen: %d  time: %d sync: %d temp: %d flightNo: %s rtc: %d udpNo: %05d  mem: %d field: %d'\
         % (data.id, data.pktLen, data.time, data.sync, data.temp, \
            data.flightNo, data.rtcRunning, data.udpNo, data.mem, data.field )
   except:
      print >>sys.stderr, 'Error >>>', sys.exc_info()[0]

