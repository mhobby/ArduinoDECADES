import socket
import struct
import sys
from collections import namedtuple
data_tuple=namedtuple('data', 'pktid pktLen time sync temp flightNo rtcRunning udpNo mem field')

A_IP="192.168.1.73"
A_PORT=3502

#open TCP/IP socket and bind.
sA=socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sA.bind((A_IP, A_PORT))
sA.listen(5) #listen for any connections including backlog

print >>sys.stderr, '\nlistening on %s:%s' % (A_IP, A_PORT)

conA, addrA=sA.accept() #wait for incoming TCP/IP connections

while True:
   dataA = conA.recv(1024) #receive data
   #read DECADES data packet
   try:
      data=data_tuple._make(struct.unpack('!9siIBH4shIBh', dataA))
      #print DECADES data packet
      print 'ID: %s  pktLen: %d  time: %d sync: %d temp: %d flightNo: %s  rtc: %d udpNo: %05d  mem: %d field: %d'\
         % (data.pktid, data.pktLen, data.time, data.sync, data.temp, data.flightNo, data.rtcRunning,\
            data.udpNo, data.mem, data.field )
   except:
      print 'Error >>>', sys.exc_info()[0]
