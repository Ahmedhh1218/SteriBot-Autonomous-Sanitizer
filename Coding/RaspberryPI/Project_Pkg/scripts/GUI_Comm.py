import socket
import time
from threading import Timer
import struct
ip_address="192.168.100.111"
s= socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((ip_address,5000))
print('server is now runing.')

#def background_controller():
    #message = 'hello client'
    #print(message)
    #clientsocket.send(bytes(message,"utf-8"))
    #Timer(5, background_controller).start()
    
while 1 :
   # clientsocket,address= s.accept()
   # print(f"connection from {address} has been established.")
   #background_controller()
  print(s.recv(1024).decode('Utf-8'))

    
   