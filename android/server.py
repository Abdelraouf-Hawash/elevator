import socket
import time

listenSocket = socket.socket()
port = 8000
maxConnections = 999
hostname = socket.gethostname()
dns_resolved_addr = socket.gethostbyname(hostname)

listenSocket.bind(('',port))
listenSocket.listen(maxConnections)

print("Server is running on " + dns_resolved_addr + " on port " + str(port) )


def inAction(Clientsocket,floor,door):
    mdTest(floor,door)
    Clientsocket.send('ok\r\n'.encode())
    time.sleep(20) # excuting...
    Clientsocket.send('done\r\n'.encode())
    
def acceptConnection():
    (Clientsocket,address) = listenSocket.accept()
    print("new connection accepted ",address)

    message = Clientsocket.recv(1024).decode()
    floor,door = message.split(",")
    inAction(Clientsocket,floor,door)

    time.sleep(0.1)
    Clientsocket.close()
    acceptConnection()
    
acceptConnection()
            
"""
    send message to client by address
        sock.sendto(bytes(512), (address, port))
        address => 192.168.1.- and port => 8000
    
"""
