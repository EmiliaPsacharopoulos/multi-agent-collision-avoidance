import socket
import numpy as np
import encodings
from threading import Thread
import threading
import _thread
import time
#from thread import *
import sys
HOST = '35.3.207.175'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)
PORT1 = 65433
CONNECTION_LIST = []
def random_data():          # ANY DATA YOU WANT TO SEND WRITE YOUR SENSOR CODE HERE
    x1 = np.random.randint(0, 55, None)         # Dummy temperature
    y1 = np.random.randint(0, 45, None)         # Dummy humidigy
    my_sensor = "{},{}".format(x1,y1)
    return my_sensor                            # return data seperated by comma
#!usr/bin/python


"""
def clientthread(conn):
    buffer=0
    while True:
        data = conn.recv(1024)
        buffer+=data
        print (buffer)
    #conn.sendall(reply)
    conn.close()
"""

def main():
    try:
        host = '35.3.207.175'
        port = 6666
        tot_socket = 2
        list_sock = []
        for i in range(tot_socket):
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind((host, port+i))
            s.listen(10)
            list_sock.append(s)
            print("clients: ", s)
            print ("[*] Server listening on %s %d" %(host, (port+i)))
            
        conn0, addr0 = list_sock[0].accept()
        conn1, addr1 = list_sock[1].accept()
        counter = 0
        while 1:
            #for j in range(len(list_sock)):
            
            #conn, addr = list_sock[j].accept()
            #print ('[*] Connected with ' + addr0[0] + ':' + str(addr0[1]))
            #_thread.start_new_thread(clientthread ,(conn,))
            my_data = random_data()
            encoded = my_data.encode('utf-8')
            conn0.sendall(encoded)
            conn1.sendall(encoded)
            print(counter)
            counter += 1
            time.sleep(1)
        s.close()

    except KeyboardInterrupt as msg:
        sys.exit(0)


if __name__ == "__main__":
    main()