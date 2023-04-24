
import socket,pickle
import numpy as np
import encodings
from threading import Thread
import threading
import _thread
import time
#from thread import *
import sys
from MAPF import cbs_wrapper, cbs


HOST = '35.3.105.32'  # Standard loopback interface address (localhost)
PORT = 65432        # Port to listen on (non-privileged ports are > 1023)
PORT1 = 65433
CONNECTION_LIST = []

def input():
    cbs = cbs_wrapper.CBS_WRAPPER()
    cbs.set_map("MAPF/instances/demo_hard_2.txt")
    path = cbs.solve(show_animation=False)
    print("whole path: ", path)
    return path

def input1():
    path = input()
    print("first command: ", path[0])
    return path[0]

def input2():
    path = input()
    print("second command: ", path[1])
    return path[1]

def input3():
    path = input()
    print("third command: ", path[2])
    return path[2]

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
    """
    input()
    input1()
    input2()
    input3()
    """
    try:
        host = '35.3.105.32'
        port = 6666
        tot_socket = 3
        list_sock = []
        #list_sock = [0,0,0]
        for i in range(tot_socket):
            s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            s.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
            s.bind((host, port+i))
            s.listen(10)
            list_sock.append(s)
            #list_sock[i] = s
            print("clients: ", s)
            print ("[*] Server listening on %s %d" %(host, (port+i)))
            
        conn0, addr0 = list_sock[0].accept()
        print("conn0: ", conn0)
        conn1, addr1 = list_sock[1].accept()
        conn2, addr2 = list_sock[2].accept()
        counter = 0

        


        while (1):
            #for j in range(len(list_sock)):
            
            #conn, addr = list_sock[j].accept()
            #print ('[*] Connected with ' + addr0[0] + ':' + str(addr0[1]))
            #_thread.start_new_thread(clientthread ,(conn,))
            if (counter == 0):
                my_data = input1()
                encoded = pickle.dumps(my_data)

                sec_data = input2()
                encoded2 = pickle.dumps(sec_data)

                third_data = input3()
                encoded3 = pickle.dumps(third_data)

                conn0.sendall(encoded)
                print("first client sent: ", my_data)
                print("after sent conn0: ", conn0)
                
                conn1.sendall(encoded2)
                print("second client sent: ", sec_data)
                print("after sent conn1: ", conn1)
                conn2.sendall(encoded3)
                print("after sent conn2: ", conn2)
                print("third client sent: ", third_data)
                print("counter:", counter,'\n')
               
                #print("data 2:", sec_data,'\n')
            

            counter += 1

            time.sleep(1)
        
        s.close()

    except KeyboardInterrupt as msg:
        sys.exit(0)


if __name__ == "__main__":
    main()
