import socket,pickle
import threading
import time


HOST = '35.3.197.134'  # The server's hostname or IP address
PORT = 6666      # The port used by the server

def init():
    global path
    path = [[]]
    global port_num
    port_num = PORT
    my_client()


def process_data_from_server(x):

    for i in range (len(x)):
        if i%2 == 0:
            path[i/2][0] = x[i]
        else:
            path[i//2][1] = x[i]

    #x1, y1 = x.split(",")
    #return x1,y1



def my_client():
    threading.Timer(11, my_client).start()

    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
        s.connect((HOST, PORT))
        if(True):

            """
            my = input("Enter command ")

            my_inp = my.encode('utf-8')

            s.sendall(my_inp)
            """

            my = "Data"

            my_inp = my.encode('utf-8')

            s.sendall(my_inp)
            data = s.recv(1024)

            path = pickle.loads(data)
            #process_data_from_server(data)
            

            #print("Temperature {}".format(x_temperature))
            #print("Humidity {}".format(y_humidity))
            #path = s.recv(1024).decode('utf-8')

        #s.close()
            #time.sleep(5)
        return path


if __name__ == "__main__":
    init()
