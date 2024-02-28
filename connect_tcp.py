import socket
import time
import sys

#  TCP_IP = 'localhost'
TCP_IP = '0.0.0.0'
TCP_PORT = 8080 
BUFFER_SIZE = 50 # Normally 1024, but we want fast response

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
s.bind((TCP_IP, TCP_PORT))
s.listen(1)

print('waiting')
conn, addr = s.accept()
print('Connection address:', addr)
time.sleep(3)

v = 0.80
w = 0.0
for i in range(20):
    # try input from keyboard
    txt = input("Input cmd_vel in format [v]: ")
    v = int(txt)
    msg = "%04d,%04d\n" % (v, v)

    #  txt = input("Input cmd_vel in format [v,kP,kI]: ")
    #  txt = txt.split(',')
    #  [v, kP, kI] = [float(tx) for tx in txt]
    #  v = int(v)
    #  msg = "%04d,%04d,%.3f,%.3f\n" % (v, v, kP, kI)

    conn.send(msg.encode())
    print('msg: ', msg)

    #  if i < 20 and v > -0.8:
        #  v -= 0.1
    #  else:
        #  v = 0.0
        #  w += 0.50

    # data = arduino.readline()
    data = conn.recv(BUFFER_SIZE)
    print('receive: ', data)

    #  [l, r] = data.split(',')
    #  l = int(l)
    #  r = int(r)

    time.sleep(0.5)
conn.close()
