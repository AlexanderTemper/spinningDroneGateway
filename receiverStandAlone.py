#!/usr/bin/python
import pygatt
import time
import binascii
import socket

TCP_IP = '127.0.0.1'
TCP_PORT = 5005

# Many devices, e.g. Fitbit, use random addressing - this is required to
# connect.
# sudo gatttool -t random -b C4:D0:0D:79:59:91 -I
# connect
# char-desc
# 2d30c083-f39f-4ce6-923f-3484ea480596 zum schreiben

ADDRESS_TYPE   = pygatt.BLEAddressType.random
DEVICE_ADDRESS = "c0:08:80:00:08:80"
adapter = pygatt.GATTToolBackend()

s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.bind((TCP_IP, TCP_PORT))
print("socket binded to post", TCP_PORT) 
s.listen(1)
conn, addr = s.accept()
print('Connected to :', addr[0], ':', addr[1]) 


def send_data(device,data):
    if not device:
        return None
    try:
        device.char_write("00008882-0000-1000-8000-00805f9b34fb", bytearray(data), wait_for_response=False)
        return True
    except pygatt.exceptions.NotConnectedError:            
        print("write failed")
        return None


def handle_data(handle, value):
    conn.sendall(value)
    #print("Received data: %s" % (value))
    


def connect():
    print("Try connecting to "+DEVICE_ADDRESS);
    try:
        device = adapter.connect(DEVICE_ADDRESS, address_type=ADDRESS_TYPE)
        print("connected to "+DEVICE_ADDRESS)
        return device
    except pygatt.exceptions.NotConnectedError:
        return None;


try:
    adapter.start()
    device = connect()
    while not device:
        device = connect()
        
    device.subscribe("00008881-0000-1000-8000-00805f9b34fb",callback=handle_data)
    
    while True:
        time.sleep(0.01)
        # data received from client 
        data = conn.recv(20) 
        if data:
            while not send_data(device,data):
                device = connect()
                if device:
                    device.subscribe("00008881-0000-1000-8000-00805f9b34fb",callback=handle_data)
        
        
except KeyboardInterrupt:
    print('kill signal')

finally:
    print('Adapter Stop')
    adapter.stop()
    conn.close()
