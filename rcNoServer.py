#!/usr/bin/python
import pygatt
import time
from binascii import hexlify

# Many devices, e.g. Fitbit, use random addressing - this is required to
# connect.
# sudo gatttool -t random -b C4:D0:0D:79:59:91 -I
# connect
# char-desc
# 2d30c083-f39f-4ce6-923f-3484ea480596 zum schreiben


ADDRESS_TYPE   = pygatt.BLEAddressType.random
DEVICE_ADDRESS = "c9:67:1e:d6:db:d7"
adapter = pygatt.GATTToolBackend()

mytime = time.time()

def handle_data(handle, value):
	global mytime
	print("Received data: %d" % (value[0]<<8 | value[1]))
	print (time.time()-mytime)*1000
	mytime = time.time()

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
        time.sleep(100)
        
except KeyboardInterrupt:
    print('kill signal')
 
finally:
    print('Adapter Stop')
    adapter.stop()
