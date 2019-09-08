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

#12345678901234567890ABCDEFGHIJKLNOPQRSTU12345678901234567890ABCDEFGHIJKLNOPQRSTU12345678901234567890ABCDEFGHIJKLNOPQRSTU12345678901234567890ABCDEFGHIJKLNOPQRSTU12345678901234567890ABCDEFGHIJKLNOPQRSTU
#ABCDEFGHIJKLNOPQRSTU

ADDRESS_TYPE   = pygatt.BLEAddressType.random
DEVICE_ADDRESS = "c0:08:80:00:08:80"
adapter = pygatt.GATTToolBackend()


def handle_data(handle, value):
	print(value)

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
		device.char_write("00008882-0000-1000-8000-00805f9b34fb", [65,66,67,13,10], wait_for_response=False)
		#time.sleep(1)
        
except KeyboardInterrupt:
    print('kill signal')
 
finally:
    print('Adapter Stop')
    adapter.stop()