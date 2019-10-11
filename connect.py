#!/usr/bin/python
import pygatt
import time
import sys
from binascii import hexlify
from inputs import get_gamepad
from threading import Thread

withConfig = False;
taranis = True;
if withConfig:
	import socket
	
	TCP_IP = '127.0.0.1'
	TCP_PORT = 5005
	s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
	s.bind((TCP_IP, TCP_PORT))
	print("socket binded to post", TCP_PORT) 
	s.listen(1)
	conn, addr = s.accept()
	print('Connected to :', addr[0], ':', addr[1]) 

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

def progress(count, total, status=''):
    bar_len = 5
    filled_len = int(round(bar_len * count / float(total)))

    percents = round(100.0 * count / float(total), 1)
    bar = '=' * filled_len + '-' * (bar_len - filled_len)

    sys.stdout.write('%s[%s]' % (status,bar))
    sys.stdout.flush()  

def millis():
	return int(round(time.time() * 1000))

class ps3joyClass:
	throttle = 1000  #intervall 1000 <-> 2000
	yaw = 1500
	pitch = 1500
	roll = 1500
	arm = 1000
	timeout = millis()
	
	def scale(self,raw):
		return int((raw / 255.0) *1000 +1000)
	def show(self):
		progress(self.throttle-1000, 1000, status='T')
		sys.stdout.write(' ')
		progress(self.yaw-1000, 1000, status='Y')
		sys.stdout.write(' ')
		progress(self.pitch-1000, 1000, status='P')
		sys.stdout.write(' ')
		progress(self.roll-1000, 1000, status='R')
		sys.stdout.write(' A[%s]' % ( 'X' if self.arm > 1000 else '.'))
		sys.stdout.write('\t[%s,%s,%s,%s|%s]' % (self.throttle,self.yaw,self.pitch,self.roll,self.arm))
		sys.stdout.write('\r')
		sys.stdout.flush()  
		#print (self.throttle,self.yaw,self.pitch,self.roll)
	def reset(self):
		self.throttle = 1000
		self.yaw = 1500
		self.pitch = 1500
		self.roll = 1500
		self.arm = 1000
		
	def todataArray(self):
		return [
			0x00FF & self.roll, self.roll >> 8,
			0x00FF & self.pitch, self.pitch >> 8,
			0x00FF & self.yaw, self.yaw >> 8,
			0x00FF & self.throttle, self.throttle >> 8,
			0x00FF & self.arm, self.arm >> 8,
			]
		
	def update(self,events):
		for event in events:
			self.timeout = millis()
			if taranis:
				#print(event.ev_type, event.code, event.state)
				if event.code == 'ABS_X':
					self.throttle = self.scale(event.state+127)
				elif event.code == 'ABS_Z':
					self.pitch = self.scale(event.state+127)
				elif event.code == 'ABS_Y':
					self.roll = self.scale(event.state+127)
				elif event.code == 'ABS_RX':
					self.yaw = self.scale(event.state+127)
				elif event.code == 'ABS_RZ':
					self.arm = self.scale(event.state+127)
			else:
				if event.code == 'ABS_X':
					self.yaw = self.scale(event.state)
					self.timeout = millis()
				#elif event.code == 'ABS_RZ':
					#self.throttle = int((event.state / 255.0) *500 +1000)
				elif event.code == 'ABS_RY':
					self.pitch = self.scale(255-event.state)
				elif event.code == 'ABS_RX':
					self.roll = self.scale(event.state)
				elif event.code == 'BTN_TR':
					self.arm = self.scale(event.state*255)
				elif event.code == 'BTN_SOUTH':
					self.throttle = self.throttle + event.state*5;
				elif event.code == 'BTN_EAST':
					self.throttle = self.throttle - event.state*5;
				#elif event.code != 'SYN_REPORT':
					#print(event.ev_type, event.code, event.state)
		
		if millis()-self.timeout > 500:
			self.reset()
			print ("Joy Timout Reset")
		
ps3joy = ps3joyClass()


stop_threads = False
def joythread(threadname):
    while True:
		try: 
			ps3joy.update(get_gamepad())

			if stop_threads == 1:
				break
		except:
			ps3joy.reset();
			print ("No Joypad found")
			return
		
def send_MSP(command,data):
	outdata = []
	outdata.append(0x24)
	outdata.append(0x4d)
	outdata.append(0x3c)
	
	outdata.append(len(data))
	outdata.append(command)
	
	crc = []
	crc.append(len(data))
	crc.append(command)
	
	for x in data:
		outdata.append(x)
		crc.append(x)
	
	outdata.append(xor(crc))
	
	#print(outdata)
	
	return outdata
	
def xor(data):
	erg = 0
	for x in data:
		erg = erg ^ x
		
	return erg

def handle_data(handle, value):
	if withConfig:
		conn.sendall(value)
	#print("Received data: %s" % value)
	#print("Received datahex : %s" % hexlify(value))

def connect():
    print("Try connecting to "+DEVICE_ADDRESS);
    try:
        device = adapter.connect(DEVICE_ADDRESS, address_type=ADDRESS_TYPE)
        print("connected to "+DEVICE_ADDRESS)
        return device
    except pygatt.exceptions.NotConnectedError:
        return None

def send_data():
	if not device:
		return None
	try:
		device.char_write("00008882-0000-1000-8000-00805f9b34fb", send_MSP(200,ps3joy.todataArray()), wait_for_response=False)#RC SET ROW
		#device.char_write("00008882-0000-1000-8000-00805f9b34fb", send_MSP(105,[]), wait_for_response=False)#RC SET ROW
		#device.char_write("00008882-0000-1000-8000-00805f9b34fb", send_MSP(10,[]), wait_for_response=False)#Reply Gateway
		#device.char_write("00008882-0000-1000-8000-00805f9b34fb", [0x24,0x4d,0x3c,5,20,0,1,2,3,4,xor([5,20,0,1,2,3,4])], wait_for_response=False)#JUST PASSTHRO
		return True
	except pygatt.exceptions.NotConnectedError:            
		print("write failed")
		return None
def msp_data(device,data):
    if not device:
        return None
    try:
        device.char_write("00008882-0000-1000-8000-00805f9b34fb", bytearray(data), wait_for_response=False)
        return True
    except pygatt.exceptions.NotConnectedError:            
        print("write failed")
        return None
try:
	
	adapter.start()
	joyth = Thread( target=joythread, args=("Thread-1", ) )
	joyth.start()
	device = connect()
	while not device:
	    device = connect()
 	     
	#device.subscribe("00008881-0000-1000-8000-00805f9b34fb",callback=handle_data)
	timer = millis()
	while True:
		
 		if millis() -timer  > 25:
			ps3joy.show()
 			timer = millis()
			while not send_data():
				device = connect()
				if device:
					device.subscribe("00008881-0000-1000-8000-00805f9b34fb",callback=handle_data)
 			
		if withConfig:
			data = conn.recv(20) 
			if data:
			    while not msp_data(device,data):
			        device = connect()
			        if device:
			            device.subscribe("00008881-0000-1000-8000-00805f9b34fb",callback=handle_data)
     	    
    	
    	
        
#time.sleep()
        
except KeyboardInterrupt:
	print('kill signal')
  
finally:
	print('Adapter Stop')
	adapter.stop()
	print('wait for joythread to finish')
	stop_threads = True
	joyth.join()
	if withConfig:
		conn.close()