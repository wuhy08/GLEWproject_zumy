#!/usr/bin/python

from zumy import Zumy
from threading import Condition, Timer
import socket,time

def send_spd(zumy, speed, l_list, r_list):
	zumy.cmd(speed, speed)
	enc = zumy.read_enc()
	l_list.append(enc[0])
	r_list.append(enc[1])

if __name__ == '__main__':
	l_enc_list = []
	r_enc_list = []
	test_time = 1.0
	sampling_rate = 1000
	step = test_time/sampling_rate
	freq = 1
	magnitude = 0.2
	zumy_test = Zumy()
	for ii in range(sampling_rate):
		curr_spd = math.sin(2*math.pi*ii*step*freq)*magnitude
		give_cmd = Timer(step, send_spd ,[zumy_test, curr_spd, l_enc_list, r_enc_list])
		give_cmd.start()
	f = open('/home/glew/coop_slam_workspace/src/ros_zumy/src/test.txt', 'w')
	f.write('left: ' + repr(l_enc_list) + '\n')
	f.write('right: ' + repr(r_enc_list) + '\n')
	f.close()



