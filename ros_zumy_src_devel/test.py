#!/usr/bin/python

from zumy import Zumy
from threading import Condition, Timer
import socket,time, math, sys

def send_spd(zumy, speed, l_list, r_list):
	zumy.cmd(speed, -speed)
	time.sleep(0.032)
        enc = zumy.read_enc()
	l_list.append(enc[0])
	r_list.append(enc[1])

if __name__ == '__main__':
	l_enc_list = []
	r_enc_list = []
        timestep= []
	test_time = 1.0
	sampling_rate = 20
	step = test_time/sampling_rate
	#magnitude = float(sys.argv[1])
	zumy_test = Zumy()
	t0 = time.time()
	for ii in range(sampling_rate)+range(sampling_rate,-1,-1):
		curr_spd = 0.01*ii
		send_spd(zumy_test, curr_spd, l_enc_list, r_enc_list)
		timestep.append(time.time() - t0)
		#time.sleep(step)
        #send_spd(zumy_test, 0, l_enc_list, r_enc_list)
        #timestep.append(time.time() - t0)
	f = open('/home/glew/coop_slam_workspace/src/ros_zumy/src/test.txt', 'a')
	f.write('left: ' + repr([b-l_enc_list[0] for b in l_enc_list]) + '\n')
	f.write('right: ' + repr([b-r_enc_list[0] for b in r_enc_list]) + '\n')
	f.write('time: '+ repr(timestep) +'\n')
	f.close()



