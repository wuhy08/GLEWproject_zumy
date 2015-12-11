# -*- coding: utf-8 -*-
"""
Created on Wed Oct 29 06:39:30 2014

@author: ajc
"""

from mbedrpc import *
import threading
import time
from serial import SerialException
import numpy as np
import math
from PID_control import PID

class Motor:
    def __init__(self, a1, a2):
        self.a1=a1
        self.a2=a2

    def cmd(self, speed):
        if speed >=0:
            self.a1.write(speed)
            self.a2.write(0)
        else:
            self.a1.write(0)
            self.a2.write(-speed)

imu_names = ['accel_x','accel_y','accel_z','gyro_x','gyro_y','gyro_z']
enc_names = ['l_enc','r_enc']

def average(list_of_list):
  num = len(list_of_list)
  avg_list = [0]*len(list_of_list[0])
  for curr_list in range(num):
    avg_list = [avg_list[ii]+list_of_list[curr_list][ii] for ii in range(len(list_of_list[0])) ]
  avg_list = [float(a)/num for a in avg_list]
  return avg_list



class Zumy:
    def __init__(self, dev='/dev/ttyACM0'):
        self.mbed=SerialRPC(dev, 115200)
        a1=PwmOut(self.mbed, p21)
        a2=PwmOut(self.mbed, p22)
        b1=PwmOut(self.mbed, p23)
        b2=PwmOut(self.mbed, p24)
        self.m_left = Motor(a1, a2)
        self.m_right = Motor(b1, b2)
        self.an = AnalogIn(self.mbed, p20)
        self.imu_vars = [RPCVariable(self.mbed,name) for name in imu_names]
        self.enc_vars = [RPCVariable(self.mbed,name) for name in enc_names]
        self.rlock=threading.Lock()
        self.time = time.time()
        self.duration = 0
        self.enc = self.read_enc()
        self.denc = [0,0]
        self.cal_data_l_up = [0.0]*41
        self.cal_data_l_dn = [0.0]*41
        self.cal_data_r_up = [0.0]*41
        self.cal_data_r_dn = [0.0]*41
        self.v_desired_l = 0.0
        self.v_desired_r = 0.0
        self.vl_dir = 'u'
        self.vr_dir = 'u'
        self.v_desired_l_limit = [0.0,0.0]
        self.v_desired_r_limit = [0.0,0.0]
        self.shutdown = False
        self.feedforward_flag = False
        self.PID_l = PID(0.0002,0.00004,0.00014,0,0,100000,-100000)
        self.PID_r = PID(0.0002,0.00007,0.00025,0,0,100000,-100000)
        self.alpha = 0.35
        self.PWM_l = 0
        self.PWM_r = 0

    def cmd(self, left, right): #will take ~8ms
        self.rlock.acquire()
	# As of Rev. F, positive command is sent to both left and right
        try:
          self.m_left.cmd(left)
          self.m_right.cmd(right)
        except SerialException:
          pass
        self.rlock.release()

    def read_voltage(self):
        self.rlock.acquire()
        try:
          ain=self.an.read()*3.3
        except SerialException:
          pass
        self.rlock.release()
        volt=ain*(4.99+15.8) / 4.99
        return volt

    def read_enc(self): #will take ~4ms
      self.rlock.acquire()
      try:
        rval = [int(self.enc_vars[0].read()), -int(self.enc_vars[1].read())]
      except SerialException:
        pass
      self.rlock.release()
      return rval

    def read_imu(self):
      self.rlock.acquire()
      try:
        rval = [float(var.read()) for var in self.imu_vars]
      except SerialException:
        pass
      self.rlock.release()
      return rval

    def update_time(self):
      self.duration = time.time() - self.time
      self.time = time.time()

    def update_enc_denc(self): # will take ~4ms
      old_denc = self.denc[:]
      self.denc = [(self.read_enc()[ii] - self.enc[ii]) for ii in range(2)]
      self.enc = [(self.denc[ii]+self.enc[ii]) for ii in range(2)]
      self.denc = [(self.alpha*self.denc[ii]+(1-self.alpha)*old_denc[ii]) for ii in range(2)]

    def update_desired(self, vl, vr):
      self.PID_l.setPoint(vl)
      self.PID_r.setPoint(vr)
      # if vl>self.v_desired_l:
      #   self.vl_dir = 'u'
      # else:
      #   self.vl_dir = 'd'
      # if vr>self.v_desired_r:
      #   self.vr_dir = 'u'
      # else:
      #   self.vr_dir = 'd'
      # if abs(vl-self.v_desired_l)>0.1*(self.v_desired_l_limit[1]-self.v_desired_l_limit[0]):
      #   self.v_desired_l = vl
      #   self.feedforward_flag = True
      # else:
      #   self.v_desired_l = vl
      # if abs(vr-self.v_desired_r)>0.1*(self.v_desired_r_limit[1]-self.v_desired_r_limit[0]):
      #   self.v_desired_r = vr
      #   self.feedforward_flag = True
      # else:
      #   self.v_desired_r = vr


    def calibration(self):
      calibration_input = range(0,20)+range(20,-1,-1)
      #calibration_input_neg = [-b for b in calibration_input_pos]
      calibration_repeat = 5
      calibration_step = 0.01
      cal_data_l = [None]*calibration_repeat
      cal_data_r = [None]*calibration_repeat
      self.update_enc_denc()
      self.update_time()
      for curr_cal_num in range(calibration_repeat):
        cal_data_l[curr_cal_num] = []
        cal_data_r[curr_cal_num] = []
        for curr_cal_bias in calibration_input:
          cal_speed = curr_cal_bias*calibration_step
          self.cmd(cal_speed, -cal_speed)
          time.sleep(0.032)
          self.update_enc_denc()
          self.update_time()
          cal_data_l[curr_cal_num].append(float(self.denc[0])/self.duration)
          cal_data_r[curr_cal_num].append(float(self.denc[1])/self.duration)
        for curr_cal_bias in calibration_input:
          cal_speed = curr_cal_bias*calibration_step
          self.cmd(-cal_speed, cal_speed)
          time.sleep(0.032)
          self.update_enc_denc()
          self.update_time()
          cal_data_l[curr_cal_num].append(float(self.denc[0])/self.duration)
          cal_data_r[curr_cal_num].append(float(self.denc[0])/self.duration)
      cal_data_l_avg = average(cal_data_l)
      cal_data_r_avg = average(cal_data_r)
      self.cal_data_l_up = cal_data_l_avg[-21:]+cal_data_l_avg[1:21]
      self.cal_data_l_dn = cal_data_l_avg[-21:-41:-1]+cal_data_l_avg[40:19:-1]
      self.cal_data_r_up = cal_data_r_avg[-21:]+cal_data_r_avg[1:21]
      self.cal_data_r_dn = cal_data_r_avg[-21:-41:-1]+cal_data_r_avg[40:19:-1]
      self.v_desired_l_limit = [min(cal_data_l_avg),max(cal_data_l_avg)]
      self.v_desired_r_limit = [min(cal_data_r_avg),max(cal_data_r_avg)]


    def feedforward(self, desired, wheel, direction):
      if wheel == 'l' and direction == 'u':
        curve = self.cal_data_l_up
      elif wheel == 'l' and direction == 'd':
        curve = self.cal_data_l_dn
      elif wheel == 'r' and direction == 'u':
        curve = self.cal_data_r_up
      elif wheel == 'r' and direction == 'd':
        curve = self.cal_data_r_dn
      x = np.linspace(-0.2, 0.2, 41)
      return np.interp(desired, curve, x)

    def feedback(self, desired, feedback, curr_input, wheel, direction):
      if wheel == 'l' and direction == 'u':
        curve = self.cal_data_l_up
      elif wheel == 'l' and direction == 'd':
        curve = self.cal_data_l_dn
      elif wheel == 'r' and direction == 'u':
        curve = self.cal_data_r_up
      elif wheel == 'r' and direction == 'd':
        curve = self.cal_data_r_dn
      err = desired - feedback
      curr_input_pos = math.floor((curr_input+0.2)/0.01)
      slope = (curve[curr_input_pos+1] - curve[curr_input_pos])/0.01
      new_input = curr_input + err/slope
      return new_input

      # def run(self):
      #   #self.calibration()
      #   self.update_enc_denc()
      #   self.update_time()
      #   while not self.shutdown:
      #     pid_l = self.PID_l.update(self.denc)
      #     pid_r = self.PID_r.update(self.r_enc_count_diff)

      #     #if self.feedforward_flag:

      #   self.cmd(0,0)




if __name__ == '__main__':
    z=Zumy()
    #z.cmd(0.3,0.3)
    #time.sleep(0.3)
    z.cmd(0,0) 
