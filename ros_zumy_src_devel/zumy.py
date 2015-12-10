# -*- coding: utf-8 -*-
"""
Created on Wed Oct 29 06:39:30 2014

@author: ajc
"""

from mbedrpc import *
import threading
import time
from serial import SerialException

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

class Zumy:
    def __init__(self, dev='/dev/ttyACM0'):
        self.mbed=SerialRPC(dev, 115200)
        a1=PwmOut(self.mbed, p21)
        a2=PwmOut(self.mbed, p22)
        b1=PwmOut(self.mbed, p23)
        b2=PwmOut(self.mbed, p24)
        self.m_right = Motor(a1, a2)
        self.m_left = Motor(b1, b2)
        self.an = AnalogIn(self.mbed, p20)
        self.imu_vars = [RPCVariable(self.mbed,name) for name in imu_names]
        self.enc_vars = [RPCVariable(self.mbed,name) for name in enc_names]
        self.rlock=threading.Lock()

    def cmd(self, left, right):
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

    def read_enc(self):
      self.rlock.acquire()
      try:
        rval = [int(var.read()) for var in self.enc_vars]
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

if __name__ == '__main__':
    z=Zumy()
    z.cmd(0.3,0.3)
    time.sleep(0.3)
    z.cmd(0,0) 
