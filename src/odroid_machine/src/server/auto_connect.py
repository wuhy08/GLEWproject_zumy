#! /usr/bin/python

import rospy
from zeroconf_msgs.msg import *
from zeroconf_msgs.srv import *
import nmap
import threading
from std_msgs.msg import String, Int8
import subprocess
import rosgraph.masterapi
import re

class connection():
  def __init__(self, name):
    self.name = name
    self.status = 1 # 0 for offline, 1 for online, 2 for alive
    self.process = subprocess.Popen(["roslaunch","odroid_machine","remote_zumy.launch","mname:=" + self.name])
    
    self.last_HB = None
    self.heartBeat = False
    self.death_time = 5
   
    rospy.Subscriber("/" + self.name + "/heartBeat", String, self.HBCB) 
    self.pub = rospy.Publisher('online_detector/HB_observer/' + self.name, Int8 ,queue_size=5)

    # helper variable
    self.beep = 1

  def HBCB(self, data):
    self.last_HB = rospy.get_time()

  def HB_observer(self):
    if self.status == 1:
      self.pub.publish(0)
    elif self.status == 2:
      self.pub.publish(self.beep)
      self.beep = self.beep*(-1)

  def check_HB(self):
    self.HB_observer()
    if self.last_HB != None:
      now = rospy.get_time() 
      gap = now - self.last_HB
      if gap > self.death_time:
          self.heartBeat = False
      else: 
          self.heartBeat = True   
 

  def new_process(self):
     if self.process.poll() == None:
         self.process.terminate() 
     self.process = subprocess.Popen(["roslaunch","odroid_machine","remote_zumy.launch","mname:=" + self.name])

  def keep_process(self):
     if not self.process.poll() == None:
         self.process = subprocess.Popen(["roslaunch","odroid_machine","remote_zumy.launch","mname:=" + self.name])

  def end_process(self):
     if self.process.poll() == None:
         self.process.terminate()

class autoConnect():
  def __init__(self):
    self.master = rosgraph.masterapi.Master('/rostopic')
    self.lock = threading.Condition()
    self.nm = nmap.PortScanner()
    ### it is useless currently
    # self.host = 'toughbook'
    self.family_name = 'odroid'
    
    self.connections = {}
    self.pdict = {}

    self.pub_online = rospy.Publisher('/online_detector/online_robots', String, queue_size=5)
    self.pub_alive = rospy.Publisher('/online_detector/alive_robots', String, queue_size=5)
    self.pub_lost = rospy.Publisher('/online_detector/lost_robots', String, queue_size=5)

  ### helper functions
  def validation(self, name):
      if name[:len(self.family_name)] == self.family_name:
          return True
      return False

  ### callbacks
  def newCB(self, data):
      name = data.name.split()
      name = name[0]
      self.lock.acquire()
      if (name not in self.connections) and self.validation(name):
          self.connections[name] = connection(name)
      self.lock.release()

  ### scan functions
  def online_scan(self, name):
      result = self.nm.scan( name +'.local', arguments='-sP')
      uphosts =  result['nmap']['scanstats']['uphosts']
      if not int(uphosts):
          self.connections[name].status = 0
          self.connections[name].end_process()
      else:
          if self.connections[name].heartBeat:
              self.connections[name].keep_process()
              self.connections[name].status = 2   

  def lost_scan(self, name): 
      result = self.nm.scan( name +'.local', arguments='-sP')
      uphosts =  result['nmap']['scanstats']['uphosts']
      if int(uphosts):
          self.connections[name].status = 1
          self.connections[name].new_process()

  def alive_scan(self, name): 
      if not self.connections[name].heartBeat:
          self.connections[name].status = 1
          self.connections[name].end_process()        

  ### avahi listener adding function
  def call_service(self):
      rospy.wait_for_service('/zeroconf/add_listener')
      try:
          add_listener = rospy.ServiceProxy('/zeroconf/add_listener', AddListener)
          r = add_listener('_workstation._tcp')
      except rospy.ServiceException, e:
          print "Servce call failed: %s"%e

  ### main function
  def run(self):

      rospy.init_node("onlineDetector", anonymous=True)
      rospy.Subscriber("/zeroconf/new_connections", DiscoveredService, self.newCB)
      self.call_service()
      rate = rospy.Rate(1)

      rospy.loginfo(" Online Detector: waiting for connection...")
      while len(self.connections) == 0:
          rate.sleep()

      while not rospy.is_shutdown():
          rospy.loginfo(" Online Detector: scanning...")

          self.lock.acquire()
          online_robots = []
          alive_robots = []
          lost_robots = []
          for n in self.connections:
              c = self.connections[n]
              c.check_HB()
              if c.status == 1:
                  self.online_scan(c.name)
                  online_robots.append(c.name)
              elif c.status == 2:
                  self.alive_scan(c.name)
                  alive_robots.append(c.name)
              elif c.status == 0:
                  self.lost_scan(c.name)
                  lost_robots.append(c.name)
          self.pub_online.publish(','.join(online_robots))
          self.pub_alive.publish(','.join(alive_robots))
          self.pub_lost.publish(','.join(lost_robots))
          self.lock.release()

          rate.sleep()


if __name__ == "__main__":
    rospy.loginfo(" Online Detector: initializing...")
    ac = autoConnect()
    ac.run()

