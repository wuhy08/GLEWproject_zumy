#!/usr/bin/python

import rospy

from geometry_msgs.msg import Twist
from threading import Condition
from zumy import Zumy
from std_msgs.msg import String,Header
from std_msgs.msg import Int32, Float32, Duration
from sensor_msgs.msg import Imu
import socket,time
from PID_control import PID

def confined(number, cap):
  if number>cap:
    number = cap
  elif number<-cap:
    number = -cap
  return number

class ZumyROS:	
  def __init__(self):
    self.zumy = Zumy()
    rospy.init_node('zumy_ros')
    self.cmd = (0,0)
    rospy.Subscriber('cmd_vel', Twist, self.cmd_callback_test, queue_size=1)
    self.lock = Condition()
    self.rate = rospy.Rate(30)
    self.name = socket.gethostname()
    self.vl = 0
    self.vr = 0
    self.feedback = True
    self.alpha = 0.8
    self.heartBeat = rospy.Publisher('/' + self.name + '/heartBeat', String, queue_size=5)
    self.imu_pub = rospy.Publisher('/' + self.name + '/imu', Imu, queue_size = 1)
    self.r_enc_pub = rospy.Publisher('/' + self.name + '/r_enc', Int32, queue_size = 5)
    self.l_enc_pub = rospy.Publisher('/' + self.name + '/l_enc', Int32, queue_size = 5)
    self.l_spd_pub = rospy.Publisher('/'+ self.name + '/l_spd_pub', Float32, queue_size = 5)
    self.r_spd_pub = rospy.Publisher('/' + self.name + '/r_spd_pub', Float32, queue_size = 5)
    self.l_denc_pub = rospy.Publisher('/' + self.name + '/l_denc_pub', Float32, queue_size = 5)
    self.r_denc_pub = rospy.Publisher('/' + self.name + '/r_denc_pub', Float32, queue_size = 5)
    self.duration_pub = rospy.Publisher('/'+ self.name + '/Duration', Float32, queue_size = 5)
    self.imu_count = 0
    #self.timestamp = rospy.Time.now()
    #self.prevtimestamp = rospy.Time.now()
    #self.duration = (self.timestamp - self.prevtimestamp).to_sec()
    #self.pl = PID(0.0008,0.000055,0.00005)
    #self.pr = PID(0.0009,0.000065,0.00006)
    self.l_enc_count = 0
    self.l_enc_prev_count = 0
    self.l_enc_count_diff = 0
    self.prev_l_enc_count_diff = 0
    self.r_enc_count = 0
    self.r_ecn_prev_count = 0
    self.r_enc_count_diff = 0
    self.prev_r_enc_count_diff = 0
    self.l_vel_list = []
    self.r_vel_list = []

  def cmd_callback(self, msg):
    lv = 1
    la = 1
    v = msg.linear.x
    a = msg.angular.z
    self.vr = lv*v - la*a
    self.vl = lv*v + la*a
    self.zumy.PID_l.setPoint(self.vl)
    self.zumy.PID_r.setPoint(self.vr)

  def cmd_callback_test(self,msg):
    self.vl = msg.linear.x
    self.vr = msg.angular.z
    self.zumy.PID_l.setPoint(self.vl)
    self.zumy.PID_r.setPoint(self.vr)





  def run(self):
    while not rospy.is_shutdown():
      self.zumy.update_enc_denc()
      self.zumy.update_time()
      # self.prevtimestamp = self.timestamp
      # self.timestamp = rospy.Time.now()
      # self.duration = (self.timestamp - self.prevtimestamp).to_sec()
      # self.duration_pub.publish(self.duration)
      
   #   if time_now - self.timestamp > .5:
   #       self.cmd = (0,0)

      # self.lock.acquire()
      # imu_data = self.zumy.read_imu()
      # enc_data = self.zumy.read_enc()
      # self.lock.release()
      
      # imu_msg = Imu()
      # imu_msg.header = Header(self.imu_count,rospy.Time.now(),self.name)
      # imu_msg.linear_acceleration.x = 9.81 * imu_data[0]
      # imu_msg.linear_acceleration.y = 9.81 * imu_data[1]
      # imu_msg.linear_acceleration.z = 9.81 * imu_data[2]
      # imu_msg.angular_velocity.x = 3.14 / 180.0 * imu_data[3]
      # imu_msg.angular_velocity.y = 3.14 / 180.0 * imu_data[4]
      # imu_msg.angular_velocity.z = 3.14 / 180.0 * imu_data[5]
      # self.imu_pub.publish(imu_msg)
      
      # enc_msg = Int32()
      # enc_msg.data = enc_data[1]
      # self.r_enc_pub.publish(enc_msg)
      # enc_msg.data = enc_data[0]
      # self.l_enc_pub.publish(enc_msg)
      # self.l_enc_prev_count = self.l_enc_count
      # self.r_enc_prev_count = self.r_enc_count
      # self.l_enc_count = enc_data[0]
      # self.r_enc_count = enc_data[1]
      # self.prev_l_enc_count_diff = self.l_enc_count_diff
      # self.l_enc_count_diff = self.alpha*\
      #           (self.l_enc_count - self.l_enc_prev_count)/\
      #           self.duration + \
      #           self.alpha*\
      #           self.prev_l_enc_count_diff
      # self.prev_r_enc_count_diff = self.r_enc_count_diff
      # self.r_enc_count_diff = self.alpha*\
      #           (self.r_enc_count - self.r_enc_prev_count)/\
      #           self.duration + \
      #           self.alpha*\
      #           self.prev_r_enc_count_diff
      pid_l = self.zumy.PID_l.update(self.zumy.denc[0]/self.zumy.duration)
      pid_r = self.zumy.PID_r.update(self.zumy.denc[1]/self.zumy.duration)
      #pid_l = confined(pid_l, 0.2)
      #pid_r = confined(pid_r, 0.2)
      # pid_l_pub = Float32()
      # pid_r_pub = Float32()
      # l_set_point_pub = Float32()
      # r_set_point_pub = Float32()
      # l_set_point_pub.data = self.l_enc_count_diff
      # r_set_point_pub.data = self.r_enc_count_diff
      # pid_l_pub.data = pid_l
      # pid_r_pub.data = pid_r
      # self.l_spd_pub.publish(pid_l_pub)
      # self.r_spd_pub.publish(pid_r_pub)
      self.l_denc_pub.publish(self.zumy.denc[0]/self.zumy.duration)
      self.r_denc_pub.publish(self.zumy.denc[1]/self.zumy.duration)
      self.l_vel_list.append(self.zumy.denc[0]/self.zumy.duration)
      self.r_vel_list.append(self.zumy.denc[1]/self.zumy.duration)

      self.heartBeat.publish("I am alive from Glew!!")
      
      self.lock.acquire()
      if self.feedback:
        self.zumy.cmd(pid_l,pid_r)
      else:
        self.zumy.cmd(self.vl, self.vr)
      self.lock.release()

      self.rate.sleep()

    # If shutdown, turn off motors
    self.zumy.cmd(0,0)
    f = open('/home/glew/coop_slam_workspace/src/ros_zumy/src/test.txt', 'a')
    for ii in range(len(self.l_vel_list)):
      f.write(("%.2f\t" % self.l_vel_list[ii]))
      f.write(("%.2f\n" % self.r_vel_list[ii]))
    f.close()


if __name__ == '__main__':
  zr = ZumyROS()
  zr.run()
