#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

from dynamic_reconfigure.server import Server
from robotic_arm.cfg import MotorHelperConfig

import math, serial, threading

JOINT_STATE_TOPIC = '/robotic_arm/joint_states'
PUB_TOPIC_LIST = [
    '/robotic_arm/0_shoulder_LR_position_controller/command',
    '/robotic_arm/1_shoulder_FB_position_controller/command',
    '/robotic_arm/2_elbow_FB_position_controller/command',
    '/robotic_arm/3_elbow_R_position_controller/command',
    '/robotic_arm/4_wrist_P_position_controller/command',
    '/robotic_arm/5_wrist_Y_position_controller/command',
    '/robotic_arm/6_hand_left_position_controller/command',
    '/robotic_arm/7_hand_right_position_controller/command',
]

MAP_LIST = [2, 0, 1, 4, 5, 3]
BIAS_LIST= [ 1.42, 3.00, -0.02 + math.pi/2, 2.84, 0.09, 2.99]
DIR_LIST= [ -1, -1, -1, -1, -1, -1]

class Filter:
    def __init__(self, n):
        self.buf = [0.0] * n
        self.i = 0
    
    def push(self, n_in):
        self.buf[self.i] = n_in
        self.i = (self.i + 1) % len(self.buf)
    
    def get(self):
        return sum(self.buf) / len(self.buf)

class Serial_Publisher:
    def __init__(self):
        self.motor_num = 6
        self.joystick_num = 3
        self.ch = len(PUB_TOPIC_LIST)
        # init variables
        self.motor_rad_l = []
        for _ in range(self.motor_num):
            self.motor_rad_l.append(Filter(5))
        self.joystick_l = [0.0] * self.joystick_num
        self.pub_l = [None] * self.ch
        self.sub_l = [None] * self.ch
        self.joint_efforts = [0.0] * self.ch
        self.joint_position = [0.0] * self.ch
        self.lost = -1
        # init node
        self.joint_state_sub = rospy.Subscriber(JOINT_STATE_TOPIC, JointState, callback=self._joint_callback)
        rospy.init_node('serial_publisher', anonymous=False)
        # serial setup
        # self.ser = serial.Serial('/dev/ttyUSB0', baudrate=115200)
        self.ser = serial.Serial('/dev/ttyACM0')
        self.recv_thd = threading.Thread(group=None, target=self._recv_fcn)
        self.recv_thd.start()        
        self.ratio_3510 = 0.0
        self.ratio_6020 = 0.0
        # publisher setup
        for i in range(self.ch):
            self.pub_l[i] = rospy.Publisher(PUB_TOPIC_LIST[i], Float64, queue_size=10)
        # dynamic reconfigure the motor_helper
        self.srv = Server(MotorHelperConfig, self._dynm_recfg_callback)
        self.rate = rospy.Rate(1000) # 1000hz
        print("Done Initialise")

    def _recv_fcn(self):
        while not rospy.is_shutdown():
            sof = None
            while sof != b'\xAA':
                sof = self.ser.read(1)
            bs = self.ser.read(2*self.motor_num + self.joystick_num + 1)
            if bs[-1] != sum(sof + bs[:-1]) & 0xFF:
                info = "Serial receive error..."
                rospy.loginfo(info)
            # handle motor data
            for i in range(self.motor_num):
                ecd = int.from_bytes(bs[2*i: 2*i+2], 'little', signed=False)
                # index for PUB_TOPIC_LIST
                index = MAP_LIST[i]
                rad = 2 * math.pi * ecd/8192
                rad -= BIAS_LIST[index]
                rad *= DIR_LIST[index]
                self.motor_rad_l[index].push(rad)
            # handle joystick data
            for i in range(self.joystick_num):
                self.joystick_l[i] = bs[2*self.motor_num + i]
            self.lost += 1
            # print("lost pack", self.lost)

    def _joint_callback(self, data):
        self.joint_efforts[:] = data.effort[:]
        self.joint_position[:] = data.position[:]
            
    def _dynm_recfg_callback(self, config, level):
        self.ratio_3510 = config.ratio_3510
        self.ratio_6020 = config.ratio_6020
        return config 

    def pub(self):
        while self.motor_rad_l[0].get() == 0.0:
            print('no msg')
        while not rospy.is_shutdown():
            bs = b'\xAA'
            for i in range(self.motor_num):
                # Publish encoder to ROS
                self.pub_l[i].publish(self.motor_rad_l[i].get())
                # Construct bytestream
                cmd = self.joint_efforts[MAP_LIST[i]]
                if MAP_LIST[i] < 3:
                    cmd = int(self.ratio_6020 * cmd)
                    if cmd >= 30000:
                        rospy.logwarn("6020 Motor cmd exceed 30000 on " + str(i) + " joint")
                        cmd = 30000
                    if cmd <= -30000:
                        rospy.logwarn("6020 Motor cmd exceed -30000 on " + str(i) + " joint")
                        cmd = -30000
                    bs = bytes(bs + cmd.to_bytes(2, 'big', signed=True))
                else:
                    cmd = int(self.ratio_3510 * cmd)
                    if cmd >= 29000:
                        rospy.logwarn("3510 Motor cmd exceed 29000 on " + str(i) + " joint")
                        cmd = 29000
                    if cmd <= -29000:
                        rospy.logwarn("3510 Motor cmd exceed -29000 on " + str(i) + " joint")
                        cmd = -29000
                    bs = bytes(bs + cmd.to_bytes(2, 'big', signed=True))
            bs += (sum(bs[:]) & 0xFF).to_bytes(1,'little')
            self.ser.write(bs)

            # for i in range(self.joystick_num):
            #     if self.joystick_l[2] == 1:
            #         self.pub_l[self.motor_num].publish(-0.1)
            #         self.pub_l[self.motor_num + 1].publish(0.1)
            #     else:
            #         self.pub_l[self.motor_num].publish(-0.0)
            #         self.pub_l[self.motor_num + 1].publish(0.0)

            self.lost = -1
            self.rate.sleep()
            
    def wait_for_sync(self):
        while True:
            sync = True
            for i in range(self.motor_num):
                diff = self.joint_position[i] - self.motor_rad_l[i].get()
                if diff > math.pi: diff -= 2 * math.pi
                elif diff < -math.pi: diff += 2 * math.pi 
                if abs(diff) > 0.5:
                    sync = False
                    print("Joint ", i, 'not synced! diff is', diff)
                    rospy.Rate(5).sleep()
                    break
            if sync:
                return
    
    def __del__(self):
        bs = b'\xAA'
        bs += bytes(2 * self.ch)
        bs += (sum(bs[:]) & 0xFF).to_bytes(1,'little')
        self.ser.write(bs)

if __name__ == '__main__':
    try:
        sp = Serial_Publisher()
        sp.wait_for_sync()
        sp.pub()
    except rospy.ROSInterruptException:
        pass