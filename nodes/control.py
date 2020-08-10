#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64

import turtle, os

TOPIC_LIST = [
    '/robotic_arm/wheel_LF_velocity_controller/command',
    '/robotic_arm/wheel_LB_velocity_controller/command',
    '/robotic_arm/wheel_RF_velocity_controller/command',
    '/robotic_arm/wheel_RB_velocity_controller/command',
    '/robotic_arm/6_hand_left_position_controller/command',
    '/robotic_arm/7_hand_right_position_controller/command',
]

PUB_LIST = [rospy.Publisher(n, Float64, queue_size=1, tcp_nodelay=True) for n in TOPIC_DICT]
cmd_list = [0.0, 0.0, 0.0, 0.0]

def chassis_pub():
    for i in range(4):
        PUB_LIST[i].publish(cmd_list[i])
    

def press_gripper():
    PUB_LIST[4].publish(-0.1)
    PUB_LIST[5].publish(0.1)

def release_gripper():
    PUB_LIST[4].publish(0.0)
    PUB_LIST[5].publish(0.0)
    
def press_w():
    cmd[:] += 10.0

def release_w():
    cmd[:] -= 10.0

PRESS_FUNC_DICT = {
    ' ': press_gripper,
    'w': press_w
    # 'a': press_a,
    # 's': press_s,
    # 'd': press_d
}
    
RELEASE_FUNC_DICT = {
    ' ': release_gripper,
    'w': release_w
    # 'a': release_w,
    # 's': release_w,
    # 'd': release_w,
}

if __name__ == '__main__':
    try:
        rospy.init_node('keyboard_controller')
        os.system('xset r off')
        for k in TOPIC_DICT.keys():
            turtle.onkeypress(PRESS_FUNC_DICT[' '], ' ')
            turtle.onkeyrelease(RELEASE_FUNC_DICT[' '], ' ')
        turtle.listen()
        turtle.mainloop()
        os.system('xset r on')
    except rospy.ROSInterruptException:
        os.system('xset r on')