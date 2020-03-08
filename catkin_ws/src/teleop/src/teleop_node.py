#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios

TARGET_LIN_VEL = 0.1 # m/s
TARGET_ANG_VEL = 0.03 # m/s 

msg = """
Control Ozzie!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : move forward/backward
a/d : turn left/right
s   : stop


CTRL-C to quit
"""

e = """
Communications Failed
"""

def getKey():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "new:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)


if __name__=="__main__":
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rospy.init_node('teleop_node')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    prev_key = ''
    target_linear_vel = 0.0
    target_angular_vel = 0.0

    try:
        print(msg)
        while(1):
            key = getKey()
            if (key == '\x03'): # Ctrl-C
                break
            elif key == 'w':
                target_linear_vel = TARGET_LIN_VEL
                target_angular_vel = 0.0
            elif key in ['s', ' '] :
                target_linear_vel = 0.0
                target_angular_vel = 0.0
            elif key == 'x':
                target_linear_vel = - TARGET_LIN_VEL
                target_angular_vel = 0.0
            elif key == 'a':
                target_linear_vel = 0.0
                target_angular_vel = TARGET_ANG_VEL
            elif key == 'd':
                target_linear_vel = 0.0
                target_angular_vel = - TARGET_ANG_VEL
                
            if key != prev_key and key != '':
                print(vels(target_linear_vel, target_angular_vel))
            prev_key = key

            # if status == 20 :
            #     print(msg)
            #     status = 0

            twist = Twist()

            twist.linear.x = target_linear_vel; twist.linear.y = 0.0; twist.linear.z = 0.0

            twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = target_angular_vel

            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = Twist()
        twist.linear.x = 0.0; twist.linear.y = 0.0; twist.linear.z = 0.0
        twist.angular.x = 0.0; twist.angular.y = 0.0; twist.angular.z = 0.0
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
