#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import sys, select, os
import tty, termios

import os, struct, array
from fcntl import ioctl

# We'll store the states here.
axis_states = {}
button_states = {}

# These constants were borrowed from linux/input.h
axis_names = {
    0x00 : 'x',
    0x01 : 'y',
    0x02 : 'z',
    0x03 : 'rx',
    0x04 : 'ry',
    0x05 : 'rz',
    0x06 : 'trottle',
    0x07 : 'rudder',
    0x08 : 'wheel',
    0x09 : 'gas',
    0x0a : 'brake',
    0x10 : 'hat0x',
    0x11 : 'hat0y',
    0x12 : 'hat1x',
    0x13 : 'hat1y',
    0x14 : 'hat2x',
    0x15 : 'hat2y',
    0x16 : 'hat3x',
    0x17 : 'hat3y',
    0x18 : 'pressure',
    0x19 : 'distance',
    0x1a : 'tilt_x',
    0x1b : 'tilt_y',
    0x1c : 'tool_width',
    0x20 : 'volume',
    0x28 : 'misc',
}

button_names = {
    0x120 : 'trigger',
    0x121 : 'thumb',
    0x122 : 'thumb2',
    0x123 : 'top',
    0x124 : 'top2',
    0x125 : 'pinkie',
    0x126 : 'base',
    0x127 : 'base2',
    0x128 : 'base3',
    0x129 : 'base4',
    0x12a : 'base5',
    0x12b : 'base6',
    0x12f : 'dead',
    0x130 : 'a',
    0x131 : 'b',
    0x132 : 'c',
    0x133 : 'x',
    0x134 : 'y',
    0x135 : 'z',
    0x136 : 'tl',
    0x137 : 'tr',
    0x138 : 'tl2',
    0x139 : 'tr2',
    0x13a : 'select',
    0x13b : 'start',
    0x13c : 'mode',
    0x13d : 'thumbl',
    0x13e : 'thumbr',

    0x220 : 'dpad_up',
    0x221 : 'dpad_down',
    0x222 : 'dpad_left',
    0x223 : 'dpad_right',

    # XBox 360 controller uses these codes.
    0x2c0 : 'dpad_left',
    0x2c1 : 'dpad_right',
    0x2c2 : 'dpad_up',
    0x2c3 : 'dpad_down',
}



axis_map = []
button_map = []

# Open the joystick device.
fn = '/dev/input/js3'
jsdev = open(fn, 'rb')

# Get the device name.
#buf = bytearray(63)
buf = array.array('B', [0] * 64)
ioctl(jsdev, 0x80006a13 + (0x10000 * len(buf)), buf) # JSIOCGNAME(len)
js_name = buf.tostring().rstrip(b'\x00').decode('utf-8')

# Get number of axes and buttons.
buf = array.array('B', [0])
ioctl(jsdev, 0x80016a11, buf) # JSIOCGAXES
num_axes = buf[0]

buf = array.array('B', [0])
ioctl(jsdev, 0x80016a12, buf) # JSIOCGBUTTONS
num_buttons = buf[0]

# Get the axis map.
buf = array.array('B', [0] * 0x40)
ioctl(jsdev, 0x80406a32, buf) # JSIOCGAXMAP

for axis in buf[:num_axes]:
    axis_name = axis_names.get(axis, 'unknown(0x%02x)' % axis)
    axis_map.append(axis_name)
    axis_states[axis_name] = 0.0

# Get the button map.
buf = array.array('H', [0] * 200)
ioctl(jsdev, 0x80406a34, buf) # JSIOCGBTNMAP

for btn in buf[:num_buttons]:
    btn_name = button_names.get(btn, 'unknown(0x%03x)' % btn)
    button_map.append(btn_name)
    button_states[btn_name] = 0

TARGET_LIN_VEL = 0.5 # m/s
TARGET_ANG_VEL = 0.5 # m/s 

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
            evbuf = jsdev.read(8)
            if evbuf:
                time, value, type, number = struct.unpack('IhBB', evbuf)

                # If button
                if type & 0x01:
                    button = button_map[number]
                    if button:
                        button_states[button] = value
                        if value:
                            print(("%s pressed" % (button)))
                        else:
                            print(("%s released" % (button)))

                # If axis
                if type & 0x02:
                    axis = axis_map[number]
                    if axis:
                        fvalue = value / 32767.0
                        axis_states[axis] = fvalue
                if abs(fvalue) > 0.1:
                        # Left stick up/down
                        if axis == "rx":
                            target_linear_vel = TARGET_LIN_VEL * fvalue
                        # Right stick left/right
                        elif axis == "ry"
                            target_angular_vel = TARGET_ANG_VEL * fvalue * -1
                else:
                        # Left stick "stop"
                        if axis == "rx":
                            target_linear_vel = 0
                        elif axis == "ry":
                            target_angular_vel = 0
         
            print(vels(target_linear_vel, target_angular_vel))

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
