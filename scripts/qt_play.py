#! /usr/bin/env python3
# coding: utf-8

import roslib
roslib.load_manifest('theatre')
import rospy
import rospkg
import actionlib

from std_msgs.msg import String
from std_msgs.msg import Float64MultiArray

from geometry_msgs.msg import Twist
from theatre.msg import QTBehaviorAction
from theatre.msg import QTBehaviorGoal

import sys, select, termios, tty
import numpy as np
import pandas as pd

#nao
#from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

stdin_settings = termios.tcgetattr(sys.stdin)

def getKey(key_timeout):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], key_timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, stdin_settings)
    return key


class QTPlay:
    def __init__(self):
        self.rate = rospy.Rate(10) # 10hz

        self.keyboard_pub = rospy.Publisher('/keyboard', String, queue_size=10)
        self.state_pub = rospy.Publisher('/robot_state', String, queue_size=10)


        # state: ( {g: gesture, s: say, h: [head], la: [left_arm], ra: [right_arm], w: [x, y, z, ex, ey, ez]}, [(trigger, param, next_state)])
        self.states = { 'begin': ( {}, [('time', 1, 'choice')]),

                        'choice': ( {'g': '', 's': '' },
                                        [   ('key', 'z', 'look_up'), ('key', 's', 'look_center'), ('key', 'x', 'look_down'),('key', 'e', 'look_up_right'),
                                            ('key', 'q', 'look_left'), ('key', 'd', 'look_right'),('key', 'a', 'look_up_left'),
                                            ('key', 'w', 'look_down_left'),('key', 'c', 'look_down_right'),

                                            ('key', 'h', 'hello'),('key', 'k', 'dontknow'),('key', 'y', 'oui'),('key', 'n', 'non'),('key', 'f', 'suivi'),('key', 'g', 'public'),
                                            ('key', 'r', 'objetDroite'),('key', 'l', 'objetGauche'),('key', 'p', 'pense'),('key', 'o', 'pense2'),('key', 'm', 'neutral'),

                                            ('key', 'j', 'end')]),

                        'look_up': ( {'s': '', 'h': [0.0,-20.0]}, [('time', 0.1, 'choice')]),
                        'look_center': ( {'s': '', 'h': [0.0,0.0]}, [('time', 0.1, 'choice')]),
                        'look_down': ( {'s': '', 'h': [0.0,+10.0]}, [('time', 0.1, 'choice')]),
                        'look_right': ( {'s': '', 'h': [-20.0,0.0]}, [('time', 0.1, 'choice')]),
                        'look_left': ( {'s': '', 'h': [+20.0,0.0]}, [('time', 0.1, 'choice')]),
                        'look_up_right': ( {'s': '', 'h': [-20.0,-20.0]}, [('time', 0.1, 'choice')]),
                        'look_up_left': ( {'s': '', 'h': [+20.0,-20.0]}, [('time', 0.1, 'choice')]),
                        'look_down_right': ( {'s': '', 'h': [-10.0,+10.0]}, [('time', 0.1, 'choice')]),
                        'look_down_left': ( {'s': '', 'h': [+10.0,+10.0]}, [('time', 0.1, 'choice')]),


                        'hello': ( {'e': 'QT/happy', 'g': 'QT/hi', 's': '\\pau=2000\\Salut!', 'h': [0,0]}, [('time', 1, 'choice')]),
                        'dontknow': ( {'e': '', 'g': 'QT/touch-head', 's': '~\\pau=500\\Je ne sais pas'}, [('time', 1, 'choice')]),
                        'oui': ( {'e': '', 'g': 'QT/imitation/nodding-yes', 's': '~\\pau=500\\Oui!'}, [('time', 1, 'choice')]),
                        'non': ( {'e': '', 'g': 'QT/emotions/sad', 's': '~\\pau=500\\Non!'}, [('time', 1, 'choice')]),
                        'suivi': ( {'e': 'QT/surprise', 'g': 'QT/imitation/head-right-left', 's': '\\pau=500\\'}, [('time', 1, 'choice')]),
                        'public': ( {'e': 'QT/surprise', 'h': [0.0,0.0], 's': '\\pau=500\\'}, [('time', 1, 'choice')]),
                        'objetDroite': ( {'e': 'QT/happy', 'h': [+10.0,0.0], 's': '\\pau=1000\\Ici!'}, [('time', 1, 'choice')]),
                        'objetGauche': ( {'e': 'QT/happy', 'h': [-10.0,0.0], 's': '\\pau=1000\\Ici!'}, [('time', 1, 'choice')]),
                        'pense': ( {'e': 'QT/confused', 'g': 'QT/imitation/hands-on-hip', 's': '\\pau=500\\'}, [('time', 1, 'choice')]),
                        'pense2': ( {'e': 'QT/afraid', 'g': 'QT/touch-head-back', 's': '\\pau=500\\'}, [('time', 1, 'choice')]),
                        'neutral': ( {'e': 'QT/neutral', 'g': 'QT/neutral', 's': '\\pau=500\\'}, [('time', 1, 'choice')]),


                        'end': ((), [('time', 0.1, 'end')]) }

        # print(self.states)

        self.state = 'begin'

        self.head_pub = rospy.Publisher('/qt_robot/head_position/command', Float64MultiArray, queue_size=1)
        self.left_arm_pub = rospy.Publisher('/qt_robot/left_arm_position/command', Float64MultiArray, queue_size=1)
        self.right_arm_pub = rospy.Publisher('/qt_robot/right_arm_position/command', Float64MultiArray, queue_size=1)

        rospy.Timer(rospy.Duration( self.states[self.state][1][0][1]), self.time_callback, oneshot=True)
        print(self.state)


    def time_callback(self, event):
        # go to next state
        print('time callback')
        triggers = self.states[self.state][1]
        for trigger in triggers:
            if trigger[0] == 'time':
                self.state = trigger[2]
                print('next_state: ' + self.state)
                self.next_state()
#        self.state = self.states[self.state][2]
#        pass

    def keyboard_callback(self, key):
        print('keyboard callback: ' + key)
        self.keyboard_pub.publish(key)
        triggers = self.states[self.state][1]
        for trigger in triggers:
            if trigger[0] == 'key':
                if trigger[1] == key:
                    self.state = trigger[2]
                    print('next_state: ' + self.state)
                    self.next_state()
                    break;
#        pass

    def next_state(self):
        if(self.state != 'end'):
            self.state_pub.publish(self.state)
            # send bhw
            bhw = self.states[self.state][0]
            if(len(bhw)):
                # AL machine => pass to smach
                print(self.state + ' =bhw=> ' + str(bhw))

                if 'h' in bhw:
                    self.head_pub.publish(Float64MultiArray(data=bhw['h']))
                if 'la' in bhw:
                    self.left_arm_pub.publish(Float64MultiArray(data=bhw['la']))
                if 'ra' in bhw:
                    self.right_arm_pub.publish(Float64MultiArray(data=bhw['ra']))
                
                if ('e' in bhw) or ('g' in bhw) or ('s' in bhw):
                    client = actionlib.SimpleActionClient('/qt_behave', QTBehaviorAction)
                    client.wait_for_server()

                    goal = QTBehaviorGoal( emotion=bhw['e'] if 'e' in bhw else '',
                                           gesture=bhw['g'] if 'g' in bhw else '',
                                           speech=bhw['s'] if 's' in bhw else '')
                    client.send_goal(goal)
                    # ...
                    client.wait_for_result()
                    result = client.get_result()


            # prepare jump for next state
            triggers = self.states[self.state][1]

            print(self.state + ' =trig=> ' + str(triggers))
            for trigger in triggers:
                if trigger[0] == 'time':
                    rospy.Timer(rospy.Duration(trigger[1]), self.time_callback, oneshot=True)


    def execute(self):
        while not rospy.is_shutdown():
            if self.state == 'end': break
            key = getKey(0.1)
            if (len(key)):
                self.keyboard_callback(key)
            self.rate.sleep()





if __name__ == '__main__':
    rospy.init_node('qt_play')

    qt_play = QTPlay()
    qt_play.execute()


