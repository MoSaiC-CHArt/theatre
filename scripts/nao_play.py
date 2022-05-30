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
from theatre.msg import NaoBehaviorAction
from theatre.msg import NaoBehaviorGoal

import sys, select, termios, tty
import numpy as np
import pandas as pd

#nao
from naoqi_bridge_msgs.msg import JointAnglesWithSpeed

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


class NaoPlay:
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

                                            ('key', 'Z', 'LSU'),('key', 'X', 'LSD'),

                                            ('key', 'h', 'hello'),('key', 'k', 'dontknow'),('key', 'y', 'oui'),('key', 'n', 'non'),('key', 'f', 'suivi'),('key', 'g', 'public'),
                                            ('key', 'r', 'objetDroite'),('key', 'l', 'objetGauche'),('key', 'p', 'pense'),('key', 'o', 'pense2'),('key', 'm', 'neutral'),
                                            ('key', 'R', 'really'),('key', 'H', 'comment'),('key','J','jaime'),('key','b','bored'),('key','Y','happy'),('key', 'S', 'sad'),
                                            ('key', 'u', 'standup'),('key', 'K', 'kisses'),('key', 'E', 'excited'),('key', 't', 'thinking'),('key', 'C', 'curious'),
                                            ('key', 'F', 'fear'),('key', 'O', 'confused'),

                                            ('key', '8', 'walk_fwd'), ('key', '5', 'stop'), ('key', '2', 'walk_back'),
                                            ('key', '4', 'strife_left'), ('key', '6', 'strife_right'),
                                            ('key', '7', 'rotate_left'), ('key', '9', 'rotate_right'),

                                            ('key', 'j', 'end')]),

                        'look_up': ( {'s': '', 'h': [0.0,-1]}, [('time', 0.1, 'choice')]),
                        'look_center': ( {'s': '', 'h': [0.0,+0.0]}, [('time', 0.1, 'choice')]),
                        'look_down': ( {'s': '', 'h': [0.0,+1]}, [('time', 0.1, 'choice')]),
                        'look_right': ( {'s': '', 'h': [-2,+0.0]}, [('time', 0.1, 'choice')]),
                        'look_left': ( {'s': '', 'h': [+2,+0.0]}, [('time', 0.1, 'choice')]),
                        'look_up_right': ( {'s': '', 'h': [-0.5,-5.0]}, [('time', 0.1, 'choice')]),
                        'look_up_left': ( {'s': '', 'h': [+0.5,-0.5]}, [('time', 0.1, 'choice')]),
                        'look_down_right': ( {'s': '', 'h': [-0.5,+0.5]}, [('time', 0.1, 'choice')]),
                        'look_down_left': ( {'s': '', 'h': [+0.5,+0.5]}, [('time', 0.1, 'choice')]),

                        'LSD': ( {'s': '', 'la': [0.5,0,0,0,0,1]}, [('time', 1, 'see')]),
                        'LSU': ( {'s': '', 'la': [-0.5,0,0,0,0,1]}, [('time', 1, 'see')]),
                        'see': ( {'s': 'Did you see ?'}, [('time', 0.1, 'choice')]),



                        'hello': ( {'g': 'hello', 's': '\\pau=4000\\Hello!', 'h': [0,0]}, [('time', 3, 'choice')]),
                        'dontknow': ( {'g': 'dontknow', 's': '\\pau=1000\\I dont know'}, [('time', 3, 'choice')]),
                        'oui': ( {'g': 'oui', 's': '\\pau=50\\yes!'}, [('time', 3, 'choice')]),
                        'non': ( {'g': 'non', 's': '\\pau=5\\no!'}, [('time', 3, 'choice')]),
                        'suivi': ( {'g': 'suivi', 's': '\\pau=2000\\'}, [('time', 3, 'choice')]),
                        'public': ( {'g': 'public', 's': '\\pau=2000\\'}, [('time', 3, 'choice')]),
                        'objetDroite': ( {'g': 'objetDroite', 's': '\\pau=9500\\Here !'}, [('time', 3, 'choice')]),
                        'objetGauche': ( {'g': 'objetGauche', 's': '\\pau=9500\\Here !'}, [('time', 3, 'choice')]),
                        'pense': ( {'g': 'pense', 's': '\\pau=2000\\'}, [('time', 3, 'choice')]),
                        'pense2': ( {'g': 'pense2', 's': '\\pau=2000\\'}, [('time', 3, 'choice')]),
                        'neutral': ( {'g': 'neutral', 's': '\\pau=2000\\'}, [('time', 3, 'choice')]),
                        'really': ( {'g': 'really', 's': '\\pau=2000\\really ?'}, [('time', 3, 'choice')]),
                        'comment': ( {'g': 'comment', 's': '\\pau=2000\\how ?'}, [('time', 3, 'choice')]),
                        'jaime': ( {'g':'jaime', 's': '\\pau=1500\\ I love it !'}, [('time', 3, 'choice')]),
                        'bored': ( {'g':'bored', 's': '\\pau=1500\\ Huff'}, [('time', 3, 'choice')]),
                        'happy': ( {'g':'happy', 's': '\\pau=1500\\ YES !'}, [('time', 3, 'choice')]),
                        'sad': ( {'g':'sad', 's': '\\pau=1500\\ oh'}, [('time', 3, 'choice')]),
                        'standup': ( {'g':'standup'}, [('time', 3, 'choice')]),

                        'kisses': ( {'g':'kiss'}, [('time', 3, 'choice')]),
                        'excited': ( {'g':'excited','s': 'yes !'}, [('time', 3, 'choice')]),
                        'thinking': ( {'g':'thinking'}, [('time', 3, 'choice')]),
                        'curious': ( {'g':'curious','s': '\\pau=1500\\ oh'}, [('time', 3, 'choice')]),
                        'fear': ( {'g':'fear'}, [('time', 3, 'choice')]),
                        'confused': ( {'s':'okey','g': 'confused'}, [('time', 3, 'choice')]),


                        'walk_fwd': ( {'w': [1,0,0,0,0,0]}, [('time', 0.1, 'choice')]),
                        'stop': ( {'w': [0,0,0,0,0,0]}, [('time', 0.1, 'choice')]),
                        'walk_back': ( {'w': [-1,0,0,0,0,0]}, [('time', 0.1, 'choice')]),
                        'strife_left': ( {'w': [0,1,0,0,0,0]}, [('time', 0.1, 'choice')]),
                        'strife_right': ( {'w': [0,-1,0,0,0,0]}, [('time', 0.1, 'choice')]),
                        'rotate_left': ( {'w': [0,0,0,0,0,1]}, [('time', 0.1, 'choice')]),
                        'rotate_right': ( {'w': [0,0,0,0,0,-1]}, [('time', 0.1, 'choice')]),



                        'end': ((), [('time', 0.1, 'end')]) }

        # print(self.states)

        self.state = 'begin'

        self.angles_pub = rospy.Publisher('/joint_angles', JointAnglesWithSpeed, queue_size=10)
        self.walk_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)


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
                    self.angles_pub.publish(JointAnglesWithSpeed(joint_names=['HeadYaw', 'HeadPitch'], joint_angles=bhw['h'], speed=0.25))
                if 'la' in bhw:
                    self.angles_pub.publish(JointAnglesWithSpeed(joint_names=['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll', 'LWristYaw', 'LHand'], joint_angles=bhw['la'], speed=0.25))
                if 'ra' in bhw:
                    self.angles_pub.publish(JointAnglesWithSpeed(joint_names=['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll', 'RWristYaw', 'RHand'], joint_angles=bhw['ra'], speed=0.25))
                if ('g' in bhw) or ('s' in bhw):
                    client = actionlib.SimpleActionClient('/nao_behave', NaoBehaviorAction)
                    client.wait_for_server()

                    goal = NaoBehaviorGoal( gesture=bhw['g'] if 'g' in bhw else '',
                                            speech=bhw['s'] if 's' in bhw else '')
                    client.send_goal(goal)
                    # ...
                    client.wait_for_result()
                    result = client.get_result()
                if 'w' in bhw:
                    cmd_vel = Twist()
                    cmd_vel.linear.x=bhw['w'][0]
                    cmd_vel.linear.y=bhw['w'][1]
                    cmd_vel.linear.z=bhw['w'][2]
                    cmd_vel.angular.x=bhw['w'][3]
                    cmd_vel.angular.y=bhw['w'][4]
                    cmd_vel.angular.z=bhw['w'][5]
                    self.walk_pub.publish(cmd_vel) 


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
    rospy.init_node('nao_play')

    nao_play = NaoPlay()
    nao_play.execute()


