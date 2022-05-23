#! /usr/bin/env python
# coding: utf-8

import roslib
roslib.load_manifest('theatre')
import rospy
import actionlib

from std_msgs.msg import String
import theatre.msg 

import sys, select, termios, tty

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


class DramaWaking:
    def __init__(self):
        self.rate = rospy.Rate(10) # 10hz

        # state: ( (emotion, gesture, say), [(trigger, param, next_state)])

        self.states = { 'begin': ((), [('time', 1, 'sleeping')]),
                        'sleeping': (('QT/yawn', 'QT/yawn', '\\pau=1000\\#BREATH01#'),
                                        [('time', 5, 'waking')] ),
                        'waking': (('','','~Quelle belle journée! Je me suis vraiment bien réposé!'),
                                        [('key', 'c', 'wtf')] ), #('time', 5, 'wtf')] ),

                        'wtf' : (('', 'QT/imitation/head-right-left', ''), [('time', 4, 'asking')]),
                        'asking' : (('QT/afraid', '', '\pau=500\Mais qui êtes vous ?'), [('time', 4, 'asking_again')]),
                        'asking_again': (('', '', '~\pau=1000\Pourquoi vous me regardez tous ?'), [('time', 7, 'waiting')]),
                        'waiting' : (('QT/confused', '', ''), [('time', 4, 'waiting_again')]),
                        'waiting_again' : (('QT/surprise', '', ''), [('key', 'a', 'answerA'), ('key', 'b', 'answerB')]), # [('time', 5, 'answerB')]), # <--

                        'answerA' : (('QT/happy', 'QT/happy', '\pau=1000\Je suis content de faire votre connaissance, nous pourrions devenir amis!'), [('time', 4, 'end')]),
                        'answerB' : (('QT/cry', 'QT/emotions/disgusted', '\pau=1000\J\'ai peur. Je me sens observé'), [('time', 4, 'end')]),
                        'end': ((), [('time', 0.1, 'end')]) }

        self.state = 'begin'
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
            # send bhw
            bhw = self.states[self.state][0]
            if(len(bhw)):
                # AL machine => pass to smach
                print(self.state + ' =bhw=> ' + str(bhw))
                client = actionlib.SimpleActionClient('/qt_behave', theatre.msg.QTBehaviorAction)
                client.wait_for_server()

                goal = theatre.msg.QTBehaviorGoal( emotion=bhw[0],
                                                    gesture=bhw[1],
                                                    speech=bhw[2])
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
    rospy.init_node('drama_waking')
    drama = DramaWaking()
    drama.execute()  

