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

        self.states = {  'begin': ((), [('time', 1, 'bonjour')] ),
                            'bonjour': ( ('QT/happy', 'QT/hi', 'Bonjour!'), [('key', 'c', 'presentation')] ),
                            'presentation': ( ('', '', '~Bienvenue dans la classe français langue étrangère, aujourd’hui vous allez raconter une belle histoire que vous avez vécu en France.'), [('key', 'c', 'histoire')] ), 
                            'histoire': ( ('QT/confused', 'QT/angry', 'Hein? Je ne comprends pas.'), [('key', 'c', 'colere')] ),
                            'colere': ( ('QT/angry', 'QT/happy', 'Holàlà mais ça ne va pas du tout ce n''est pas une belle histoire !'), [('key', 'c', 'reconciliation')] ),
                            'reconciliation': ( ('', '', '~Bon ça va. C’est acceptable. Assieds-toi. As-tu une jolie histoire Zihan ?'), [('key', 'c', 'validation_01')] ),
                            'validation_01': ( ('', 'QT/imitation/nodding_yes', '~Je suis d’accord'), [('time', 2, 'validation_02')] ),
                            'validation_02': ( ('QT/showing_smile', '', ''), [('time', 6, 'validation_03')] ),
                            'validation_03': ( ('', 'QT/happy', '~Belle perception de la vie !'), [('key', 'c', 'affirmation')] ),
                            'affirmation': ( ('', '', '~Elle a raison.'),  [('key', 'c', 'bizarre')] ),
                            'bizarre': ( ('', 'QT/emotions/afraid', '~Vous êtes bizarres.'),  [('key', 'c', 'incomprhension_A')] ),
                            'incomprhension_A': ( ('', 'QT/happy', '~Je ne comprends pas ce que vous dites.'),  [('key', 'c', 'parle_fr_A')] ),
                            'parle_fr_A': ( ('', '', '~Parlez en français.'),  [('key', 'c', 'incomprhension_B')] ),
                            'incomprhension_B': ( ('', '', '~Je ne comprends pas ce que vous dites.'),  [('key', 'c', 'parle_fr_B')] ),
                            'parle_fr_B': ( ('', 'QT/angry', '~Parlez en français.'),  [('key', 'c', 'incomprhension_C')] ),
                            'incomprhension_C': ( ('QT/sad', 'QT/sad', 'Je ne comprends pas ce que vous dites.'),  [('key', 'c', 'end')] ),
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
    rospy.init_node('drama_group_colere')
    drama = DramaWaking()
    drama.execute()  

