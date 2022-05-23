#! /usr/bin/env python

import roslib
roslib.load_manifest('theatre')
import rospy
import actionlib
import importlib
import rospy
import numpy as np
import pandas as pd

from std_msgs.msg import String
from theatre.msg import NaoBehaviorAction
#from naoqi_bridge_msgs.msg import JointAnglesWithSpeed
from naoqi_bridge_msgs.msg import JointAngleTrajectory
from sensor_msgs.msg import JointState

class NaoBehaveServer:
  def __init__(self):
    self.server = actionlib.SimpleActionServer('nao_behave', NaoBehaviorAction, self.execute, False)

    # [HeadYaw, HeadPitch, LShoulderPitch, LShoulderRoll, LElbowYaw, LElbowRoll, LWristYaw,
    #   LHand, LHipYawPitch, LHipRoll, LHipPitch, LKneePitch, LAnklePitch, LAnkleRoll, RHipYawPitch,
    #   RHipRoll, RHipPitch, RKneePitch, RAnklePitch, RAnkleRoll, RShoulderPitch, RShoulderRoll,
    #   RElbowYaw, RElbowRoll, RWristYaw, RHand]
    #self.sub_joints = rospy.Subscriber('/joint_states', JointState, self.joints_callback)
    #self.joint_states = None

    self.pub_gesture = rospy.Publisher('/joint_angles_trajectory', JointAngleTrajectory, queue_size=1)

    self.pub_speech = rospy.Publisher('/speech', String, queue_size=1)

    self.server.start()
    print('start')

  #def joints_callback(self, joint_states):
    #print(joint_states)
    #self.joint_states = pd.DataFrame(columns=joint_states.name)
    #self.joint_states.loc[0] = joint_states.position

  def execute(self, goal):
    print('exec') 

    print(goal.speech)
    self.pub_speech.publish(String(data=goal.speech))

    """
    print(goal.gesture)
    gesture = importlib.import_module('Nao.' + goal.gesture)
    #print(gesture.df)
    dt = np.diff(np.append([0.0], gesture.df.index.values) )
    for i, t in enumerate(gesture.df.index):
      print(t,i,dt[i])
      max_v = ((gesture.df.loc[t] - self.joint_states[gesture.df.columns])/dt[i] ).max(axis=1).max()
      max_v = 1 if max_v > 1 else max_v
      #max_v = np.ceil(max_v*10)/10.0
      print(max_v)
      self.pub_gesture.publish(JointAnglesWithSpeed(joint_names=gesture.df.columns, joint_angles=gesture.df.loc[t].values, speed=max_v))
      rospy.sleep(dt[i])
    """

    print(goal.gesture)
    if len(goal.gesture):
        gesture = importlib.import_module('Nao.' + goal.gesture)
        #print(gesture.df)

        self.pub_gesture.publish(JointAngleTrajectory(joint_names=gesture.names, joint_angles=np.ravel(gesture.keys), times=np.ravel(gesture.times)))


       #self.pub_emotion.publish(goal.emotion)
       #self.pub_gesture.publish(goal.gesture)

      #  if goal.speech.startswith('~'):
      #      self.pub_bhw_speech.publish(goal.speech[1:])
      #  else:
      #      self.pub_speech.publish(goal.speech)

    self.server.set_succeeded()



if __name__ == '__main__':
  rospy.init_node('nao_behave')
  server = NaoBehaveServer()
  rospy.spin()

