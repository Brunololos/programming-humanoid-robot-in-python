'''In this exercise you need to implement forward kinematics for NAO robot

* Tasks:
    1. complete the kinematics chain definition (self.chains in class ForwardKinematicsAgent)
       The documentation from Aldebaran is here:
       http://doc.aldebaran.com/2-1/family/robots/bodyparts.html#effector-chain
    2. implement the calculation of local transformation for one joint in function
       ForwardKinematicsAgent.local_trans. The necessary documentation are:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    3. complete function ForwardKinematicsAgent.forward_kinematics, save the transforms of all body parts in torso
       coordinate into self.transforms of class ForwardKinematicsAgent

* Hints:
    the local_trans has to consider different joint axes and link parameters for different joints
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'joint_control'))

from numpy.matlib import matrix, identity
import numpy as np

from joint_control.angle_interpolation import AngleInterpolationAgent


class ForwardKinematicsAgent(AngleInterpolationAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(ForwardKinematicsAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.transforms = {n: identity(4) for n in self.joint_names}

        # chains defines the name of chain and joints of the chain
        self.chains = {
            'Head': ['HeadYaw', 'HeadPitch'],
            'LArm': ['LShoulderPitch', 'LShoulderRoll', 'LElbowYaw', 'LElbowRoll'],#, 'LWristYaw'],
            'LLeg': ['LHipYawPitch', 'LHipRoll', 'LHipPitch', 'LKneePitch', 'LAnklePitch', 'LAnkleRoll'], #TODO - documentation unclear on L/RAnkleRoll
            'RLeg': ['RHipYawPitch', 'RHipRoll', 'RHipPitch', 'RKneePitch', 'RAnklePitch', 'RAnkleRoll'],
            'RArm': ['RShoulderPitch', 'RShoulderRoll', 'RElbowYaw', 'RElbowRoll']#, 'RWristYaw']
        }

    def think(self, perception):
        self.forward_kinematics(perception.joint)
        return super(ForwardKinematicsAgent, self).think(perception)

    def local_trans(self, joint_name, joint_angle):
        '''calculate local transformation of one joint

        :param str joint_name: the name of joint
        :param float joint_angle: the angle of joint in radians
        :return: transformation
        :rtype: 4x4 matrix
        '''
        T = identity(4)

        # YOUR CODE HERE
        sine = np.sin(joint_angle)
        cosine = np.cos(joint_angle)

        '''if("YawPitch" in joint_name):
            return
        el'''
        if("Roll" in joint_name):
            T[1,1] = cosine
            T[1,2] = -sine
            T[2,1] = sine
            T[2,2] = cosine

        elif("Pitch" in joint_name):
            T[0,0] = cosine
            T[0,2] = -sine
            T[2,0] = sine
            T[2,2] = cosine

        elif("Yaw" in joint_name):
            T[0,0] = cosine
            T[0,1] = -sine
            T[1,0] = sine
            T[1,1] = cosine

        X = 0
        Y = 0
        Z = 0
        #head
        if("HeadYaw" in joint_name):
            Z = 126.5
        #arms
        elif("ShoulderPitch" in joint_name):
            Y = 98
            Z = 100
        elif("ShoulderRoll" in joint_name):
            X = 105
            Y = 15
        elif("WristYaw" in joint_name):
            X = 55.95
        #legs
        elif("HipYaw" in joint_name):
            Y = 50
            Z = -85
        elif("KneePitch" in joint_name):
            Z = -100
        elif ("AnklePitch" in joint_name):
            Z = -102.9

        if("R" in joint_name):
            Y = -Y

        T[3,0] = X
        T[3,1] = Y
        T[3,2] = Z

        return T

    def forward_kinematics(self, joints):
        '''forward kinematics

        :param joints: {joint_name: joint_angle}
        '''
        for chain_joints in self.chains.values():
            T = identity(4)
            for joint in chain_joints:
                angle = joints[joint]
                Tl = self.local_trans(joint, angle)
                # YOUR CODE HERE
                T = np.dot(T, Tl)

                self.transforms[joint] = T

if __name__ == '__main__':
    agent = ForwardKinematicsAgent()
    agent.run()
