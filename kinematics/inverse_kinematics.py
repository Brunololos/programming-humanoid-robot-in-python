'''In this exercise you need to implement inverse kinematics for NAO's legs

* Tasks:
    1. solve inverse kinematics for NAO's legs by using analytical or numerical method.
       You may need documentation of NAO's leg:
       http://doc.aldebaran.com/2-1/family/nao_h21/joints_h21.html
       http://doc.aldebaran.com/2-1/family/nao_h21/links_h21.html
    2. use the results of inverse kinematics to control NAO's legs (in InverseKinematicsAgent.set_transforms)
       and test your inverse kinematics implementation.
'''
from numpy import random
from forward_kinematics import ForwardKinematicsAgent
from numpy.matlib import identity
import numpy as np
from math import atan2


class InverseKinematicsAgent(ForwardKinematicsAgent):
    def inverse_kinematics(self, effector_name, transform):
        '''solve the inverse kinematics

        :param str effector_name: name of end effector, e.g. LLeg, RLeg
        :param transform: 4x4 transform matrix
        :return: list of joint angles
        '''
        joint_angles = {}
        joints = self.chains[effector_name]

        N = len(joints) # - 1
        theta = random.random(N) * 1e-5
        lambda_ = 1
        max_step = 0.075

        target = np.matrix([self.from_trans(transform)])

        for i in range(1000):
            self.forward_kinematics(self.perception.joint)
            Ts = []
            for joint in joints:
                Ts.append(self.transforms[joint])

            Te = np.matrix([self.from_trans(Ts[-1])]).T
            e = target - Te
            e[e > max_step] = max_step
            e[e < -max_step] = -max_step
            T = np.matrix([self.from_trans(i) for i in Ts]).T # Ts[1:-1]
            J = Te - T
            dT = Te - T
            J[0, :] = -dT[1, :]
            J[1, :] = dT[0, :]
            J[-1, :] = 1
            d_theta = lambda_ * np.linalg.pinv(J) * e
            theta += np.asarray(d_theta.T)[0]

            i = 0
            for joint in joints:
                joint_angles[joint] = theta[i]
                i = i + 1
                if(i==len(theta)):
                    break

            if np.linalg.norm(d_theta) < 1e-4:
                break
        return joint_angles

    def set_transforms(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        joint_names = self.chains[effector_name]
        times = []
        keytimes = []
        self.forward_kinematics(self.perception.joint)
        joint_angles = self.inverse_kinematics(effector_name, transform)

        handle = [3, 0, 0]

        for joint in joint_names:
            keytimes.append([[self.perception.joint[joint], handle, handle], [joint_angles[joint], handle, handle]])

        for i in range(0, len(joint_names)):
            times.append([1.0, 3.0])

        self.keyframes = (joint_names, times, keytimes)

    def from_trans(self, matrix):
        x = matrix[3, 0]
        y = matrix[3, 1]
        z = matrix[3, 2]

        theta = 0
        if(matrix[1, 1] == matrix[2, 2]): #classification incorrect ? cos(0) = 1
            theta = atan2(matrix[2, 1], matrix[1, 1])
        elif(matrix[0, 0] == matrix[2, 2]):
            theta = atan2(matrix[0, 2], matrix[0, 0])
        elif(matrix[0, 0] == matrix[1, 1]):
            theta = atan2(matrix[0, 1], matrix[0, 0])

        return x, y, z, theta

if __name__ == '__main__':
    agent = InverseKinematicsAgent()
    # test inverse kinematics
    T = identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    agent.set_transforms('LLeg', T)
    agent.run()
