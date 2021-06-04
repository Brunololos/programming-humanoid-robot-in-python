'''In this file you need to implement remote procedure call (RPC) client

* The agent_server.py has to be implemented first (at least one function is implemented and exported)
* Please implement functions in ClientAgent first, which should request remote call directly
* The PostHandler can be implement in the last step, it provides non-blocking functions, e.g. agent.post.execute_keyframes
 * Hints: [threading](https://docs.python.org/2/library/threading.html) may be needed for monitoring if the task is done
'''

import weakref
from time import sleep
from xmlrpc.client import ServerProxy
from joint_control.keyframes import *
import numpy as np
import threading

class PostHandler(object):
    '''the post hander wraps function to be excuted in parallel
    '''
    def __init__(self, obj):
        self.proxy = weakref.proxy(obj)
        self.server = ServerProxy('http://localhost:8000')

    def execute_keyframes(self, keyframes):
        '''non-blocking call of ClientAgent.execute_keyframes'''
        # YOUR CODE HERE
        keyt = threading.Thread(target=self.server.execute_keyframes, args=[keyframes.tolist()])
        keyt.start()

    def set_transform(self, effector_name, transform):
        '''non-blocking call of ClientAgent.set_transform'''
        # YOUR CODE HERE
        self.server.set_transforms(effector_name, transform.tolist())

class ClientAgent(object):
    '''ClientAgent request RPC service from remote server
    '''
    # YOUR CODE HERE
    def __init__(self):
        self.post = PostHandler(self)
    
    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.post.server.get_angle(joint_name)
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.post.server.set_angle(joint_name, angle)

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE
        return self.post.server.get_posture()

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.post.server.execute_keyframes(keyframes)
        times = keyframes[1]
        m = max([max(list) for list in times])
        sleep(m)
        self.keyframes = ([], [], [])

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.post.server.get_transform(name)

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.post.server.set_transform(effector_name, transform.tolist())

if __name__ == '__main__':
    agent = ClientAgent()
    # TEST CODE HERE
    '''print(agent.get_angle('HeadYaw'))
    agent.set_angle('HeadYaw', 1)
    print("SET HeadYaw Angle to 1")
    print(agent.get_angle('HeadYaw'))'''
    '''print(agent.get_posture())
    agent.execute_keyframes(hello())
    agent.execute_keyframes(rightBackToStand())'''
    #print(agent.get_transform('HeadYaw'))
    T = np.identity(4)
    T[-1, 1] = 0.05
    T[-1, 2] = 0.26
    #agent.post.set_transform('LLeg', T)
    agent.execute_keyframes(hello())



