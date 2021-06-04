'''In this file you need to implement remote procedure call (RPC) server

* There are different RPC libraries for python, such as xmlrpclib, json-rpc. You are free to choose.
* The following functions have to be implemented and exported:
 * get_angle
 * set_angle
 * get_posture
 * execute_keyframes
 * get_transform
 * set_transform
* You can test RPC server with ipython before implementing agent_client.py
'''

# add PYTHONPATH
import os
import sys
sys.path.append(os.path.join(os.path.abspath(os.path.dirname(__file__)), '..', 'kinematics'))

from kinematics.inverse_kinematics import InverseKinematicsAgent
from xmlrpc.server import SimpleXMLRPCServer, SimpleXMLRPCRequestHandler
import threading
import pickle
from os import listdir
from time import sleep
import numpy as np

class RequestHandler(SimpleXMLRPCRequestHandler):
    rpc_paths = ('/RPC2',)

class ServerAgent(InverseKinematicsAgent):
    '''ServerAgent provides RPC service
    '''
    # YOUR CODE HERE
    def __init__(self):
        super(ServerAgent, self).__init__()
        self.posture_classifier = pickle.load(open('../joint_control/robot_pose.pkl', 'rb'))  # LOAD YOUR CLASSIFIER

    def get_angle(self, joint_name):
        '''get sensor value of given joint'''
        # YOUR CODE HERE
        return self.perception.joint[joint_name]
    
    def set_angle(self, joint_name, angle):
        '''set target angle of joint for PID controller
        '''
        # YOUR CODE HERE
        self.target_joints[joint_name] = angle

    def get_posture(self):
        '''return current posture of robot'''
        # YOUR CODE HERE

        posture = 'unknown'
        classes = listdir('../joint_control/robot_pose_data')
        posture_data = [self.perception.joint['LHipYawPitch'], self.perception.joint['LHipRoll'], self.perception.joint['LHipPitch'],
                        self.perception.joint['LKneePitch'], self.perception.joint['RHipYawPitch'],
                        self.perception.joint['RHipRoll'], self.perception.joint['RHipPitch'], self.perception.joint['RKneePitch'],
                        self.perception.imu[0], self.perception.imu[1]]
        posture = classes[self.posture_classifier.predict([posture_data])[0]]

        return posture

    def execute_keyframes(self, keyframes):
        '''excute keyframes, note this function is blocking call,
        e.g. return until keyframes are executed
        '''
        # YOUR CODE HERE
        self.keyframes = keyframes

    def get_transform(self, name):
        '''get transform with given name
        '''
        # YOUR CODE HERE
        return self.transforms[name].tolist()

    def set_transform(self, effector_name, transform):
        '''solve the inverse kinematics and control joints use the results
        '''
        # YOUR CODE HERE
        self.set_transforms(effector_name, np.array(transform))

if __name__ == '__main__':
    agent = ServerAgent()

    server = SimpleXMLRPCServer(('localhost', 8000), allow_none=True, requestHandler=RequestHandler)
    server.register_introspection_functions()
    server.register_instance(agent)
    t = threading.Thread(target = server.serve_forever)
    t.start()

    '''t1 = threading.Thread(target=agent.run)
    t1.start()'''
    agent.run()
