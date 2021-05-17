'''In this exercise you need to implement an angle interploation function which makes NAO executes keyframe motion

* Tasks:
    1. complete the code in `AngleInterpolationAgent.angle_interpolation`,
       you are free to use splines interploation or Bezier interploation,
       but the keyframes provided are for Bezier curves, you can simply ignore some data for splines interploation,
       please refer data format below for details.
    2. try different keyframes from `keyframes` folder

* Keyframe data format:
    keyframe := (names, times, keys)
    names := [str, ...]  # list of joint names
    times := [[float, float, ...], [float, float, ...], ...]
    # times is a matrix of floats: Each line corresponding to a joint, and column element to a key.
    keys := [[float, [int, float, float], [int, float, float]], ...]
    # keys is a list of angles in radians or an array of arrays each containing [float angle, Handle1, Handle2],
    # where Handle is [int InterpolationType, float dTime, float dAngle] describing the handle offsets relative
    # to the angle and time of the point. The first Bezier param describes the handle that controls the curve
    # preceding the point, the second describes the curve following the point.
'''


from pid import PIDAgent
from keyframes import *
import numpy as np
from matplotlib import pyplot as plt
from scipy import interpolate as ip



class AngleInterpolationAgent(PIDAgent):
    def __init__(self, simspark_ip='localhost',
                 simspark_port=3100,
                 teamname='DAInamite',
                 player_id=0,
                 sync_mode=True):
        super(AngleInterpolationAgent, self).__init__(simspark_ip, simspark_port, teamname, player_id, sync_mode)
        self.keyframes = ([], [], [])

    def think(self, perception):
        target_joints = self.angle_interpolation(self.keyframes, perception)
        self.target_joints.update(target_joints)
        return super(AngleInterpolationAgent, self).think(perception)

    def angle_interpolation(self, keyframes, perception):
        target_joints = {}
        # YOUR CODE HERE
        for j in range(len(keyframes[0])):
            joint = keyframes[0][j]
        
            keytimes = keyframes[1][j]
            angles = [data[0] for data in keyframes[2][j]]
            #vectangles = [np.array([keytimes[i], angles[i]]) for i in range(0, len(keytimes))]
            #pangles = [data[1][2] for data in keyframes[2][j]]
            time = perception.time

            perkeytimes = keytimes + [keytimes[-1] + 1.0]
            perangles = angles + [angles[0]]

            anim_len = max(5, perkeytimes[-1])
            animation_time = (time % anim_len)
            #spline = ip.CubicSpline(keytimes, angles, bc_type='natural')
            spline = ip.CubicSpline(perkeytimes, perangles, bc_type='periodic')
            #spline = ip.BSpline(np.array(angles), np.array([1, 1, 1, 1 ]), 3)
            target_joints[joint] = spline((animation_time + 0.01) % anim_len)



            #x = [(i/100)*keytimes[-1] for i in range(0, 100)]
            #y = [spline((i/100)*keytimes[-1]) for i in range(0, 100)]

            #plt.plot(x, y, '--')
            #plt.plot(keytimes, angles, 'o')
            #plt.show()
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()

    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
