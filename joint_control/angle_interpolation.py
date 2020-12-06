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
from keyframes import hello


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
        (names, times, keys) = keyframes

        if(self.started == -1):
            self.started = perception.time
            
        time = perception.time - self.started

        for j, joint_name in enumerate(names):
            if joint_name not in self.joint_names:
                continue
            
            joint_time = times[j]
            joint_key = keys[j]

            for t in range(len(joint_time)-1):
                if t == 0 and time < joint_time[0]:
                    t0 = 0
                    t3 = joint_time[0]
                    p0 = 0
                    p1 = 0
                    p2 = joint_key[0][1][2]
                    p3 = joint_key[0][0]
                elif joint_time[t] < time < joint_time[t+1]:
                    t0 = joint_time[t]
                    t3 = joint_time[t+1]
                    p0 = joint_key[t][0]
                    p3 = joint_key[t+1][0]
                    p1 = p0 + joint_key[t][2][2]
                    p2 = p3 + joint_key[t+1][1][2]
                else:
                    target_joints[joint_name] = perception.joint[joint_name]
                    continue
                
                i = (time - t0)/(t3 -t0)

                target_joints[joint_name] = ((1-i)**3)*p0 + 3*((1-i)**2)*i*p1 + 3*((1-i)**3)*(i**2)*p2 +(i**3)*p3
    
        return target_joints

if __name__ == '__main__':
    agent = AngleInterpolationAgent()
    agent.keyframes = hello()  # CHANGE DIFFERENT KEYFRAMES
    agent.run()
