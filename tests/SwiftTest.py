import swift
import roboticstoolbox as rtb
import spatialmath as sm 
import numpy as np 
import time


class RobotControl():
    def __init__(self, dt, env, robot):
        self.dt = dt
        self.env = env
        self.robot = robot
        self.env.add(self.robot)


    def move(self, positionX, positionY, positionZ, numberOfSteps):

        Tep = sm.SE3.Trans(positionX, positionY, positionZ) * sm.SE3.OA([1, 0,1], [0, 0, -1])
        sol = self.robot.ik_LM(Tep)         # solve IK

        qt = rtb.jtraj(self.robot.q, sol[0], numberOfSteps)
        #print(qt.q[numberOfSteps-1])
        for steps in range(numberOfSteps):
            self.robot.q = qt.q[steps]
            self.env.step(self.dt)
            time.sleep(self.dt)
            
    
    def pause(self, second):
        for i in range(int(second/(self.dt))):
            self.env.step(self.dt)
            time.sleep(self.dt)


if __name__ == "__main__":  # pragma nocover

    env = swift.Swift()
    env.launch(realtime = True)

    robot = rtb.models.Lite6()

    rob = RobotControl(0.1, env, robot)

    positionX = [0.15, 0.15, 0.35]
    positionY = [-0.2, -0.2, 0.2]
    positionZ = [0.05, 0.4, 0.2]
    step = [50, 50, 50]

    for position in range(len(step)):
        rob.move(positionX[position], positionY[position], positionZ[position], step[position])
        rob.pause(1)
        
    rob.pause(10)



