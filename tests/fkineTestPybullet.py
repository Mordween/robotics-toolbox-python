import pybullet as p
import time
import pybullet_data
import numpy as np 

from spatialmath import SE3
import roboticstoolbox as rtb
panda = rtb.models.Panda()


class RobotControl():
    def __init__(self, dt):
        self.dt = dt
        p.getCameraImage(500,500)


    def move(self, robotId, positionX, positionY, positionZ, numberOfSteps):

        robotJointsStates = np.zeros(7) 
        for joint in range(6):
            robotJointsStates[joint] = p.getJointState(robotId, joint)[0]

        Tep = SE3.Trans(positionX, positionY, positionZ) * SE3.OA([1, 1, 1], [0, 0, -1])
        sol = panda.ik_LM(Tep)         # solve IK

        qt = rtb.jtraj(robotJointsStates, sol[0], numberOfSteps)

        for steps in range(numberOfSteps):
            p.setJointMotorControlArray(robotId, [0, 1, 2, 3, 4, 5, 6], controlMode = p.POSITION_CONTROL, targetPositions = qt.q[steps])
            p.stepSimulation()
            time.sleep(0.01)

    def grasp(self, open=True):
        if (open == True):
            
            for steps in range(100):
                p.setJointMotorControlArray(robotId, [9,10], controlMode = p.POSITION_CONTROL, targetPositions = [0.04*steps/100,0.04*steps/100])
                p.stepSimulation()
                time.sleep(0.01)
            

        elif (open == False) :
            for steps in range(100):
                p.setJointMotorControlArray(robotId, [9,10], controlMode = p.POSITION_CONTROL, targetPositions = [0.04-0.04*steps/100,0.04-0.04*steps/100])
                p.stepSimulation()
                time.sleep(0.01)
            
    
    def pause(self, second):
        for i in range(int(second/(self.dt))):
            p.stepSimulation()
            time.sleep(0.01)


physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
p.resetSimulation()
p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
p.setGravity(0,0,-10)

p.setRealTimeSimulation(0)

planeId = p.loadURDF("plane.urdf")
startPos = [0,0,0]
robotPos = [0,0,0.63]
duckPos = [0.5,0.2,0.68]
startOrientation = p.getQuaternionFromEuler([0,0,0])
robotId = p.loadURDF("franka_panda/panda.urdf",robotPos, startOrientation, useFixedBase = True)
tableId = p.loadURDF("../urdf-object/table.urdf",startPos, startOrientation, useFixedBase = True)
duckId = p.loadURDF("duck_vhacd.urdf",duckPos, startOrientation)


focus_position , _ = p.getBasePositionAndOrientation(robotId)
p.resetDebugVisualizerCamera(cameraDistance = 3, cameraYaw = 0, cameraPitch = -40, cameraTargetPosition = focus_position)

jlower = np.zeros(p.getNumJoints(robotId))
jupper = np.zeros(p.getNumJoints(robotId))
#print("\nnumber of joints : ", p.getNumJoints(robotId), "\n")
for i in range(p.getNumJoints(robotId)):
    jlower[i] = p.getJointInfo(robotId, i)[8]
    jupper[i] = p.getJointInfo(robotId, i)[9]

#set the center of mass frame (loadURDF sets base link frame) startPos/Ornp.resetBasePositionAndOrientation(boxId, startPos, startOrientation)

position = np.zeros(6)
motors = range(6)

positionX = [0.5, -0.2, 0.5]
positionY = [0.2, -0.2, 0.2]
positionZ = [0, 0.5, 0.2]
step = [1000, 500, 500]

control = RobotControl(0.01)

control.pause(2)
control.grasp(True)
control.move(robotId, positionX[0], positionY[0], positionZ[0], step[0])
control.pause(1)
control.grasp(False)
control.pause(1)
control.move(robotId, positionX[1], positionY[1], positionZ[1], step[1])
control.pause(1)
control.move(robotId, positionX[2], positionY[2], positionZ[2], step[2])
control.pause(10)


    

p.disconnect()
