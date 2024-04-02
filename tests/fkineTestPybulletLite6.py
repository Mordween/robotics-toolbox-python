import pybullet as p
import time
import pybullet_data
import numpy as np 


from spatialmath import SE3
import roboticstoolbox as rtb
lite = rtb.models.Lite6()


class RobotControl():
    def __init__(self, dt):
        self.dt = dt
        #p.getCameraImage(500,500)


    def move(self, robotId, positionX, positionY, positionZ, numberOfSteps):

        robotJointsStates = np.zeros(6) 
        for joint in range(5):
            robotJointsStates[joint] = p.getJointState(robotId, joint)[0]

        Tep = SE3.Trans(positionX, positionY, positionZ) * SE3.OA([1, 0,1], [0, 0, -1])
        sol = lite.ik_LM(Tep)         # solve IK

        qt = rtb.jtraj(robotJointsStates, sol[0], numberOfSteps)
        print(qt.q[numberOfSteps-1])
        for steps in range(numberOfSteps):
            p.setJointMotorControlArray(robotId, [0, 1, 2, 3, 4, 5], controlMode = p.POSITION_CONTROL, targetPositions = qt.q[steps])
            #p.setJointMotorControlArray(robotId, [0, 1, 2, 3, 4, 5], controlMode = p.POSITION_CONTROL, targetPositions = qt.q[numberOfSteps-1])
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


if __name__ =="__main__":
    physicsClient = p.connect(p.GUI)#or p.DIRECT for non-graphical version
    p.resetSimulation()
    p.setAdditionalSearchPath(pybullet_data.getDataPath()) #optionally
    p.setGravity(0,0,-10)

    p.setRealTimeSimulation(0)

    planeId = p.loadURDF("plane.urdf")
    startPos = [0,0,0]
    robotPos = [0.5,1,0.63]
    duckPos = [0.5,0.2,0.68]
    startOrientation = p.getQuaternionFromEuler([0,0,0])
    robotId = p.loadURDF("../rtb-data/rtbdata/xacro/ufactory_description/lite6/urdf/lite6.urdf",robotPos, startOrientation, useFixedBase = True)
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

    position = np.zeros(p.getNumJoints(robotId))
    motors = range(p.getNumJoints(robotId))

    """positionX = [0.5, 0.2, 0.2]
    positionY = [0.2, 0.2, -0.2]
    positionZ = [0.5, 0.5, 0.5]"""

    """positionX = [0.1, 0.1, 0.1, 0.3, 0.5]
    positionY = [0.1, 0.1, 0.3, 0.5, 0.5]
    positionZ = [0.1, 0.4, 0.4, 0.4, 0.4]"""

    positionX = [0.1, 0.1, 0.1, 0.1, 0.1]
    positionY = [0.1, 0.1, 0.1, 0.1, 0.1]
    positionZ = [0.1, 0.25, 0.40, 0.55, 0.65]
    step = [500, 500, 500, 500, 500]

    control = RobotControl(0.01)


    control.pause(2)
    #control.grasp(True)
    """control.move(robotId, positionX[0], positionY[0], positionZ[0], step[0])
    control.pause(1)
    #control.grasp(False)
    control.pause(1)
    control.move(robotId, positionX[1], positionY[1], positionZ[1], step[1])
    control.pause(1)
    control.move(robotId, positionX[2], positionY[2], positionZ[2], step[2])"""

    for position in range(len(step)):
        control.move(robotId, positionX[position], positionY[position], positionZ[position], step[position])
        control.pause(1)
    control.pause(100) 


    """ 
    movement1 =  {2.82663761, -1.57756928 , 3.61519653 , 1.45406641  ,0.67045485 , 0.5528252 }
    movement2 = {-2.35625948 , 0.40434333 , 2.19309064 , 3.14158387 ,-1.78874597 ,-0.78547947}
    movement3 = {-4.67935945 , 1.4393472  , 2.44381663 ,-4.07265153 ,-2.04874782 ,-5.76008189}
    """
    #[ 3.28751894  2.42690161  1.69672506 -1.22674449  0.26309968  0.68421579]
    #[-2.3562099   0.41290105  2.20864892 -3.14158952 -1.79574041 -0.78539891]
    #[ 5.52479612  1.5093511   3.7609645   2.85063293 -0.83976376  1.83021769]


    """p.setJointMotorControlArray(robotId, [0, 1, 2, 3, 4, 5], controlMode = p.POSITION_CONTROL, targetPositions = movement1)

    control.pause(5)
    p.setJointMotorControlArray(robotId, [0, 1, 2, 3, 4, 5], controlMode = p.POSITION_CONTROL, targetPositions = movement2)

    control.pause(5)
    p.setJointMotorControlArray(robotId, [0, 1, 2, 3, 4, 5], controlMode = p.POSITION_CONTROL, targetPositions = movement3)

    control.pause(5)
"""
    p.disconnect()
