"""
bipedal robot walking simulation

by Einsbon (Sunbin Kim)
- GitHub: https://github.com/Einsbon
- Youtube:  https://www.youtube.com/channel/UCt7FZ-8uzV_jHJiKp3NlHvg
- Blog: https://blog.naver.com/einsbon
"""

import pybullet as p
import time
from time import sleep
import pybullet_data
import numpy as np
import math
import os

import motorController
import walkGenerator

# motor setting
motor_kp = 1.0
motor_kd = 0.5
motor_torque = 1.0
motor_max_velocity = 5.0

# physics parameter setting
fixedTimeStep = 1./2000
numSolverIterations = 100

physicsClient = p.connect(p.GUI)
p.setTimeStep(fixedTimeStep)
p.setPhysicsEngineParameter(numSolverIterations=numSolverIterations)
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # to load plane.urdf

p.setGravity(0, 0, 0)

planeId = p.loadURDF("plane.urdf")
humanoid = p.loadURDF("humanoid_leg_12dof.4.urdf", [0.5, 0, 0.05],
                      p.getQuaternionFromEuler([0, 0, 1.5707963]), useFixedBase=False)

nJoints = p.getNumJoints(humanoid)
print(nJoints)

jointNameToId = {}
for i in range(nJoints):
    jointInfo = p.getJointInfo(humanoid, i)
    jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]

motorsController = motorController.MotorController(
    humanoid, motor_kp, motor_kd, motor_torque, motor_max_velocity, fixedTimeStep)
motorsController.printMotorsAngle()


walk = walkGenerator.WalkGenerator()
walk.setWalkParameter(bodyMovePoint=8, legMovePoint=8, h=50, l=90, sit=40, swayBody=45, swayFoot=0,
                      bodyPositionXPlus=5, swayShift=3, weightStart=0.4, weightEnd=0.7, swayPlus=0, stepTime=0.06, damping=0.0, incline=0.0)
walk.generate()
walk.showGaitPoint3D()
walk.inverseKinematicsAll()

actionTime = walk._stepTime
p.setGravity(0, 0, -9.8)
p.setRealTimeSimulation(0)
motorsController.setMotorsAngleInFixedTimestep(walk._walkPointStartRightInverse[0], 2, 0)

# rest 1 second in engine
waitTime = 1
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()
    # time.sleep(fixedTimeStep)

p.setGravity(0, 0, -9.8)


# walk 8 steps
# start walking. right foot step
for i in range(np.size(walk._walkPointStartRightInverse, 0)):
    motorsController.setMotorsAngleInFixedTimestep(walk._walkPointStartRightInverse[i], actionTime, 0)
for i in range(3):  # repeat twice
    # left foot step
    for i in range(np.size(walk._walkPointLeftStepInverse, 0)):
        motorsController.setMotorsAngleInFixedTimestep(walk._walkPointLeftStepInverse[i], actionTime, 0)
    # right foot step
    for i in range(np.size(walk._walkPointRightStepInverse, 0)):
        motorsController.setMotorsAngleInFixedTimestep(walk._walkPointRightStepInverse[i], actionTime, 0)
# end walking. left
for i in range(np.size(walk._walkPointEndLeftInverse, 0)):
    motorsController.setMotorsAngleInFixedTimestep(walk._walkPointEndLeftInverse[i], actionTime, 0)


# rest 2 seconds in engine
waitTime = 2
repeatTime = int(waitTime/fixedTimeStep)
for _ in range(repeatTime):
    p.stepSimulation()

# More applied version. Press Enter to start or stop walking.
walking = False
rightStep = True
while(1):
    keys = p.getKeyboardEvents()
    for k, v in keys.items():
        if (k == 65309) and (v == 3 or v == 6):  # if enter key is pressed
            walking = True
            keys = {}

    if walking == True:
        for i in range(np.size(walk._walkPointStartRightInverse, 0)):
            motorsController.setMotorsAngleInFixedTimestep(walk._walkPointStartRightInverse[i], actionTime, 0)
        rightStep = False

        keys = p.getKeyboardEvents()
        for k, v in keys.items():
            if (k == 65309) and (v == 3 or v == 6):  # if enter key is pressed
                walking = False
                keys = {}

        while(walking):
            if(rightStep):
                for i in range(np.size(walk._walkPointRightStepInverse, 0)):
                    motorsController.setMotorsAngleInFixedTimestep(walk._walkPointRightStepInverse[i], actionTime, 0)
                rightStep = False
            else:
                for i in range(np.size(walk._walkPointLeftStepInverse, 0)):
                    motorsController.setMotorsAngleInFixedTimestep(walk._walkPointLeftStepInverse[i], actionTime, 0)
                rightStep = True

            keys = p.getKeyboardEvents()
            for k, v in keys.items():
                if (k == 65309) and (v == 3 or v == 6):  # if enter key is pressed
                    walking = False
                    keys = {}

        if rightStep == True:
            for i in range(np.size(walk._walkPointEndRightInverse, 0)):
                motorsController.setMotorsAngleInFixedTimestep(walk._walkPointEndRightInverse[i], actionTime, 0)
        else:
            for i in range(np.size(walk._walkPointEndLeftInverse, 0)):
                motorsController.setMotorsAngleInFixedTimestep(walk._walkPointEndLeftInverse[i], actionTime, 0)

    else:
        p.stepSimulation()
