"""
This helps controll motors in pybullet physics simulator.

by Einsbon (Sunbin Kim)
- GitHub: https://github.com/Einsbon
- Youtube:  https://www.youtube.com/channel/UCt7FZ-8uzV_jHJiKp3NlHvg
- Blog: https://blog.naver.com/einsbon
"""


import numpy as np
import time
from time import sleep
import pybullet as p


class MotorController:
    def __init__(self, robot, kp, kd, torque, max_velocity, timeStep):
        self._robot = robot
        jointNameToId = {}
        joint_id_list = []
        joint_pos_list = []
        self._joint_number = 0
        for i in range(p.getNumJoints(robot)):
            jointInfo = p.getJointInfo(robot, i)
            if jointInfo[2] == 0:
                joint_id_list.append(jointInfo[0])
                joint_pos_list.append(p.getJointState(robot, jointInfo[0])[0])
                jointNameToId[jointInfo[1].decode('UTF-8')] = jointInfo[0]
                self._joint_number += 1
        self._joint_id = np.array(joint_id_list, dtype=np.int32)
        self._joint_targetPos = np.array(joint_pos_list, dtype=np.float)
        self._joint_currentPos = np.array(joint_pos_list, dtype=np.float)

        self._jointNameToId = jointNameToId
        self._kp = kp
        self._kd = kd
        self._torque = torque
        self._max_velocity = max_velocity
        self._timeStep = timeStep
        print(self._joint_id)
        print(self._joint_targetPos)
        print(self._jointNameToId)

    def printMotorsAngle(self):
        for i in range(self._joint_number):
            print(list(self._jointNameToId)[i])
            print(p.getJointState(self._robot, self._joint_id[i])[0])
            # self._currentPos[i] = p.getJointState(self._robot, self._joint_id[i])[0]

    def getMotorAngel(self):
        for i in range(self._joint_number):
            self._joint_currentPos[i] = p.getJointState(self._robot, self._joint_id[i])[0]

    def printMotorAngel(self):
        for i in range(self._joint_number):
            self._joint_currentPos[i] = p.getJointState(self._robot, self._joint_id[i])[0]
        print(self._joint_currentPos)

    def setMotorAngle(self, motorTargetAngles):
        for i in range(self._joint_number):
            self._joint_targetPos[i] = motorTargetAngles[i]
            p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                    targetPosition=self._joint_targetPos[i],
                                    positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity)

    def setMotorsAngleInRealTimestep(self, motorTargetAngles, motorTargetTime, delayTime):
        if(motorTargetTime == 0):
            self._joint_targetPos = np.array(motorTargetAngles)
            for i in range(self._joint_number):
                p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                        targetPosition=self._joint_targetPos[i],
                                        positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity)
            time.sleep(delayTime)
        else:
            self._joint_currentPos = self._joint_targetPos
            self._joint_targetPos = np.array(motorTargetAngles)
            for i in range(self._joint_number):
                dydt = (self._joint_targetPos-self._joint_currentPos)/motorTargetTime
            internalTime = 0.0
            reft = time.time()
            while internalTime < motorTargetTime:
                internalTime = time.time() - reft
                for i in range(self._joint_number):
                    p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                            targetPosition=self._joint_currentPos[i] + dydt[i] * internalTime,
                                            positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity)

    def setMotorsAngleInFixedTimestep(self, motorTargetAngles, motorTargetTime, delayTime):
        if(motorTargetTime == 0):
            self._joint_targetPos = np.array(motorTargetAngles)
            for i in range(self._joint_number):
                p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                        targetPosition=self._joint_targetPos[i],
                                        positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity)
                p.stepSimulation()
                time.sleep(self._timeStep)
        else:
            self._joint_currentPos = self._joint_targetPos
            self._joint_targetPos = np.array(motorTargetAngles)
            for i in range(self._joint_number):
                dydt = (self._joint_targetPos-self._joint_currentPos)/motorTargetTime
            internalTime = 0.0
            while internalTime < motorTargetTime:
                internalTime += self._timeStep
                for i in range(self._joint_number):
                    p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                            targetPosition=self._joint_currentPos[i] + dydt[i] * internalTime,
                                            positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity)
                p.stepSimulation()
                # time.sleep(self._timeStep)
            '''
            for i in range(self._joint_number):
                p.setJointMotorControl2(bodyIndex=self._robot, jointIndex=self._joint_id[i], controlMode=p.POSITION_CONTROL,
                                        targetPosition=motorTargetAngles[i],
                                        positionGain=self._kp, velocityGain=self._kd, force=self._torque, maxVelocity=self._max_velocity)
            p.stepSimulation()
            # time.sleep(timestep)
            '''
            time.sleep(delayTime)
