"""
This generates points of a gait.
It has an inverse kinematics function.
What a messy code....

by Einsbon (Sunbin Kim)
- GitHub: https://github.com/Einsbon
- Youtube:  https://www.youtube.com/channel/UCt7FZ-8uzV_jHJiKp3NlHvg
- Blog: https://blog.naver.com/einsbon
"""

import numpy as np
import matplotlib.pyplot as plt
import matplotlib as mpl
from mpl_toolkits.mplot3d import Axes3D
import math


class WalkGenerator():
    def __init__(self):
        #                        0   1   2   3   4   5   6   7   8   9  10  11
        self._motorDirection = [+1, +1, +1, -1, +1, +1, +1, -1, -1, +1, -1, -1]
        self._motorDirectionRight = [+1, +1, +1, -1, +1, +1]
        self._motorDirectionLeft = [+1, -1, -1, +1, -1, -1]
        self._anklePlus = 0
        self._walkPoint0 = 0
        self._walkPoint1 = 0
        self._walkPoint2 = 0
        self._walkPoint3 = 0
        # self._walkPoint1f = 0
        # self._walkPoint2f = 0
        self._walkPointStartRight = 0  # 오른쪽 발을 먼저 내밈. 그때의 오른쪽 발.
        self._walkPointStartLeft = 0  # 오른쪽쪽 발을 먼저 내밈. 그때의 왼쪽 발.
        self._walkPointEndRight = 0  # 오른쪽 발을 디디면서 끝남. 그때의 오른쪽 발.
        self._walkPointEndLeft = 0  # 오른쪽 발을 디디면서 끝남. 그때의 왼쪽 발.

        self._walkPoint0Inverse = 0
        self._walkPoint1Inverse = 0
        self._walkPoint2Inverse = 0
        self._walkPoint3Inverse = 0
        # self._walkPoint1fInverse = 0
        # self._walkPoint2fInverse = 0

        self._walkPointRightStep = 0
        self._walkPointLeftStep = 0

        self._walkPointRightStepInverse = 0
        self._walkPointLeftStepInverse = 0

        self._walkPointStartRightInverse = 0  # 왼쪽으로 sway 했다가 오른발을 먼저 내밈.
        self._walkPointStartLeftInverse = 0  # 오른쪽으로 sway 했다가 왼발을 먼저 내밈.
        self._walkPointEndRightInverse = 0  # 오른발을 디디면서 끝남.
        self._walkPointEndLeftInverse = 0  # 왼발을 디디면서 끝남.

        self._walkPoint0AnkleX = 0
        self._walkPoint1AnkleX = 0
        self._walkPoint2AnkleX = 0
        self._walkPoint3AnkleX = 0

        # 로봇의 다리 길이 설정. 길이 단위: mm
        self._pelvic_interval = 70.5
        self._legUp_length = 110
        self._legDown_length = 110
        self._footJoint_to_bottom = 45

        self._bodyMovePoint = 0
        self._legMovePoint = 0
        self._h = 0
        self._l = 0
        self._sit = 0
        self._swayBody = 0
        self._swayFoot = 0
        self._swayShift = 0
        self._weightStart = 0
        self._weightEnd = 0
        self._swayPlus = 0
        self._stepTime = 0
        self._bodyPositionXPlus = 0
        self._damping = 0

    def setRobotParameter(self, pelvic_interval, leg_up_length, leg_down_length, foot_to_grount, foot_to_heel, foot_to_toe):
        pass

    def setWalkParameter(self, bodyMovePoint, legMovePoint, h, l, sit, swayBody, swayFoot, bodyPositionXPlus, swayShift, weightStart, weightEnd, swayPlus, stepTime, damping, incline):
        self._bodyMovePoint = bodyMovePoint
        self._legMovePoint = legMovePoint
        self._h = h
        self._l = l
        self._sit = sit
        self._swayBody = swayBody
        self._swayFoot = swayFoot
        self._swayShift = swayShift
        self._weightStart = weightStart
        self._weightEnd = weightEnd
        self._swayPlus = swayPlus
        self._stepTime = stepTime
        self._bodyPositionXPlus = bodyPositionXPlus  # +면 몸을 앞으로, -면 몸을 뒤로 하고 걸음.
        self._damping = damping
        self._incline = incline

    def generate(self):
        walkPoint = self._bodyMovePoint*2+self._legMovePoint*2
        trajectoryLength = self._l*(2*self._bodyMovePoint + self._legMovePoint) / \
            (self._bodyMovePoint + self._legMovePoint)
        print('trajectoryLength')
        print(trajectoryLength)

        walkPoint0 = np.zeros((3, self._bodyMovePoint))
        walkPoint1 = np.zeros((3, self._legMovePoint))
        walkPoint2 = np.zeros((3, self._bodyMovePoint))
        walkPoint3 = np.zeros((3, self._legMovePoint))

        self._walkPointStartRight = np.zeros((3, self._bodyMovePoint+self._legMovePoint))
        self._walkPointStartLeft = np.zeros((3, self._bodyMovePoint+self._legMovePoint))
        self._walkPointEndRight = np.zeros((3, self._bodyMovePoint+self._legMovePoint))
        self._walkPointEndLeft = np.zeros((3, self._bodyMovePoint+self._legMovePoint))

        for i in range(self._bodyMovePoint):
            t = (i+1)/(walkPoint-self._legMovePoint)
            walkPoint0[0][i] = -trajectoryLength*(t-0.5)
            walkPoint0[2][i] = self._sit
            walkPoint0[1][i] = self._swayBody*math.sin(2 * math.pi*((i+1-self._swayShift)/walkPoint))

        for i in range(self._legMovePoint):
            t = (i+1 + self._bodyMovePoint)/(walkPoint-self._legMovePoint)
            walkPoint1[0][i] = -trajectoryLength*(t-0.5)
            walkPoint1[2][i] = self._sit
            walkPoint1[1][i] = self._swayBody * \
                math.sin(2 * math.pi*((i + 1 + self._bodyMovePoint-self._swayShift)/walkPoint))

        for i in range(self._bodyMovePoint):
            t = (i + 1 + self._bodyMovePoint+self._legMovePoint)/(walkPoint-self._legMovePoint)
            walkPoint2[0][i] = -trajectoryLength*(t-0.5)
            walkPoint2[2][i] = self._sit
            walkPoint2[1][i] = self._swayBody * \
                math.sin(2 * math.pi*((i + 1 + self._bodyMovePoint+self._legMovePoint-self._swayShift)/walkPoint))

        for i in range(self._legMovePoint):
            t = (i+1) / self._legMovePoint
            sin_tpi = math.sin(t * math.pi)

            walkPoint3[0][i] = (2 * t - 1 + (1-t) * self._weightStart * -sin_tpi +
                                t * self._weightEnd * sin_tpi) * trajectoryLength / 2
            walkPoint3[2][i] = math.sin(t * math.pi) * self._h + self._sit
            walkPoint3[1][i] = math.sin(t * math.pi) * self._swayFoot + self._swayBody * \
                math.sin(2 * math.pi*((i+1+walkPoint-self._legMovePoint-self._swayShift)/walkPoint))

        # 시작 동작 만들기
        for i in range(self._bodyMovePoint-self._swayShift):
            t = (i+1)/self._bodyMovePoint
            self._walkPointStartRight[0][i] = 0
            self._walkPointStartRight[2][i] = self._sit

            self._walkPointStartLeft[0][i] = 0
            self._walkPointStartLeft[2][i] = self._sit
        for i in range(self._legMovePoint):
            t = (i+1)/self._legMovePoint
            t2 = (i+1)/(self._legMovePoint+self._swayShift)
            sin_tpi = math.sin(t * math.pi)

            self._walkPointStartRight[2][i+self._bodyMovePoint -
                                         self._swayShift] = math.sin(t * math.pi) * self._h + self._sit
            self._walkPointStartRight[0][i+self._bodyMovePoint - self._swayShift] = (
                2 * t + (1-t) * self._weightStart * -sin_tpi + t * self._weightEnd * sin_tpi) * trajectoryLength / 4
            self._walkPointStartLeft[0][i+self._bodyMovePoint-self._swayShift] = (math.cos(
                t2*math.pi/2)-1) * trajectoryLength * self._legMovePoint/(self._bodyMovePoint*2+self._legMovePoint)/2
            self._walkPointStartLeft[0][i+self._bodyMovePoint-self._swayShift] = (math.cos(t2*math.pi/2)-1) * trajectoryLength * (
                (self._swayShift+self._bodyMovePoint+self._legMovePoint)/(self._bodyMovePoint*2+self._legMovePoint)-0.5)

            self._walkPointStartLeft[2][i+self._bodyMovePoint-self._swayShift] = self._sit

        for i in range(self._swayShift):
            t2 = (i+1+self._legMovePoint)/(self._legMovePoint+self._swayShift)

            self._walkPointStartRight[0][i+self._legMovePoint+self._bodyMovePoint-self._swayShift] = - \
                trajectoryLength*((i+1)/(walkPoint-self._legMovePoint)-0.5)
            self._walkPointStartRight[2][i+self._legMovePoint+self._bodyMovePoint-self._swayShift] = self._sit

            self._walkPointStartLeft[0][i+self._legMovePoint+self._bodyMovePoint-self._swayShift] = - \
                trajectoryLength*((i + 1 + self._bodyMovePoint+self._legMovePoint)/(walkPoint-self._legMovePoint)-0.5)

            self._walkPointStartLeft[0][i+self._legMovePoint+self._bodyMovePoint-self._swayShift] = (math.cos(t2*math.pi/2)-1) * trajectoryLength * (
                (self._swayShift+self._bodyMovePoint+self._legMovePoint)/(self._bodyMovePoint*2+self._legMovePoint)-0.5)

            self._walkPointStartLeft[2][i+self._legMovePoint+self._bodyMovePoint-self._swayShift] = self._sit

        for i in range(self._bodyMovePoint+self._legMovePoint):
            t = (i+1)/(self._bodyMovePoint+self._legMovePoint)
            #self._walkPointStartRight[1][i] = -self._swayBody * math.sin(t*math.pi) * math.sin(t*math.pi)
            #self._walkPointStartLeft[1][i] = self._swayBody * math.sin(t*math.pi) * math.sin(t*math.pi)
            if t < 1/4:
                self._walkPointStartRight[1][i] = -self._swayBody * \
                    (math.sin(t*math.pi) - (1-math.sin(math.pi*2*t))*(math.sin(4*t*math.pi)/4))
                self._walkPointStartLeft[1][i] = self._swayBody * \
                    (math.sin(t*math.pi) - (1-math.sin(math.pi*2*t))*(math.sin(4*t*math.pi)/4))
            else:
                self._walkPointStartRight[1][i] = -self._swayBody * math.sin(t*math.pi)
                self._walkPointStartLeft[1][i] = self._swayBody * math.sin(t*math.pi)

        # 마무리 동작 만들기. 왼발이 뜸. 그러나 둘다 오른쪽다리 기준
        for i in range(self._bodyMovePoint-self._swayShift):
            self._walkPointEndRight[0][i] = -trajectoryLength*((i+1+self._swayShift)/(walkPoint-self._legMovePoint)-0.5)
            self._walkPointEndRight[2][i] = self._sit

            self._walkPointEndLeft[0][i] = -trajectoryLength * \
                ((i + 1 + self._swayShift + self._bodyMovePoint+self._legMovePoint)/(walkPoint-self._legMovePoint)-0.5)
            self._walkPointEndLeft[2][i] = self._sit
        for i in range(self._legMovePoint):
            t = (i+1)/self._legMovePoint
            sin_tpi = math.sin(t * math.pi)

            self._walkPointEndRight[0][i+self._bodyMovePoint-self._swayShift] = (math.sin(t*math.pi/2)-1) * trajectoryLength * (
                (self._bodyMovePoint)/(self._bodyMovePoint*2+self._legMovePoint)-0.5)
            self._walkPointEndRight[2][i+self._bodyMovePoint-self._swayShift] = self._sit

            self._walkPointEndLeft[0][i+self._bodyMovePoint-self._swayShift] = (
                2 * t-2 + (1-t) * self._weightStart * -sin_tpi + t * self._weightEnd * sin_tpi) * trajectoryLength / 4
            self._walkPointEndLeft[2][i+self._bodyMovePoint -
                                      self._swayShift] = math.sin(t * math.pi) * self._h + self._sit
        for i in range(self._swayShift):
            self._walkPointEndRight[0][i+self._bodyMovePoint+self._legMovePoint-self._swayShift] = 0
            self._walkPointEndRight[2][i+self._bodyMovePoint+self._legMovePoint-self._swayShift] = self._sit

            self._walkPointEndLeft[0][i+self._bodyMovePoint+self._legMovePoint-self._swayShift] = 0
            self._walkPointEndLeft[2][i+self._bodyMovePoint+self._legMovePoint-self._swayShift] = self._sit

        for i in range(self._bodyMovePoint+self._legMovePoint):
            t = 1 - (i+1)/(self._bodyMovePoint+self._legMovePoint)

            if t < 1/4:
                self._walkPointEndRight[1][i] = self._swayBody * \
                    (math.sin(t*math.pi) - (1-math.sin(math.pi*2*t))*(math.sin(4*t*math.pi)/4))
                self._walkPointEndLeft[1][i] = -self._swayBody * \
                    (math.sin(t*math.pi) - (1-math.sin(math.pi*2*t))*(math.sin(4*t*math.pi)/4))
            else:
                self._walkPointEndRight[1][i] = self._swayBody * math.sin(t*math.pi)
                self._walkPointEndLeft[1][i] = -self._swayBody * math.sin(t*math.pi)

        # 추가 파라미터의 조정

        if self._incline != 0:  # 기울기. 계단 등에서 사용.
            walkPoint0[2] = walkPoint0[2] + walkPoint0[0]*self._incline
            walkPoint1[2] = walkPoint1[2] + walkPoint1[0]*self._incline
            walkPoint2[2] = walkPoint2[2] + walkPoint2[0]*self._incline
            walkPoint3[2] = walkPoint3[2] + walkPoint3[0]*self._incline
            self._walkPointStartRight[2] = self._walkPointStartRight[2] + self._walkPointStartRight[0]*self._incline
            self._walkPointStartLeft[2] = self._walkPointStartLeft[2] + self._walkPointStartLeft[0]*self._incline
            self._walkPointEndRight[2] = self._walkPointEndRight[2] + self._walkPointEndRight[0]*self._incline
            self._walkPointEndLeft[2] = self._walkPointEndLeft[2] + self._walkPointEndLeft[0]*self._incline

        if self._bodyPositionXPlus != 0:  # 허리 앞뒤 위치 조절
            walkPoint0[0] = walkPoint0[0] - self._bodyPositionXPlus
            walkPoint1[0] = walkPoint1[0] - self._bodyPositionXPlus
            walkPoint2[0] = walkPoint2[0] - self._bodyPositionXPlus
            walkPoint3[0] = walkPoint3[0] - self._bodyPositionXPlus
            self._walkPointStartRight[0] = self._walkPointStartRight[0] - self._bodyPositionXPlus
            self._walkPointStartLeft[0] = self._walkPointStartLeft[0] - self._bodyPositionXPlus
            self._walkPointEndRight[0] = self._walkPointEndRight[0] - self._bodyPositionXPlus
            self._walkPointEndLeft[0] = self._walkPointEndLeft[0] - self._bodyPositionXPlus

        if self._damping != 0:  # 댐핑 조절
            dampHeight = (walkPoint3[2][-1]-walkPoint0[2][0])/2
            walkPoint0[2][0] = walkPoint0[2][0]+dampHeight*self._damping
            walkPoint2[2][0] = walkPoint2[2][0]-dampHeight*self._damping

        self._walkPoint0 = walkPoint0
        self._walkPoint1 = walkPoint1
        self._walkPoint2 = walkPoint2
        self._walkPoint3 = walkPoint3

        self._walkPointLeftStep = np.column_stack(
            [walkPoint0[:, self._swayShift:], walkPoint1, walkPoint2[:, :self._swayShift]])
        self._walkPointRightStep = np.column_stack(
            [walkPoint2[:, self._swayShift:], walkPoint3, walkPoint0[:, :self._swayShift]])

    def inverseKinematics(self, point, isRightLeg):
        inverseAngle = np.zeros((point[0].size, 6))
        for i in range(point[0].size):
            x = point[0][i]
            y = point[1][i]
            z = point[2][i]

            l3 = self._legUp_length
            l4 = self._legDown_length

            fx = x
            fy = y
            fz = self._legUp_length + self._legDown_length - z

            a = math.sqrt(fx*fx + fy * fy + fz * fz)

            d1 = math.asin(fx/a)
            d2 = math.acos((l3*l3+a*a-l4*l4)/(2*l3*a))
            d3 = math.acos(fz/a)
            d4 = math.acos((l4*l4+a*a-l3*l3)/(2*l4*a))
            d5 = math.pi-d2-d4

            t1 = (math.atan2(fy, fz))
            t2 = d1+d2
            t3 = math.pi-d5
            t4 = -t2+t3
            t5 = -t1

            if isRightLeg:
                inverseAngle[i] = np.array([0, t1, t2, t3, t4, t5]) * self._motorDirectionRight
            else:
                inverseAngle[i] = np.array([0, t1, t2, t3, t4, t5]) * self._motorDirectionLeft

        return inverseAngle

    def showGaitPoint2D(self):
        plt.plot(self._walkPoint0[0], self._walkPoint0[2], 'o-', c='red',  ms=7, lw=5)
        plt.plot(self._walkPoint1[0], self._walkPoint1[2], 'o-', c='blue', ms=7, lw=5)
        plt.plot(self._walkPoint2[0], self._walkPoint2[2], 'o-', c='red',  ms=7, lw=5)
        plt.plot(self._walkPoint3[0], self._walkPoint3[2], 'o-', c='blue', ms=7, lw=5)

        plt.plot(self._walkPointStartRight[0], self._walkPointStartRight[2], '*-')
        plt.plot(self._walkPointStartLeft[0], self._walkPointStartLeft[2],   '*-')
        plt.plot(self._walkPointEndRight[0], self._walkPointEndRight[2],     '*-')
        plt.plot(self._walkPointEndLeft[0], self._walkPointEndLeft[2],       '*-')

        plt.show()

    def showGaitPoint2DTop(self):
        plt.plot(self._walkPoint0[0], self._walkPoint0[1], 'o-')
        plt.plot(self._walkPoint1[0], self._walkPoint1[1], 'o-')
        plt.plot(self._walkPoint2[0], self._walkPoint2[1], 'o-')
        plt.plot(self._walkPoint3[0], self._walkPoint3[1], 'o-')

        plt.plot(self._walkPointStartRight[0], self._walkPointStartRight[1], '.-')
        plt.plot(self._walkPointStartLeft[0], self._walkPointStartLeft[1], '.-')

        plt.plot(self._walkPointEndRight[0], self._walkPointEndRight[1], '+-')
        plt.plot(self._walkPointEndLeft[0], self._walkPointEndLeft[1], '+-')

        plt.show()

    def showGaitPoint3D(self):
        fig = plt.figure(1)
        ax = fig.add_subplot(111, projection='3d')

        ax.plot(self._walkPointRightStep[0], self._walkPointRightStep[1],
                self._walkPointRightStep[2], 'co-', lw=10, ms=6)
        ax.plot(self._walkPointLeftStep[0], self._walkPointLeftStep[1], self._walkPointLeftStep[2], 'mo-', lw=10, ms=5)

        ax.plot(self._walkPoint0[0], self._walkPoint0[1], self._walkPoint0[2], 'o')
        ax.plot(self._walkPoint1[0], self._walkPoint1[1], self._walkPoint1[2], 'o')
        ax.plot(self._walkPoint2[0], self._walkPoint2[1], self._walkPoint2[2], 'o')
        ax.plot(self._walkPoint3[0], self._walkPoint3[1], self._walkPoint3[2], 'o')

        ax.plot(self._walkPointStartRight[0], self._walkPointStartRight[1], self._walkPointStartRight[2], '*-')
        ax.plot(self._walkPointStartLeft[0], self._walkPointStartLeft[1], self._walkPointStartLeft[2], '*-')

        ax.plot(self._walkPointEndRight[0], self._walkPointEndRight[1], self._walkPointEndRight[2], '+-')
        ax.plot(self._walkPointEndLeft[0], self._walkPointEndLeft[1], self._walkPointEndLeft[2], '+-')

        plt.show()

    def inverseKinematicsAll(self):
        walkPoint0Inverse_right = self.inverseKinematics(self._walkPoint0, True)
        walkPoint1Inverse_right = self.inverseKinematics(self._walkPoint1, True)
        walkPoint2Inverse_right = self.inverseKinematics(self._walkPoint2, True)
        walkPoint3Inverse_right = self.inverseKinematics(self._walkPoint3, True)

        walkPoint0Inverse_left = self.inverseKinematics(self._walkPoint0, False)
        walkPoint1Inverse_left = self.inverseKinematics(self._walkPoint1, False)
        walkPoint2Inverse_left = self.inverseKinematics(self._walkPoint2, False)
        walkPoint3Inverse_left = self.inverseKinematics(self._walkPoint3, False)

        walkpointstartRight_rightLeg_inverse = self.inverseKinematics(self._walkPointStartRight, True)
        walkpointstartRight_leftLeg_inverse = self.inverseKinematics(self._walkPointStartLeft, False)

        walkpointstartLeft_rightLeg_inverse = self.inverseKinematics(self._walkPointStartLeft, True)
        walkpointstartLeft_leftLeg_inverse = self.inverseKinematics(self._walkPointStartRight, False)

        walkPointEndRight_rightLeg_inverse = self.inverseKinematics(self._walkPointEndRight, True)
        walkPointEndRight_leftleg_inverse = self.inverseKinematics(self._walkPointEndLeft, False)

        walkpointEndLeft_rightLeg_inverse = self.inverseKinematics(self._walkPointEndLeft, True)
        walkpointEndLeft_leftleg_inverse = self.inverseKinematics(self._walkPointEndRight, False)

        walkPoint_rightStep_rightLeg = self.inverseKinematics(self._walkPointRightStep, True)
        walkPoint_rightStep_leftLeg = self.inverseKinematics(self._walkPointLeftStep, False)

        walkPoint_leftStep_rightLeg = self.inverseKinematics(self._walkPointLeftStep, True)
        walkPoint_leftStep_leftLeg = self.inverseKinematics(self._walkPointRightStep, False)

        self._walkPoint0Inverse = np.column_stack([walkPoint0Inverse_right, walkPoint2Inverse_left])
        self._walkPoint1Inverse = np.column_stack([walkPoint1Inverse_right, walkPoint3Inverse_left])
        self._walkPoint2Inverse = np.column_stack([walkPoint2Inverse_right, walkPoint0Inverse_left])
        self._walkPoint3Inverse = np.column_stack([walkPoint3Inverse_right, walkPoint1Inverse_left])

        self._walkPointStartRightInverse = np.column_stack(
            [walkpointstartRight_rightLeg_inverse, walkpointstartRight_leftLeg_inverse])
        self._walkPointStartLeftInverse = np.column_stack(
            [walkpointstartLeft_rightLeg_inverse, walkpointstartLeft_leftLeg_inverse])

        self._walkPointEndLeftInverse = np.column_stack(
            [walkPointEndRight_rightLeg_inverse, walkPointEndRight_leftleg_inverse])
        self._walkPointEndRightInverse = np.column_stack(
            [walkpointEndLeft_rightLeg_inverse, walkpointEndLeft_leftleg_inverse])
        # self._walkPointStartLeftInverse = walkpointstartLeft_inverse
        self._walkPointRightStepInverse = np.column_stack([walkPoint_rightStep_rightLeg, walkPoint_rightStep_leftLeg])
        self._walkPointLeftStepInverse = np.column_stack([walkPoint_leftStep_rightLeg, walkPoint_leftStep_leftLeg])


def main():
    walk = WalkGenerator()
    walk.setWalkParameter(bodyMovePoint=16, legMovePoint=16, h=50, l=80, sit=30, swayBody=60, swayFoot=0,
                          bodyPositionXPlus=0, swayShift=6, weightStart=0.4, weightEnd=0.7, swayPlus=0, stepTime=0.06, damping=0.0, incline=0.0)
    walk.generate()
    walk.showGaitPoint2D()
    walk.showGaitPoint2DTop()
    walk.showGaitPoint3D()


if __name__ == "__main__":
    main()
