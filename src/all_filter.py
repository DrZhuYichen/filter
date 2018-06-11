#!/usr/bin/env python
import numpy as np
from collections import deque

import rospy
import tf
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PointStamped

import matplotlib.pyplot as plt

class Filter:
    def __init__(self):

        self.jointNum = 39
        self.jointFitNum = 40
        self.jointList = np.zeros([self.jointFitNum, self.jointNum])
        self.jointFit = np.zeros(self.jointNum)
        self.AllJointData = []

        self.posNum = 3
        self.posFitNum = 40
        self.posList = np.zeros([self.posFitNum, self.posNum])
        self.posFit = np.zeros(self.posNum)
        self.AllPosData = []

        self.ifPlot = 0

        self.pubPos = tf.TransformBroadcaster()
        self.pubJoint = rospy.Publisher('/testarm_ns/joint_states', JointState, queue_size=10)
        rospy.init_node('filter', anonymous=True)
        rospy.loginfo('start')

        self.frame = 0
        self.frameEnd = 150
        self.showData = []
        self.showIndex = 1

        pass

    def polyfit_joint(self, data):
        temp = data.position[:]
        self.AllJointData = data
        for i in range(self.jointNum):
            self.jointFit[i] = self.polyFitJointOne(i, data.position[i])
        self.AllJointData.position = tuple(self.jointFit)

        self.publish()

        self.frame += 1
        # self.polt(temp[self.showIndex], self.jointFit[self.showIndex])

    def polyfit_pos(self, data):
        self.AllPosData = data
        temp = [data.point.x, data.point.y, data.point.z]
        for i in range(self.posNum):
            self.posFit[i] = self.polyFitPosOne(i, temp[i])
        self.AllPosData.point.x, self.AllPosData.point.y, self.AllPosData.point.z \
            = self.posFit[0], self.posFit[1], self.posFit[2]

        temp2 = [self.posFit[0], self.posFit[1], self.posFit[2]]
        self.polt(temp[self.showIndex], temp2[self.showIndex])

    def polyFitJointOne(self, i, jonit):
        'method: polynomial fitting for joint'
        data = deque(np.transpose(self.jointList)[i][:])
        data.append(jonit)
        data.popleft()
        np.transpose(self.jointList)[i][:] = data
        if data[0] != 0:
            x = np.arange(1, len(data)+1, 1)
            y = data
            fx = np.polyfit(x, y, 2)
            fit = np.polyval(fx, len(self.jointList))
            # print(jonit - fit)
            return fit
        else:
            rospy.loginfo('load joint...')

    def polyFitPosOne(self, i, pos):
        'method: polynomial fitting for pos'
        data = deque(np.transpose(self.posList)[i][:])
        data.append(pos)
        data.popleft()
        np.transpose(self.posList)[i][:] = data
        if data[0] != 0:
            x = np.arange(1, len(data)+1, 1)
            y = data
            fx = np.polyfit(x, y, 2)
            fit = np.polyval(fx, len(self.posList))
            return fit
        else:
            rospy.loginfo('load pos...')

    def subscribe(self):
        rospy.Subscriber("/arm_ns/joint_states_raw", JointState, self.polyfit_joint)
        rospy.Subscriber("/inertial_poser/global_position_raw", PointStamped, self.polyfit_pos)
        rospy.spin()

    def publish(self):
        self.pubJoint.publish(self.AllJointData)
        self.pubPos.sendTransform(
            (self.AllPosData.point.x, self.AllPosData.point.y, self.AllPosData.point.z),
            (0.0, 0.0, 0.0, 1.0), rospy.Time.now(),
            'marker_0', 'world')
        pass

    def polt(self, raw, fit):
        'matplotlib'
        print(self.frame)
        # print(raw - fit)
        if self.frameEnd > self.frame > max(self.jointFitNum, self.posFitNum):
            self.showData.append([raw, fit])
            pass
        elif self.frame > self.frameEnd and self.ifPlot == 1:
            t = range(len(self.showData))
            joint_raw = np.transpose(self.showData)[0][:]
            joint_fit = np.transpose(self.showData)[1][:]

            raw, = plt.plot(t, joint_raw, 'b')
            fit, = plt.plot(t, joint_fit, 'r')
            plt.legend([raw, fit], ['raw', 'filter'], loc='upper right')
            plt.xlabel('data frame')
            plt.ylabel('state')

            plt.show()

if __name__ == '__main__':
    filter = Filter()
    filter.subscribe()
