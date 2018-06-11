#!/usr/bin/env python3
import numpy as np
from collections import deque

import rospy
from sensor_msgs.msg import JointState

import xlwt
import matplotlib.pyplot as plt

class Filter:
    def __init__(self):

        self.jointNum = 39
        self.fitNum = 100
        self.list = np.zeros([self.fitNum, self.jointNum])
        self.fit = np.zeros(self.jointNum)

        self.AllData = []
        self.pub = rospy.Publisher('/testarm_ns/joint_states', JointState, queue_size=10)
        rospy.init_node('filter', anonymous=True)
        rospy.loginfo('start')

        self.showData = []
        self.frame = 0
        self.frameEnd = 500
        self.showDateNum = 35
        pass

    def polyfit(self, data):
        self.AllData = data
        for i in range(self.jointNum):
            self.fit[i] = self.polyfitOne(i, data.position[i])
        self.AllData.position = tuple(self.fit)
        self.publish()
        # rospy.loginfo(self.AllData)
        # rospy.loginfo(self.fit)

        self.frame += 1

    def polyfitOne(self, i, jonit):
        'method: polynomial fitting'
        data = deque(np.transpose(self.list)[i][:])
        data.append(jonit)
        data.popleft()
        np.transpose(self.list)[i][:] = data
        if data[0] != 0:
            x = np.arange(1, len(data)+1, 1)
            y = data
            fx = np.polyfit(x, y, 2)
            fit = np.polyval(fx, len(self.list))
            # print(jonit - fit)

            # dataRecord
            rospy.loginfo(self.frame)
            if self.fitNum < self.frame < self.frameEnd + self.fitNum and self.showDateNum == i:
                self.showData.append([self.frame, jonit, fit])
            elif self.frame == self.frameEnd + self.fitNum:
                # self.record()
                # self.plot()
                pass
            return fit
        else:
            rospy.loginfo('load...')

    def subscribe(self):
        rospy.Subscriber("/arm_ns/joint_states", JointState, self.polyfit)
        rospy.spin()

    def publish(self):
        self.pub.publish(self.AllData)
        # rospy.loginfo(self.AllData)
        pass

    def record(self):
        'data record'
        recordBook = xlwt.Workbook(encoding='ascii')
        recordSheet = recordBook.add_sheet('sheet raw')
        for i in range(len(self.showData)):
            recordSheet.write(i, 0, self.showData[i][1])
            recordSheet.write(i, 1, self.showData[i][2])
            recordSheet.write(i, 2, abs(self.showData[i][2] - self.showData[i][1]))
        recordBook.save('data_38.xls')
        print(self.frame)
        print(self.showData)

    def plot(self):
        'matplotlib'
        t = range(len(self.showData))
        joint_raw = np.transpose(self.showData)[1][:]
        joint_fit = np.transpose(self.showData)[2][:]

        raw, = plt.plot(t, joint_raw, 'b')
        fit, = plt.plot(t, joint_fit, 'r')
        plt.legend([raw, fit], ['raw', 'filter'], loc='upper right')
        plt.xlabel('data frame')
        plt.ylabel('joint state')

        plt.show()

if __name__ == '__main__':
    filter = Filter()
    filter.subscribe()
