#!/usr/bin/env python3
import numpy as np
from collections import deque

import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState

class Filter:
    def __init__(self):

        self.jointNum = 39
        self.fitNum = 20
        self.list = np.zeros([self.fitNum, self.jointNum])
        self.fit = np.zeros(self.jointNum)

        self.AllData = []
        self.pub = rospy.Publisher('/testarm_ns/joint_states', JointState, queue_size=10)
        rospy.init_node('filter', anonymous=True)
        rospy.loginfo('start')
        pass

    def polyfit(self, data):
        self.AllData = data
        for i in range(self.jointNum):
            self.fit[i] = self.polyfitOne(i, data.position[i])
        self.AllData.position = tuple(self.fit)
        self.publish()
        # rospy.loginfo(self.AllData)
        # rospy.loginfo(self.fit)

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
            print(jonit - fit)
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

if __name__ == '__main__':
    filter = Filter()
    filter.subscribe()
