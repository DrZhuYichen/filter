#!/usr/bin/env python
import numpy as np
from collections import deque

import rospy
import tf

class Predict:
    def __init__(self):

        self.preNum = 20
        self.preTime = 5
        self.cache = np.zeros([self.preNum, 3])
        self.posPre = np.zeros(3)

        rospy.init_node('predict', anonymous=True)
        self.handRaw = tf.TransformListener()
        self.hangPre = tf.TransformBroadcaster()
        rospy.loginfo('start')
        self.rate = rospy.Rate(20)
        pass

    def start(self):
        'main'
        (pos, rot) = self.handRaw.lookupTransform('marker_0', 'rHand', rospy.Time(0))
        self.pre(pos)
        if self.posPre[0] != 0:
            self.pub(self.posPre, rot)
        else:
            self.pub(pos, rot)

    def pre(self, pos):
        for i in range(3):
            self.posPre[i] = self.preOne(i, pos[i])

    def preOne(self, i, pos):
        'use poly method'
        data = deque(np.transpose(self.cache)[i][:])
        data.append(pos)
        data.popleft()
        np.transpose(self.cache)[i][:] = data
        if data[0] != 0:
            x = np.arange(1, len(data)+1, 1)
            y = data
            fx = np.polyfit(x, y, 2)
            fit = np.polyval(fx, len(self.cache)+self.preTime)
            return fit
        else:
            rospy.loginfo('load pos...')

    def pub(self, pos, rot):
        'publish tf transform'
        self.hangPre.sendTransform(pos, rot, rospy.Time.now(), 'marker_0', 'rHandPre')

if __name__ == '__main__':
    predict = Predict()
    while not rospy.is_shutdown():
        try:
            predict.start()
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            continue
        predict.rate.sleep()
