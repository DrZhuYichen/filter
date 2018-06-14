#!/usr/bin/env python
import rospy
import tf

import numpy as np

class Dot:
    def __init__(self):
        self.table = rospy.get_param('/intersection_pub/table')
        self.table_offset = rospy.get_param('/intersection_pub/table_offset')

        rospy.init_node('intersection_pub', anonymous=True)
        self.dot_arm = tf.TransformListener()
        self.dot_wrist = tf.TransformListener()
        self.dot_goal = tf.TransformBroadcaster()

        self.rate = rospy.Rate(20)
        rospy.loginfo('start')

    def sub(self):
        while not rospy.is_shutdown():
            try:
                (pos_arm, rot_arm) = self.dot_arm.lookupTransform('base', 'rArm', rospy.Time(0))
                (pos_wrist, rot_wrist) = self.dot_wrist.lookupTransform('base', 'rWrist', rospy.Time(0))
                (x, y, z) = self.calculate(pos_arm[0], pos_arm[1], pos_arm[2],
                                           pos_wrist[0], pos_wrist[1], pos_wrist[2])

                out = self.judge(pos_arm[0], pos_arm[1], pos_arm[2],
                                 pos_wrist[0], pos_wrist[1], pos_wrist[2], x, y, z)
                print(out)
                if out == 0:
                    self.pub(x, y, z)
            except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
                continue
            self.rate.sleep()

    def calculate(self, x1, y1, z1, x2, y2, z2):
        z = self.table + self.table_offset
        x = (z-z1)*(x2-x1)/(z2-z1)+x1
        y = (z-z1)*(y2-y1)/(z2-z1)+y1
        return x, y, z

    def judge(self, x1, y1, z1, x2, y2, z2, x3, y3, z3):
        a = (x2-x1)*(x3-x1) + (y2-y1)*(y3-y1) + (z2-z1)*(z3-z1)
        b = np.square(x2-x1) + np.square(y2-y1) + np.square(z2-z1)
        c = np.square(x3-x1) + np.square(y3-y1) + np.square(z3-z1)
        d = np.sqrt(b) * np.sqrt(c)
        cos = a / d
        if 1.05 >= cos >= 0.95:
            out = 0
        else:
            out = 1
        return out

    def pub(self, x, y, z):
        self.dot_goal.sendTransform((x, y, z), (0, 0, 0, 1), rospy.Time.now(), 'desired_goal', 'base')
        pass

if __name__ == '__main__':
    intersection = Dot()
    intersection.sub()
