#!/usr/bin/env python

import rospy
from std_msgs.msg import String, Int16, Float32



class DataTransformerNode:
    def __init__(self):
        rospy.init_node('data_transformer', anonymous=True)
        self.q=0.15
        self.result = 0
        rospy.Subscriber('pramono', Int16, self.callback)
        self.pub = rospy.Publisher('/kthfs/result', Float32, queue_size=10)
        rospy.Timer(rospy.Duration(1.0/20.0), self.publish_result)

    def publish_result(self,event):
        if self.result:
            self.pub.publish(self.result)

    def callback(self,data):
        
        self.result = Float32(data.data/self.q)
        rospy.loginfo(self.result)
        

if __name__ == '__main__':
    try:
        node = DataTransformerNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
