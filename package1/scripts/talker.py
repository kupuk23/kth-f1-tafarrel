#!/usr/bin/env python


import rospy
from std_msgs.msg import String, Int16

def num_publisher():
    rospy.init_node('num_publisher', anonymous=True)

    pub = rospy.Publisher('pramono', Int16, queue_size=10) #my surname, use Int16 to reduce network traffic
    rate = rospy.Rate(20) # 20hz
    n = 4
    k = 1
    while not rospy.is_shutdown():
        k += n
        rospy.loginfo(k)
        pub.publish(k)
        rate.sleep()

if __name__ == '__main__':
    try:
        num_publisher()
    except rospy.ROSInterruptException:
        pass
