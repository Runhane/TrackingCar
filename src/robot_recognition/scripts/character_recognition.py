
# -*- coding: utf-8 -*-
import rospy
from std_msgs.msg import Bool

if __name__ == "__main__":
   
    rospy.init_node("character recognition")

    pub = rospy.Publisher("/robot_recognition/flag",Bool,queue_size=10)

    rate = rospy.Rate(30)
    # msg = Flag()
    # msg.data = true
    # while not rospy.is_shutdown():
    #     pub.publish(msg)
    #     rate.sleep()