#!/usr/bin/env python
#coding=utf-8
import rospy
import math
#导入mgs到pkg中
from reader.msg import pos

#回调函数输入的应该是msg
def callback(pos):
    # distance = math.sqrt(math.pow(pos.x, 2)+math.pow(pos.y, 2)) 
    rospy.loginfo('Listener: position: tag_num=%d, std_x=%f, std_y= %f, angle= %f, state=%s', pos.tag_num, pos.x, pos.y, pos.angle, pos.state)

def listener():
    rospy.init_node('listener', anonymous=True)
    #Subscriber函数第一个参数是topic的名称，第二个参数是接受的数据类型 第三个参数是回调函数的名称
    rospy.Subscriber('position_info', pos, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()

