#!/usr/bin/env python
from geometry_msgs.msg import Pose
import rosbag

#THIS SCRIPT PRINTS OUT ROSBAG FILES, FOR TESTING

demo_num = 5 #change this number depending on which file
bag = rosbag.Bag('/home/nq/raw_demo' + str(demo_num) + '.bag')

i=1

for topic, msg, t in bag.read_messages(topics='/ee_pose'): #find the initial and final pose
    print(i) #print the message
    print(msg)

    i+=1

bag.close()