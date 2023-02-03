#!/usr/bin/env python
from geometry_msgs.msg import Pose
import rosbag
import numpy as np #you need numpy
from scipy import interpolate
import math 
import matplotlib.pyplot as plt

#THIS SCRIPT PROCESSES THE RAW ROSBAG FILE TO BE USED TO GENERATE TRAJECTORIES

print("Enter the file number: ") 
demo_num = input() #input the file number
folder_name = 'planner_rrt_400'
filename = '/home/camilo/custom_ws/src/iiwa_stack/iiwa_custom/src/' + folder_name + '/raw_demo' + str(demo_num) + '.bag'

bag = rosbag.Bag(filename) #import the bag file

initial_pose = Pose() #initiate empty pose messages
final_pose = Pose()

i = 1
first = 1
last = bag.get_message_count()
initial_time = 0

for topic, msg, t in bag.read_messages(topics='/ee_pose'): #find the initial and final pose
    if i==1:
        initial_pose = msg
    elif i==bag.get_message_count():
        final_pose = msg
    i += 1
i = 1

for topic, msg, t in bag.read_messages(topics='/ee_pose'): #find the beginning and end msg numbers
    if msg == initial_pose:
        first = i
        initial_time = t
    if msg != final_pose:
        last = i
    i += 1
i=1
last += 1

bag.close() #close the bag file

num_msgs = last-first #determine number of unique messages in bag file

with rosbag.Bag('/home/camilo/iiwa_bag/demo' + str(demo_num) + '.bag', 'w') as outbag: #export trimmed bag file
    for topic, msg, t in rosbag.Bag(filename).read_messages():
        if i >= first and i <= last:
            t -= initial_time
            outbag.write(topic, msg, t)
        i+=1

bag = rosbag.Bag('/home/camilo/iiwa_bag/demo' + str(demo_num) + '.bag') #import trimmed bag fiile

x = list()
y = list()
z = list() #should be t
i1 = 0
i=0
for topic, msg, t in bag.read_messages(topics='/ee_pose'): #take the xyz (z is time here) from new bag file and add to list
   if i == 0 and i1 == 0:
            x.append(msg.position.x)
	    y.append(msg.position.y)
            z.append(i)
            i+=1
   if i1 >= 3:
	    x.append(msg.position.x)
	    y.append(msg.position.y)
	    z.append(i)
	    print(t.to_sec())
	    print(msg)
	    i += 1
            i1 = 0
   else:
	    i1 += 1
   prevx = msg.position.x
   prevy = msg.position.y
   #print("x is: " + str(prevx) + " y is: " + str(prevy))

if not(i1 == 0):
    x.append(prevx)
    y.append(prevy)
    z.append(i+1)
print(z)
duration = z[len(z)-1] #linear interpolation to create 100 points, with time being 10 seconds as required for the TPGMM
length = 400#len(z) 100
new_t = np.linspace(0,duration,num = length)#num = 100)

new_x = np.interp(new_t, z, x)
new_y = np.interp(new_t, z, y)

z = np.linspace(0,10,num = length)#num = 100)

data = np.row_stack((z, new_x, new_y)) #add the lists together to create a numpy array
data = np.transpose(data) #tranpose the data so it conforms to TPGMM format
np.save('/home/camilo/custom_ws/src/iiwa_stack/iiwa_custom/src/' + folder_name + '/demo' + str(demo_num) + '.npy', data) #exports data as a .npy file
#plt.figure()
#plt.plot(data[:,1], data[:,2])
#plt.show()

print(first,last) #prints out the first and last numbers as a sanity check
bag.close()

