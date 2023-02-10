#!/usr/bin/env python3
import rospy 
from sensor_msgs.msg import JointState 
 
def callback(JointState): 
    global is_header
    js_header=JointState.velocity
    joint1_vel=js_header[0]
    rospy.loginfo('I heard %s',joint1_vel) 
    print(1)#rospy.loginfo('I heard %s', JointState) 
 
def listener(): 
 
    # In ROS, nodes are uniquely named. If two nodes with the same 
    # name are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique 
    # name for our 'listener' node so that multiple listeners can 
    # run simultaneously. 


    rospy.init_node('listener', anonymous=True) 
 
    joints=rospy.Subscriber('/joint_states', JointState, callback) 
    #msg = rospy.wait_for_message("/joint_states", JointState) 

    #print(joints) 

    # spin() simply keeps python from exiting until this node is stopped 
    rospy.spin() 
 
if __name__ == '__main__': 
    listener() 
# loop (1s) 
#   subscribe data from ROS

#   Construct MTConnect data from received data
str = "2023-02-03:10.00.00.0001|Joint1|10.01|Joint2|1.13|2.3|5.5|3.3|1.1|0"
	
# https://github.com/nombreinvicto/HaasMTConnect

#   Send the constructed data to MTConnect agent (127.0.0.1:5000)

print("This module is working")
print(str)

