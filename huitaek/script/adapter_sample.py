# MTConnet adapter sample for ME597 Lab5

import random
import sys
import time
import datetime
from data_item import Event, Sample # load data_item package
from mtconnect_adapter import Adapter # load mtconnect_adapter package
import rospy 
from sensor_msgs.msg import JointState 

class MTConnectAdapter(object):

    def __init__(self, host, port):
        # MTConnect adapter connection info
        self.host = host
        self.port = port
        self.adapter = Adapter((host, port))

        # For samples
        self.j1 = Sample('j1') # self.a1 takes 'a1' data item id.
        self.adapter.add_data_item(self.j1) # adding self.a1 as a data item
        self.j2 = Sample('j2') # self.t1 takes 't1' data item id.
        self.adapter.add_data_item(self.j2) # adding self.t1 as a data item
        self.j3 = Sample('j3') # self.j3 takes 'j3' data item id.
        self.adapter.add_data_item(self.j3) # adding self.t1 as a data item
        self.j4 = Sample('j4') # self.j4 takes 'j4' data item id.
        self.adapter.add_data_item(self.j4) # adding self.t1 as a data item
        self.j5 = Sample('j5') # self.j5 takes 'j5' data item id.
        self.adapter.add_data_item(self.j5) # adding self.t1 as a data item
        self.j6 = Sample('j6') # self.t1 takes 't1' data item id.
        self.adapter.add_data_item(self.j6) # adding self.t1 as a data item
        ## Add more samples below

        # For events
        self.event = Event('event') # self.event takes 'event' data item name.
        self.adapter.add_data_item(self.event) # adding self.event as a data item
        ## Add more events below

        # MTConnnect adapter availability
        self.avail = Event('avail')
        self.adapter.add_data_item(self.avail)

        # Start MTConnect
        self.adapter.start()
        self.adapter.begin_gather()
        self.avail.set_value("AVAILABLE")
        self.adapter.complete_gather()
        # self.adapter_stream()

    def callback(self,JointState): 
        self.js_header=JointState.velocity
        j1=self.js_header[0]
        j2=self.js_header[1]
        j3=self.js_header[2]
        j4=self.js_header[3]
        j5=self.js_header[4]
        j6=self.js_header[5]
        rospy.loginfo('I heard %s',self.js_header) 
        while True:
            try:
                # Do something here.
                # a1 = random.uniform(-1,1) # this example is to take a random float between -1 and 1.
                # t1 = random.uniform(20,25) # this example is to take a random float between 15 and 25.
                
                self.adapter.begin_gather()
                self.j1.set_value(str(j1)) # set value of a1 data item, format: str(float)
                self.j2.set_value(str(j2))
                self.j3.set_value(str(j3))
                self.j4.set_value(str(j4))
                self.j5.set_value(str(j5))
                self.j6.set_value(str(j6)) # set value of t1 data item, format: str(float)
                self.adapter.complete_gather()

                print("{} Joint 1 Velocity={} mm/s".format(datetime.datetime.now(), j1)) # printing out datetime now and a1
                print("{} Joint 2 Velocity={} mm/s".format(datetime.datetime.now(), j2)) # printing out datetime now and a1
                print("{} Joint 3 Velocity={} mm/s".format(datetime.datetime.now(), j3)) # printing out datetime now and a1
                print("{} Joint 4 Velocity={} mm/s".format(datetime.datetime.now(), j4)) # printing out datetime now and a1
                print("{} Joint 5 Velocity={} mm/s".format(datetime.datetime.now(), j5)) # printing out datetime now and a1
                print("{} Joint 6 Velocity={} mm/s".format(datetime.datetime.now(), j6)) # printing out datetime now and a1
                print(datetime.datetime.now(), "MTConnect data items gathering completed...\n") # printing out MTConnect data collection is done.

                time.sleep(2) # wait for 2 seconds

            except KeyboardInterrupt:
                print("Stopping MTConnect...")
                self.adapter.stop() # Stop adapter thread
                sys.exit() # Terminate Python
        #print(1)#rospy.loginfo('I heard %s', JointState) 

    
    def listener(self): 
 
    # In ROS, nodes are uniquely named. If two nodes with the same 
    # name are launched, the previous one is kicked off. The 
    # anonymous=True flag means that rospy will choose a unique 
    # name for our 'listener' node so that multiple listeners can 
    # run simultaneously. 


        rospy.init_node('listener', anonymous=True) 
    
        joints=rospy.Subscriber('/joint_states', JointState, self.callback) 
        #msg = rospy.wait_for_message("/joint_states", JointState) 

        #print(joints) 

        # spin() simply keeps python from exiting until this node is stopped 
        rospy.spin() 

    # def adapter_stream(self):
    #     while True:
    #         try:
    #             # Do something here.
    #             # a1 = random.uniform(-1,1) # this example is to take a random float between -1 and 1.
    #             # t1 = random.uniform(20,25) # this example is to take a random float between 15 and 25.
                
    #             self.adapter.begin_gather()
    #             self.j1.set_value(str(self.j1)) # set value of a1 data item, format: str(float)
    #             self.j2.set_value(str(self.j2)) # set value of t1 data item, format: str(float)
    #             self.adapter.complete_gather()

    #             print("{} RANDOM VALUE a1={} mm/s^2".format(datetime.datetime.now(), a1)) # printing out datetime now and a1
    #             print("{} RANDOM VALUE t1={} °C".format(datetime.datetime.now(), t1)) # printin gout datetime now and t1 
    #             print(datetime.datetime.now(), "MTConnect data items gathering completed...\n") # printing out MTConnect data collection is done.

    #             time.sleep(2) # wait for 2 seconds

    #         except KeyboardInterrupt:
    #             print("Stopping MTConnect...")
    #             self.adapter.stop() # Stop adapter thread
    #             sys.exit() # Terminate Python

## ====================== MAIN ======================
if __name__ == "__main__":
    print("Starting up!")
    MTConnectAdapter('127.0.0.1', 7878) # Args: host ip, port number
