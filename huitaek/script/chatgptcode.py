import rospy
from sensor_msgs.msg import JointState
from pymtconnect import MTConnectDevice, DataItem

# Create an MTConnect device with a data item for each joint state
device = MTConnectDevice('my_robot')
for i in range(6):
    device.add_data_item(DataItem(f'joint_{i}_position', category='SAMPLE', type='POSITION'))

# Define a ROS callback function to receive joint state messages
def joint_state_callback(msg):
    # Parse the ROS message and update the MTConnect data items
    for i, position in enumerate(msg.position):
        device['joint_{i}_position'].value = position

# Subscribe to the ROS joint state topic and set the callback function
rospy.Subscriber('/joint_states', JointState, joint_state_callback)

# Start the ROS node and run the MTConnect device server
rospy.init_node('ros_to_mtconnect')
device.serve_forever()
