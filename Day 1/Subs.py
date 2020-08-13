import rospy
from sensor_msgs.msg import JointState
import time
from std_msgs.msg import String
from robotis_controller_msgs.msg import SyncWriteItem

def btn_callback(data):
    if data.data == 'mode' :
        ledRGB(0,16,0)
        enable_module('direct_control_module')
        head(1,1)
    elif data.data == 'start' :
        ledRGB(0,16,16)
        init_pose()
    elif data.data == 'user' :
        ledRGB(16,16,16)
        for x in range(1, 5):
            enable_module('direct_control_module')
            berdiri()
            time.sleep(5)
            init_pose()
            time.sleep(5)

def init():
    rospy.init_node('coba1', anonymous=False)
    pub_direct = rospy.Publisher('/robotis/direct_control/set_joint_states', JointState, queue_size = 10)
    pub_module = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size = 1)
    pub_module = rospy.Publisher('/robotis/base/ini_pose', String, queue_size=1)
    rospy.Subscriber('/robotis/open_cr/button', String, btn_callback)
    time.sleep(1)

def head(pan, tilt):
    data = JointState()
    data.name = ['head_pan', 'head_tilt']
    data.position = [pan,tilt]
    pub_direct = rospy.Publisher('/robotis/direct_control/set_joint_states', JointState, queue_size = 10)
    pub_direct.publish(data)

def init_pose():
    data = String()
    data.data = 'ini_pose'
    pub_module = rospy.Publisher('/robotis/base/ini_pose', String, queue_size = 1)
    pub_module.publish(data)
    time.sleep(0.01)

def berdiri():
    data = JointState()
    data.name = ['r_sho_roll', 'r_sho_pitch','r_el', 'l_sho_roll', 'l_sho_pitch', 'l_el', 'r_hip_pitch', 'r_knee', 'r_ank_pitch', 'l_hip_pitch', 'l_knee', 'l_ank_pitch']
    data.position = [0,0,0,0,0,0 ,0,0,0,0,0,0]
    pub_direct = rospy.Publisher('/robotis/direct_control/set_joint_states', JointState, queue_size=10)
    pub_direct.publish(data)

def enable_module(module):
    data = String()
    data.data = module
    pub_module = rospy.Publisher('/robotis/enable_ctrl_module', String, queue_size = 1)
    pub_module.publish(data)
    time.sleep(0.01)

def ledRGB(r, g, b):
    syncWritePub = rospy.Publisher("/robotis/sync_write_item", SyncWriteItem, queue_size=1)
    ledMsg = SyncWriteItem()
    ledMsg.item_name = 'LED_RGB'
    ledMsg.joint_name = ['open-cr']
    ledMsg.value = [r << 0 | g << 5 | b << 10]
    syncWritePub.publish(ledMsg)
    time.sleep(0.01)

if __name__ == '__main__':
    try:
        init()
        enable_module('direct_control_module')
        rospy.spin()

        # init_pose()
    except rospy.ROSInterruptException:
        pass
	
