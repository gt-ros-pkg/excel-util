#!/usr/bin/python

import rospy
from std_msgs.msg import Bool

from excel_hw_iface.excel_interface import ExcelInterface

class EStopMonitor(object):
    def __init__(self):
        self.excel = ExcelInterface()
        self.estop_sub = rospy.Subscriber('/mode_state_pub/is_emergency_stopped', Bool, 
                                          self.estop_cb)
        self.sstop_sub = rospy.Subscriber('/mode_state_pub/is_security_stopped', Bool, 
                                          self.sstop_cb)

    def estop_cb(self, msg):
        if msg.data:
            self.excel.rail.halt_drive()
        else:
            self.excel.rail.enable_drive()

    def sstop_cb(self, msg):
        if msg.data:
            self.excel.rail.halt_drive()
        else:
            self.excel.rail.enable_drive()

def main():
    rospy.init_node("estop_monitor")
    estop_mon = EStopMonitor()
    rospy.spin()

if __name__ == "__main__":
    main()
