#!/usr/bin/python

import rospy
from excel_hw_iface.excel_interface import ExcelInterface

def main():
    rospy.init_node("joint_pose_rec")
    excel = ExcelInterface()
    while not rospy.is_shutdown():
        print "- [" + ", ".join(["%.2f" % qi for qi in excel.get_q()]) + "]"
        if raw_input("q to quit") == 'q':
            break

if __name__ == "__main__":
    main()
