#! /bin/python

import rospy
from std_msgs.msg import UInt16

class DriveStatusControl(object):
    ENABLED = 'Drive Enable (AF)'
    READY = 'Drive Ready (Ab)'
    HALTED = 'Drive Halt (AH)'
    UNKNOWN = 'Drive Status Unknown'

    def __init__(self, topic_prefix='', timeout=5.):
        self._cur_status = None
        self._command_pub = rospy.Publisher(topic_prefix+'master_ctrl_cmd', UInt16)
        self._status_sub = rospy.Subscriber(topic_prefix+'drive_status', UInt16, self._status_cb)
        self.wait_for_status(timeout)

    def _status_cb(self, msg):
        self._cur_status = msg.data

    def wait_for_status(self, timeout=5.):
        if timeout <= 0.:
            return self._cur_status is not None
        start_time = rospy.Time.now()
        r = rospy.Rate(100.)
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < timeout:
            if self._cur_status is not None:
                return True
            r.sleep()
        rospy.logwarn('Timed out waiting for drive_status')
        return False

    def get_drive_mode(self):
        if (self._cur_status & 0xE008) == 0xC008:
            return DriveStatusControl.ENABLED
        elif (self._cur_status & 0xE008) == 0xC000:
            return DriveStatusControl.HALTED
        elif (self._cur_status & 0xE008) == 0x8000:
            return DriveStatusControl.READY
        else:
            return DriveStatusControl.UNKNOWN

    def is_drive_ready(self):
        return self.get_drive_mode() is DriveStatusControl.READY

    def is_drive_enabled(self):
        return self.get_drive_mode() is DriveStatusControl.ENABLED

    def is_drive_halted(self):
        return self.get_drive_mode() is DriveStatusControl.HALTED

    def enable_drive(self, timeout=5.):
        if self.is_drive_enabled():
            return True
        elif not self.is_drive_ready() and not self.is_drive_halted():
            return False
        # drive is ready, let's enable it
        self._command_pub.publish(UInt16(0xE000))
        start_time = rospy.Time.now()
        r = rospy.Rate(100.)
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.is_drive_enabled():
                return True
            r.sleep()
        rospy.logwarn('Timed out waiting for drive enabled confirmation')
        return False

    def halt_drive(self, timeout=5.):
        if self.is_drive_halted():
            return True
        elif not self.is_drive_enabled():
            return False
        # drive is enabled, let's halt it
        self._command_pub.publish(UInt16(0xC000))
        start_time = rospy.Time.now()
        r = rospy.Rate(100.)
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.is_drive_halted():
                return True
            self._command_pub.publish(UInt16(0xC000)) # this is important, keep sending
            r.sleep()
        rospy.logwarn('Timed out waiting for drive halted confirmation')
        return False

    def disable_drive(self, timeout=5., best_pos_deccel=False):
        if self.is_drive_ready():
            return True
        if not best_pos_deccel:
            cmd_msg = UInt16(0x0000)
        else:
            cmd_msg = UInt16(0x4000)
        self._command_pub.publish(cmd_msg)
        start_time = rospy.Time.now()
        r = rospy.Rate(100.)
        while not rospy.is_shutdown() and (rospy.Time.now() - start_time).to_sec() < timeout:
            if self.is_drive_ready():
                return True
            self._command_pub.publish(cmd_msg) # this is important, keep sending
            r.sleep()
        rospy.logwarn('Timed out waiting for drive halted confirmation')
        return False
