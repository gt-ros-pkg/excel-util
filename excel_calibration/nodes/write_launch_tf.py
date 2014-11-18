#!/usr/bin/python

import sys
import rospy

template = """
<launch>
  <node name="%s_tf" pkg="tf" type="static_transform_publisher"
        args="%f %f %f %f %f %f %f %s %s 100"/>
</launch>
"""

def main():
    rospy.init_node("write_launch_tf")
    cam_frame = sys.argv[1]
    setup = rospy.get_param("/excel_calib_srv/calibration_setup")
    world_frame = setup['world_frame']
    for cam in setup['cameras']:
        if cam['frame'][1:] != cam_frame:
            continue
        pose = cam['calibrated_cam']['pose']
        pos = pose['translation']
        rot = pose['rotation']
        launch_file = template % (cam_frame, pos['x'], pos['y'], pos['z'], 
                    rot['x'], rot['y'], rot['z'], rot['w'], world_frame, cam_frame)
        print launch_file

if __name__ == "__main__":
    main()
