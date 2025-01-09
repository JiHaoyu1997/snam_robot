#!/usr/bin/env python3

import rospy
import subprocess

def setup_v4l2():
    commands = [
        "v4l2-ctl -d /dev/video0 --set-ctrl saturation=0",
        "v4l2-ctl -d /dev/video0 --set-ctrl contrast=50",
        "v4l2-ctl -d /dev/video0 --set-ctrl white_balance_auto_preset=7",
        "v4l2-ctl -d /dev/video0 --set-ctrl scene_mode=11",
    ]

    for cmd in commands:
        result = subprocess.run(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        # rospy.loginfo(f"Command: {cmd}")
        # rospy.loginfo(f"Output: {result.stdout.decode('utf-8')}")
        if result.stderr:
            rospy.logwarn(f"Error: {result.stderr.decode('utf-8')}")

if __name__ == "__main__":
    rospy.init_node("v4l2_setup_node")
    # rospy.loginfo("Setting up V4L2 parameters...")
    setup_v4l2()
    rospy.loginfo("V4L2 setup complete.")