#!/bin/bash
sleep 1

v4l2-ctl -d /dev/video0 --set-ctrl white_balance_auto_preset=3
v4l2-ctl -d /dev/video0 --set-ctrl scene_mode=11
v4l2-ctl -d /dev/video0 --set-ctrl contrast=0
v4l2-ctl -d /dev/video0 --set-ctrl saturation=0