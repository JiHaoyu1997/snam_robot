#!/bin/bash
sleep 1

v4l2-ctl -d /dev/video0 --set-ctrl white_balance_tauto_preset=7
v4l2-ctl -d /dev/video0 --set-ctrl scene_mode=11
v4l2-ctl -d /dev/video0 --set-ctrl contrast=50