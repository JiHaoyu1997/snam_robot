# snam_robot
Distributed Control At Signal Free Intersection - Robot

[ INFO] [1730126530.245563590]: camera calibration URL: file:///home/vpaadmin/snam_robot/src/vpa_robot_interface/config/camera_default.yaml
[ INFO] [1730126530.257238982]: Starting 'head_camera' (/dev/video0) at 320x240 via mmap (yuyv) at 10 FPS
[ WARN] [1730126530.725475156]: unknown control 'white_balance_temperature_auto'

[ WARN] [1730126530.769471213]: unknown control 'focus_auto'

# camera

## v4l2-ctl  --list-ctrls

User Controls

                     brightness 0x00980900 (int)    : min=0 max=100 step=1 default=50 value=50 flags=slider
                       contrast 0x00980901 (int)    : min=-100 max=100 step=1 default=0 value=50 flags=slider
                     saturation 0x00980902 (int)    : min=-100 max=100 step=1 default=0 value=0 flags=slider
                    red_balance 0x0098090e (int)    : min=1 max=7999 step=1 default=1000 value=1000 flags=slider
                   blue_balance 0x0098090f (int)    : min=1 max=7999 step=1 default=1000 value=1000 flags=slider
                horizontal_flip 0x00980914 (bool)   : default=0 value=0
                  vertical_flip 0x00980915 (bool)   : default=0 value=0
           power_line_frequency 0x00980918 (menu)   : min=0 max=3 default=1 value=1
                      sharpness 0x0098091b (int)    : min=-100 max=100 step=1 default=0 value=0 flags=slider
                  color_effects 0x0098091f (menu)   : min=0 max=15 default=0 value=0
                         rotate 0x00980922 (int)    : min=0 max=360 step=90 default=0 value=0 flags=modify-layout
             color_effects_cbcr 0x0098092a (int)    : min=0 max=65535 step=1 default=32896 value=32896

Codec Controls

             video_bitrate_mode 0x009909ce (menu)   : min=0 max=1 default=0 value=0 flags=update
                  video_bitrate 0x009909cf (int)    : min=25000 max=25000000 step=25000 default=10000000 value=10000000
         repeat_sequence_header 0x009909e2 (bool)   : default=0 value=0
            h264_i_frame_period 0x00990a66 (int)    : min=0 max=2147483647 step=1 default=60 value=60
                     h264_level 0x00990a67 (menu)   : min=0 max=11 default=11 value=11
                   h264_profile 0x00990a6b (menu)   : min=0 max=4 default=4 value=4

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=0 value=0
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=10000 step=1 default=1000 value=1000
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=0
             auto_exposure_bias 0x009a0913 (intmenu): min=0 max=24 default=12 value=12
      white_balance_auto_preset 0x009a0914 (menu)   : min=0 max=10 default=1 value=7
            image_stabilization 0x009a0916 (bool)   : default=0 value=0
                iso_sensitivity 0x009a0917 (intmenu): min=0 max=4 default=0 value=0
           iso_sensitivity_auto 0x009a0918 (menu)   : min=0 max=1 default=1 value=1
         exposure_metering_mode 0x009a0919 (menu)   : min=0 max=2 default=0 value=0
                     scene_mode 0x009a091a (menu)   : min=0 max=13 default=0 value=11

JPEG Compression Controls

            compression_quality 0x009d0903 (int)    : min=1 max=100 step=1 default=30 value=30

## v4l2-ctl  --device=/dev/video0 --all
Driver Info:
	Driver name      : bm2835 mmal
	Card type        : mmal service 16.1
	Bus info         : platform:bcm2835-v4l2
	Driver version   : 5.4.246
	Capabilities     : 0x85200005
		Video Capture
		Video Overlay
		Read/Write
		Streaming
		Extended Pix Format
		Device Capabilities
	Device Caps      : 0x05200005
		Video Capture
		Video Overlay
		Read/Write
		Streaming
		Extended Pix Format
Priority: 2
Video input : 0 (Camera 0: ok)
Format Video Capture:
	Width/Height      : 320/240
	Pixel Format      : 'YUYV' (YUYV 4:2:2)
	Field             : None
	Bytes per Line    : 640
	Size Image        : 153600
	Colorspace        : SMPTE 170M
	Transfer Function : Default (maps to Rec. 709)
	YCbCr/HSV Encoding: Default (maps to ITU-R 601)
	Quantization      : Default (maps to Limited Range)
	Flags             : 
Format Video Overlay:
	Left/Top    : 150/50
	Width/Height: 1024/768
	Field       : None
	Chroma Key  : 0x00000000
	Global Alpha: 0xff
	Clip Count  : 0
	Clip Bitmap : No
Framebuffer Format:
	Capability    : Extern Overlay
			Global Alpha
	Flags         : Overlay Matches Capture/Output Size
	Width         : 320
	Height        : 240
	Pixel Format  : 'YU12'
Streaming Parameters Video Capture:
	Capabilities     : timeperframe
	Frames per second: 10.000 (10/1)
	Read buffers     : 1

User Controls

                     brightness 0x00980900 (int)    : min=0 max=100 step=1 default=50 value=50 flags=slider
                       contrast 0x00980901 (int)    : min=-100 max=100 step=1 default=0 value=0 flags=slider
                     saturation 0x00980902 (int)    : min=-100 max=100 step=1 default=0 value=0 flags=slider
                    red_balance 0x0098090e (int)    : min=1 max=7999 step=1 default=1000 value=1000 flags=slider
                   blue_balance 0x0098090f (int)    : min=1 max=7999 step=1 default=1000 value=1000 flags=slider
                horizontal_flip 0x00980914 (bool)   : default=0 value=0
                  vertical_flip 0x00980915 (bool)   : default=0 value=0
           power_line_frequency 0x00980918 (menu)   : min=0 max=3 default=1 value=1
				0: Disabled
				1: 50 Hz
				2: 60 Hz
				3: Auto
                      sharpness 0x0098091b (int)    : min=-100 max=100 step=1 default=0 value=0 flags=slider
                  color_effects 0x0098091f (menu)   : min=0 max=15 default=0 value=0
				0: None
				1: Black & White
				2: Sepia
				3: Negative
				4: Emboss
				5: Sketch
				6: Sky Blue
				7: Grass Green
				8: Skin Whiten
				9: Vivid
				10: Aqua
				11: Art Freeze
				12: Silhouette
				13: Solarization
				14: Antique
				15: Set Cb/Cr
                         rotate 0x00980922 (int)    : min=0 max=360 step=90 default=0 value=0 flags=modify-layout
             color_effects_cbcr 0x0098092a (int)    : min=0 max=65535 step=1 default=32896 value=32896

Codec Controls

             video_bitrate_mode 0x009909ce (menu)   : min=0 max=1 default=0 value=0 flags=update
				0: Variable Bitrate
				1: Constant Bitrate
                  video_bitrate 0x009909cf (int)    : min=25000 max=25000000 step=25000 default=10000000 value=10000000
         repeat_sequence_header 0x009909e2 (bool)   : default=0 value=0
            h264_i_frame_period 0x00990a66 (int)    : min=0 max=2147483647 step=1 default=60 value=60
                     h264_level 0x00990a67 (menu)   : min=0 max=11 default=11 value=11
				0: 1
				1: 1b
				2: 1.1
				3: 1.2
				4: 1.3
				5: 2
				6: 2.1
				7: 2.2
				8: 3
				9: 3.1
				10: 3.2
				11: 4
                   h264_profile 0x00990a6b (menu)   : min=0 max=4 default=4 value=4
				0: Baseline
				1: Constrained Baseline
				2: Main
				4: High

Camera Controls

                  auto_exposure 0x009a0901 (menu)   : min=0 max=3 default=0 value=0
				0: Auto Mode
				1: Manual Mode
         exposure_time_absolute 0x009a0902 (int)    : min=1 max=10000 step=1 default=1000 value=1000
     exposure_dynamic_framerate 0x009a0903 (bool)   : default=0 value=0
             auto_exposure_bias 0x009a0913 (intmenu): min=0 max=24 default=12 value=12
				0: -4000 (0xfffffffffffff060)
				1: -3667 (0xfffffffffffff1ad)
				2: -3333 (0xfffffffffffff2fb)
				3: -3000 (0xfffffffffffff448)
				4: -2667 (0xfffffffffffff595)
				5: -2333 (0xfffffffffffff6e3)
				6: -2000 (0xfffffffffffff830)
				7: -1667 (0xfffffffffffff97d)
				8: -1333 (0xfffffffffffffacb)
				9: -1000 (0xfffffffffffffc18)
				10: -667 (0xfffffffffffffd65)
				11: -333 (0xfffffffffffffeb3)
				12: 0 (0x0)
				13: 333 (0x14d)
				14: 667 (0x29b)
				15: 1000 (0x3e8)
				16: 1333 (0x535)
				17: 1667 (0x683)
				18: 2000 (0x7d0)
				19: 2333 (0x91d)
				20: 2667 (0xa6b)
				21: 3000 (0xbb8)
				22: 3333 (0xd05)
				23: 3667 (0xe53)
				24: 4000 (0xfa0)
      white_balance_auto_preset 0x009a0914 (menu)   : min=0 max=10 default=1 value=1
				0: Manual
				1: Auto
				2: Incandescent
				3: Fluorescent
				4: Fluorescent H
				5: Horizon
				6: Daylight
				7: Flash
				8: Cloudy
				9: Shade
				10: Greyworld
            image_stabilization 0x009a0916 (bool)   : default=0 value=0
                iso_sensitivity 0x009a0917 (intmenu): min=0 max=4 default=0 value=0
				0: 0 (0x0)
				1: 100000 (0x186a0)
				2: 200000 (0x30d40)
				3: 400000 (0x61a80)
				4: 800000 (0xc3500)
           iso_sensitivity_auto 0x009a0918 (menu)   : min=0 max=1 default=1 value=1
				0: Manual
				1: Auto
         exposure_metering_mode 0x009a0919 (menu)   : min=0 max=2 default=0 value=0
				0: Average
				1: Center Weighted
				2: Spot
                     scene_mode 0x009a091a (menu)   : min=0 max=13 default=0 value=0
				0: None
				8: Night
				11: Sports

JPEG Compression Controls

            compression_quality 0x009d0903 (int)    : min=1 max=100 step=1 default=30 value=30