./build/bin/arm_foc

ros2 topic echo /detected_objects_3

ros2 launch realsense2_camera rs_align_depth_launch.py   enable_gyro:=false   enable_accel:=false   unite_imu_method:=0   depth_module.depth_profile:=640x360x30   rgb_camera.color_profile:=640x360x30

python3 detected_objects_3d/detect_3d1.py

sudo nmcli device wifi connect "gogogo" password "1346792580" ifname wlP1p1s0
