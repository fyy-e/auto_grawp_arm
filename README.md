# dm_arm_end — 达妙机械臂 GRCNN 视觉抓取

达妙 6 轴机械臂 + Intel D435i 眼在手上相机，使用 GR-ConvNet（GRCNN）识别
物体的**位置 + 抓取角度 + 夹爪开口宽度**，控制机械臂完成抓取。
视觉后端已由原 YOLO 方案替换为 GRCNN（skumra/robotic-grasping）。

运行平台：Jetson Orin Nano（JetPack 6 / Ubuntu 22.04 / ROS2 Humble）

## 目录结构

```
dm_arm_end/
├── src/
│   ├── main.cpp                      # 全局唯一主入口（任务逻辑、抓取序列）
│   ├── algorithms/
│   │   ├── VisionDetector.*          # UDP 接收视觉结果（20 字节协议，端口 5005）
│   │   └── VisionGraspPlanner.*      # 手眼变换 + 抓取参数映射
│   └── config/handeye_result_realsense.yaml   # 手眼标定矩阵
├── vision/                           # GRCNN 视觉子系统（Python，自包含）
│   ├── grcnn_ros_node.py             # ROS2 节点
│   ├── grcnn_server.py             
│   ├── grasp_tracker.py            
│   ├── models/                       # GRCNN 预训练模型
│   └── test_data/                    # 离线自检图片
├── docs/GRCNN改造说明.md
└── build/bin/arm_foc                 # 编译产物：机械臂主控程序
```

## 使用方法

### 编译

```bash
cd ~/dm_arm_end/build && make -j$(nproc)
```

### 启动（三个终端，按顺序）

**终端 1 — 相机驱动：**

```bash
source /opt/ros/humble/setup.bash
ros2 launch realsense2_camera rs_align_depth_launch.py \
    enable_gyro:=false enable_accel:=false unite_imu_method:=0 \
    depth_module.depth_profile:=640x360x30 rgb_camera.color_profile:=640x360x30
```

**终端 2 — GRCNN 视觉节点（带实时画面窗口）：**

```bash
source /opt/ros/humble/setup.bash
cd ~/dm_arm_end && python3 vision/grcnn_ros_node.py
```

**终端 3 — 机械臂主控：**

```bash
cd ~/dm_arm_end && ./build/bin/arm_foc
```

识别成功标志：画面中出现绿色抓取框（半透明填充 + 黄色夹爪边）贴在物体上，
终端打印 `cam=(x,y,z) angle=... width=...`。

&gt; 不用 ROS 的替代方式：跳过终端 1，终端 2 改为
&gt; `python3 vision/grcnn_server.py --visualize`（两种方式**不能同时开**，相机独占）。

## WAIT_DOG_NAV：是否等待导航信号

开关在 `src/main.cpp` 第 24 行，改完**重新编译**生效：

```cpp
#define WAIT_DOG_NAV 1   // 1 = 等待模式：收到狗的导航到达信号后才开始识别抓取
                         // 0 = 独立模式：不等待，一直停在 Resting 位循环识别抓取
```

| 值    | 行为                                                                                                                                                |
| ----- | --------------------------------------------------------------------------------------------------------------------------------------------------- |
| `1` | 机械臂待机，直到 UDP 端口**12346** 收到字符串 `NAV_REACHED_SUCCESS` → 移动到 Resting 位 → 识别抓取一次 → 返回 Home，继续等下一次导航信号 |
| `0` | 上电后直接停在 Resting 位，识别到物体就抓，抓完回 Resting 位继续等下一个目标（调试/演示用）                                                         |

## 关键参数速查

| 参数                             | 位置                                      | 作用                                             |
| -------------------------------- | ----------------------------------------- | ------------------------------------------------ |
| `manual_offset`                | `src/algorithms/VisionGraspPlanner.cpp` | 基座系固定偏差补偿（实测调）                     |
| `target_base.z() -= width*0.5` | `src/main.cpp`                          | 抓取点下沉半个物体厚度，夹到物体中部             |
| `YAW_OFFSET`                   | `src/main.cpp` 第 27 行                 | 抓取方向整体差 90°/180° 时改这里               |
| `angle_sign / angle_offset`    | `VisionGraspPlanner.h`                  | 抓取角方向/零位（夹爪与物体长轴不垂直时改 sign） |
| `width_k / width_b`            | `VisionGraspPlanner.h`                  | 夹爪 宽度→电机角 映射（两点实测标定）           |
| UDP 5005                         | VisionDetector / vision 脚本              | 视觉→控制 数据通道，两端必须一致                |

## 注意事项

- 相机被独占：ROS 驱动、`grcnn_server.py`、旧 YOLO 节点（`detect_3d1.py`）
  三者同一时间只能跑一个；旧 YOLO 节点还会占用 UDP 5005 造成冲突。
- 弹窗显示需在 Jetson 桌面已解锁，并使用：
  `DISPLAY=:1 XAUTHORITY=/run/user/1000/gdm/Xauthority`
- 检测最佳工作距离：相机距桌面 0.4 ~ 0.7 m。
- 物体需位于画面中央黄色虚线框（300×300 网络视野）内才能被识别。
