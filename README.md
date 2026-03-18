
### 3. 运行方式

#### A. 环境准备

确保你的 ROS2 Humble 环境已经加载，并且底盘导航程序正在运行：

```bash
source /opt/ros/humble/setup.bash
# 启动你的 Nav2 导航节点
# ros2 launch your_mobile_base_nav_launch.py

```

#### B. 编译

```bash
mkdir build && cd build
cmake ..
make -j$(nproc)

```

#### C. 运行

```bash
./bin/arm_foc

```

#### D. 触发机制

1. 程序启动后，机械臂会 Homing 并进入等待状态。
2. 你可以在 **RViz2** 中通过 "Nav2 Goal" 工具给移动平台下达一个目标点。
3. 当底盘到达目标点，Nav2 系统会发布 `/navigate_to_pose/_action/status`。
4. 机械臂检测到 `status=4` (Success) 后，会立即启动 `VisionThread` 中的逻辑进行抓取。
5. 抓取完成后，机械臂回到 Home 位，等待下一次导航任务。

---

### 补充建议：

* **去抖动处理**：在检测到 `g_nav_finished` 为 true 时，我加了 1 秒的 `sleep`。这是因为移动平台刚停下时，机械臂末端的相机会有惯性晃动，延迟一秒可以提高视觉识别的精度。
* **安全性**：如果导航点位置选择不当，导致识别物体在机械臂范围之外，`MoveL` 可能会因为逆解失败而报警。建议在 `MoveL` 内部加入对逆解返回值的布尔检查。

Would you like me to refine the vision processing logic to handle multiple objects, or is the current single-target approach sufficient?