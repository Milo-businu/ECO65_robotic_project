# arm_grasp —— 真机整机抓取包

> 原来这个包是空壳（只有 `setup.py` / `package.xml`）。2026-11-03 实装完成：
> 把 **睿尔曼 ECO65 机械臂 + 轮趣 MS42DC 二指夹爪 + RealSense D435i + OpenPI VLA**
> 串成一条能跑通的抓取任务，并且可以从云服务器整体搬到真机。
> 同日真机侧新增了夹爪网页控制（`gripper_web.py`），已合并进本包。

---

## 一句话

**换机器只改 `config/real_params.yaml` 一个文件；代码零硬编码。**
默认 `dry-run`（不碰硬件），接电前用 `preflight.py` 就能把整条链路跑一遍验完。

---

## 目录

```
arm_grasp/
├── config/
│   ├── real_params.yaml      ★ 唯一配置文件（IP/话题/标定结果/夹爪参数/两道锁）
│   └── eco65_6fb.urdf        随包自带的运动学模型（仿真验证过，clone 即可用）
├── launch/
│   ├── d435i_eco65.launch.py 相机 + 手眼静态 TF（话题前缀从配置自动反推）
│   └── real_grasp.launch.py  整机一键启动（默认不自动抓，安全）
├── scripts/
│   ├── common.py             配置加载 / 路径解析 / 手眼矩阵
│   ├── ms42dc_driver.py      夹爪串口协议（BCC/帧/反馈/开口映射）+ 离线自测
│   ├── gripper_node.py       夹爪 ROS2 节点（open/close/home/grip:<mm>）
│   ├── gripper_web.py        ★ 夹爪网页控制（HTTP 页面 + rclpy 桥接）
│   ├── calibrate_gripper.py  开口 mm ↔ 电机角度 实机标定（先跑 --verify 回读校验）
│   ├── arm_backend.py        ★ 硬件抽象层：movej / action / dry-run 三后端
│   ├── planner.py            纯 numpy 的 IK 路径规划（抓取直线下降/抬升 + 放置三段转运）
│   ├── perception_node.py    D435i 彩色+对齐深度 → 目标 3D 位姿
│   ├── vla_policy.py         OpenPI π0 HTTP 桥接（可选高层策略）
│   ├── net_autoconf.py       ★ 机械臂 arm_ip / 本机 udp_ip 自动配对（/joint_states 没数先跑它）
│   ├── perception_mode.py    ★ 一键切 color / brain，并检查"配了没生效"
│   ├── brain.py              ★ 端到端识别大脑客户端（HTTP，Ollama/LM Studio/vLLM）
│   ├── verify_handeye.py     ★ 手眼物理打点验证（编码器当基准）+ 常数偏置求解
│   ├── teach_place_point.py  ★ 示教放置点（指尖碰台面 → place.xy / surface_z）
│   ├── grasp_task.py         ★ 抓取+放置总控状态机（8 或 11 阶段，两道安全锁）
│   └── preflight.py          ★ 移植自检（不接硬件，全链路验证）
├── package.xml / setup.py
└── README.md                 本文件
```

---

## 三条命令

```bash
cd ~/robot_vla_ws

# ① 软件侧准备（装依赖 / udev / 编译 / 自检），只跑一次
bash restore_real.sh

# ② 演练：不接任何硬件，把配置→FK→IK→夹爪协议→状态机全验一遍
python3 robot_code/arm_grasp/scripts/preflight.py

# ③ 接好硬件、做完两项标定后，起整机（会自动 source，不用记两行）
bash ~/robot_vla_ws/start_link.sh
```

### ⚠️ 仓库里的配置目前是「真机版」

真机把 `config/real_params.yaml` 的 `profile.mode` 改成了 `real`（并填了本机 IP / 标定值），
所以**云服务器或新 clone 的机器**跑演练时要显式加 `--dry-run`：

```bash
python3 robot_code/arm_grasp/scripts/grasp_task.py --dry-run
```

`preflight.py` 内部强制演练，不受影响。
真机模式仍需要 `profile.mode: real` **且** `allow_arm_motion: true` 两道锁同时打开。

> 💡 想让不同机器各存各的配置、互不打架：把机器专属值另存为
> `config/real_params.local.yaml`（可加入 `.gitignore`），每台机器自己维护，
> 仓库里的 `real_params.yaml` 只留通用模板。

### 夹爪网页控制（真机新增）

```bash
# 方式 A：随整机一起起
ros2 launch arm_grasp real_grasp.launch.py start_web:=true
# 浏览器打开 http://127.0.0.1:8777

# 方式 B：单独起两个终端
python3 robot_code/arm_grasp/scripts/gripper_node.py
python3 robot_code/arm_grasp/scripts/gripper_web.py --host 127.0.0.1 --listen-port 8777
```

launch 还提供 `web_host:=0.0.0.0`、`web_port:=<端口>` 两个参数（默认 `127.0.0.1` / `8777`）。
⚠️ 绑定 `0.0.0.0` 会让同一局域网的人都能操作夹爪，只在调试时用，别长期开着。

### 感知用颜色还是用识别大模型？一条命令切

```bash
python3 robot_code/arm_grasp/scripts/perception_mode.py            # 看当前 + 一致性检查
python3 robot_code/arm_grasp/scripts/perception_mode.py color      # 主线（默认，最稳）
python3 robot_code/arm_grasp/scripts/perception_mode.py brain --target "green cube"
```

两种模式只差"哪几个像素是目标"：color 用颜色阈值，brain 用 Qwen2.5-VL 给 bbox；
后面的对齐深度、手眼、IK、夹持、放置**完全同一套**。
brain 掉线会自动退回 color（`fallback_to_color: true`）。
详细操作见 **35 号手册**（`docs/机械臂整机开发文档/35_感知两种模式操作手册_color与brain.md`）。

> 现场对准阈值：`tune_perception.py --color green --write-config`
> 窗口里按 `s` 会**直接写进** `real_params.yaml` 的 `color_thresholds`（自动备份）。

### 报 `Package 'arm_grasp' not found`？用 start_link.sh

```
Package 'arm_grasp' not found: searching: ['/opt/ros/foxy']
```

这不是包坏了 —— 是**这个终端没 source 工作空间**（`ros2 launch` 只认已 source 的包）。
项目铁律又不允许把 source 写进 `~/.bashrc`，所以封了个脚本：

```bash
bash ~/robot_vla_ws/start_link.sh                 # 只起链路
bash ~/robot_vla_ws/start_link.sh start_task:=true
```

它内部做两件事：source（foxy + 工作空间）、判断包有没有编译过
（没编过就用绝对路径跑源码，纯 Python 包功能一样）。

### `/joint_states` 一条都没有？先跑 net_autoconf

```bash
python3 robot_code/arm_grasp/scripts/net_autoconf.py
```

`rm_driver` 是把关节角 **UDP 主动发**到配置里的 `udp_ip` 的。这个 IP 填错，
包就全发到空气里 —— 现象是"感知收不到关节角"，报错完全不提 IP。
脚本会自动挑本机在机械臂网段的 IP，同时改 `rm_driver/config/rm_eco65_config.yaml`
和 `real_params.yaml`（含 install 里的实体副本），并 ping 机械臂、探 8080。

### 视觉没收敛，也要把任务跑完（兜底路径）

```yaml
task:
  fixed_target_xyz: [0.3621, 0.0143, 0.0415]   # 示教出来的方块顶面中心
```

填了它，状态机就不再等感知，直接用这个点做 IK → 下降 → 夹持 → 抬升 → 转运 → 释放。
**整条硬件链、坐标变换、IK、夹持、放置全都真的在跑**，只是"方块在哪"由人示教。
日志抬头会打印 `目标源=示教固定点`，不会假装是视觉抓的。

用 `teach_place_point.py` 读坐标：把**指尖**移到方块顶面中心，回车即可。

### 手眼标定怎么验（别用尺子硬量）

```bash
python3 robot_code/arm_grasp/scripts/verify_handeye.py --points 4 --marker-size 0.047
```

脚本把指尖开到"相机算出来的标记中心"上方 2cm，你用键盘微调（w/s/a/d，1mm）把指尖挪到正好压住标记中心。
它读那一刻的关节角反算指尖真实位置（**不经过相机**），得到误差。
4 个点误差一致 → 输出 `perception.xy_bias_m` 补偿掉；忽大忽小 → 是旋转/尺度问题，必须重标。

### 放置（抓完放到固定区域）
抓取只解决"拿到手"，放置才是终点。放置点和抓取最大的不同：**它不靠视觉**，
是示教出来的固定坐标（量一次长期用）。

```bash
# 示教：指尖移到放置区中心、刚好碰到台面 → 回车 → 抄出 xy / surface_z
python3 robot_code/arm_grasp/scripts/teach_place_point.py
```

填进配置后，状态机从 8 阶段变 11 阶段：

```
…抬升 → [9] 转运（竖直抬 → 水平移 → 竖直降，全程笛卡尔）
       → [10] 张开释放（顺带比对开口变化，判断方块真脱手了没）
       → [11] 撤离 →（可选）回 home
```

```yaml
place:
  enabled: true
  xy: [0.3215, -0.1987]     # 基座系，示教读出
  surface_z: 0.0213         # 台面高度
  release_gap_m: 0.008      # 方块底面离台面 8mm 时张嘴
```

`preflight.py` 会自动检查放置点可达性、路径连续性、限位余量——
IK 无解会直接 ❌（实测 `xy=[0.32,-0.20]` 时 z 上限约 0.20m，别把 `retreat_height_m` 设太大）。
详见 30 号手册 **10.1 节**。

### ⚠️ 编译顺序：`arm_grasp` 之前必须先编三个 C++ 包

新 clone 的工作空间里 `ros2_rm_robot` 的 C++ 包还没编过时，直接
`colcon build --packages-select arm_grasp` 会报
`Failed to find .../install/rm_driver/share/rm_driver/package.sh`
——`package.sh` 是包编译后才生成的「环境钩子」，找不到就是没编过。

`restore_real.sh` 已自动检测并补编。手动补的话：

```bash
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash
colcon build --packages-select rm_ros_interfaces rm_driver rm_control --symlink-install
colcon build --packages-select arm_grasp --symlink-install
```

`package.xml` **故意不声明**这三个包为 `exec_depend`：它们是运行期提供话题的
C++ 节点，本包是纯 Python、构建期不 import；列进去会让 colcon 在构建期
错误地要求它们先编译完（2026-11-03 真机实测踩到）。

### 完全不编译也能跑（降级路径）

纯 Python 包，`preflight.py` / dry-run / 整机 launch 都不需要它先编译：
用**绝对路径**起 launch 即可（不写包名，就不需要 `install/share`）：

```bash
python3 ~/robot_vla_ws/robot_code/arm_grasp/scripts/preflight.py
ros2 launch ~/robot_vla_ws/robot_code/arm_grasp/launch/real_grasp.launch.py start_arm:=false
```

`start_arm:=false` 时 `rm_driver` 没编译也只是告警跳过，相机/夹爪/感知照常起，
可先验证话题链路。**但真要动机械臂，那三个包必须编好。**

---

## 架构（为什么要有"硬件抽象层"）

| 层 | 仿真 | 真机 | 由谁抹平 |
|---|---|---|---|
| 机械臂运动 | `FollowJointTrajectory` action | `/rm_driver/movej_cmd` 话题 | `arm_backend.py` |
| 关节角反馈 | Gazebo `/joint_states` | rm_driver **UDP 主动上报** `/joint_states` | `arm_backend.py` |
| 相机 | Gazebo 仿真相机 | D435i `realsense2_camera` | `perception_node.py` + 配置 |
| 相机姿态 | 写死的 `CAM_ROT` | **手眼标定 X**（4×4） | `perception_node.py` |
| 夹爪 | effort 力矩（仿真才有） | 串口位置模式 + **堵转** | `gripper_node.py` |
| 夹持力 | 力矩 0.030 N·m | 目标开口略小于物体 → 顶到停 | `interference_mm` |
| 放置点 | 仿真里写死坐标 | 示教读出写进 yaml（不靠视觉） | `teach_place_point.py` |

⚠️ **别把仿真脚本直接搬到真机**：
`rm_control` 的 action 桥接要求轨迹点带 `velocities` / `accelerations` 且点数 > 3，
仿真的 `eff_test.py` 只填了 `positions`，真机会越界崩溃。
本项目默认 `backend: movej`，绕开这个坑；`ActionBackend` 里也做了补齐。

---

## 节点接口（都用标准消息，零自定义 msg）

| 方向 | 话题 | 类型 | 内容 |
|---|---|---|---|
| 订阅 | `/gripper/cmd` | `std_msgs/String` | `open` / `close` / `home` / `grip:<mm>` / `query` / `stop` |
| 订阅 | `/gripper/cmd_width` | `std_msgs/Float64` | 目标开口 mm |
| 发布 | `/gripper/state` | `Float64MultiArray` | `[开口mm, 累计角度°, 到位, 堵转]`（开口未知 = -1） |
| 发布 | `/grasp/target_xyz` | `Float64MultiArray` | `[x,y,z]` 目标顶面中心（base_link 系） |
| 发布 | `/grasp/target_xyz_pose` | `PoseStamped` | 同上，给 RViz 看 |
| 订阅 | `/rm_driver/movej_cmd` | `rm_ros_interfaces/Movej` | 关节目标 |
| 订阅 | `/rm_driver/movej_result` | `std_msgs/Bool` | 到位/失败 |
| 订阅 | `/joint_states` | `JointState` | 当前关节角（**禁止缓存**） |

网页控制 `gripper_web.py` 不新增 ROS 话题，它只是把 `/gripper/cmd` 与
`/gripper/state` 用 HTTP 页面包了一层（浏览器 ↔ HTTP ↔ rclpy 节点）。

---

## 两道安全锁（防止误动真机）

`real_params.yaml` 里必须**同时**满足才会下发机械臂运动指令：

```yaml
profile:
  mode: real                 # 第一道：不是 dry-run
  allow_arm_motion: true     # 第二道：显式确认
```

另外还有两道"数据锁"：
- `handeye.calibrated: false` → 感知节点拒绝发布目标位姿
- `gripper.calibration_done: false` → 总控打警告（开口只是粗估）

---

## 完整手册

- `docs/机械臂整机开发文档/30_云服务器到真机一键移植手册.md` —— 照着做就能跑通
- `docs/机械臂整机开发文档/32_真机与云机双机Git协作流程.md` —— 双机改代码 / 提交 / 合并的流程与报错速查

（30 / 32 号在 work1 根目录也各放了一份）
