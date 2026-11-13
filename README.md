## 机械臂项目上手操作流程（Ubuntu 20.04 + ROS2 Foxy）

> 本文是「从零到把方块抓起来」的完整路径：拿代码 → 装环境 → 标定 → 自检 → 演练 → 真机抓取。
> 全程约 **2~4 小时**，其中标定占一半。**第一次务必按顺序做，不要跳步。**
>
> **三条安全线（每次动臂前都要确认）**：① 机械臂周围清空 ② 急停按钮在手边 ③ 速度 ≤ 20%（默认 10%）

> **仓库安全说明**：本仓库不含任何密码、密钥或访问令牌；提交的配置是**安全默认**
> （`mode: dry-run` + `allow_arm_motion: false`），克隆下来不会让机械臂动。
> 要真机运行必须先读第 5 步与第 11 步，并确认急停在手边。详见 [SECURITY.md](SECURITY.md)。
### 开始前，先确认这三件事

| 检查项 | 命令 | 期望 |
|---|---|---|
| 系统 | `lsb_release -a` | Ubuntu **20.04** |
| ROS2 | `ls /opt/ros/foxy/setup.bash` | 文件存在（报错就先去装 ROS2 Foxy） |
| 机械臂在线 | `ping 192.168.1.18` | 有回包（不通先查网线、机械臂是否上电） |

硬件清单：睿尔曼 ECO65-6FB 六轴臂、MS42DC 二指电动夹爪、Intel RealSense D435i（腕装）。

---

### 第 1 步 · 拿代码（目录名不能改）

```bash
git clone https://github.com/Milo-businu/cloud2_robot_eco65.git ~/robot_vla_ws
cd ~/robot_vla_ws
ls
```

应该看到 `robot_code/`、`docs/`、`aliases.sh`、`restore_real.sh`、`start_link.sh`。

> **为什么必须叫 `~/robot_vla_ws`**：启动文件和脚本里多处按这个绝对路径找文件。换个目录名，后面会遇到一堆"找不到 xxx"。

**顺便把快捷命令装上**（后面所有命令都有简称）：

```bash
echo 'source ~/robot_vla_ws/aliases.sh' >> ~/.bashrc
source ~/robot_vla_ws/aliases.sh
rhelp
```

`rhelp` 会打印全部别名，忘了命令随时敲它。**注意：`git pull` 之后已开着的终端不会自动重读别名**，要么 `source ~/robot_vla_ws/aliases.sh`，要么重开终端。

---

### 第 2 步 · 补一个软链（仓库里没有，少这步一定失败）

```bash
cd ~/robot_vla_ws
ln -s robot_code src
ls -ld src
```

应该显示 `src -> robot_code`。

> **为什么要这样**：`colcon`（ROS2 的编译工具）只认 `src/` 目录下的包，而本项目的代码都放在 `robot_code/`。
> `src` 是个软链，被 `.gitignore` 排除了，所以**新克隆下来的仓库里没有它**，必须手动建一次。
> 少了这一步，`colcon build` 会报"找不到任何包"。

---

### 第 3 步 · 装第三方依赖

```bash
# 相机驱动（apt 装最快，约 1 分钟）
sudo apt install -y ros-foxy-realsense2-camera
```

> 机械臂驱动 `ros2_rm_robot` **已经在仓库里**（`robot_code/ros2_rm_robot/`，783 个文件），不用另外下。
> 它是睿尔曼官方的 Foxy 版驱动，提供 `rm_ros_interfaces` / `rm_driver` / `rm_control` 三个 C++ 包。

---

### 第 4 步 · 一键准备（装依赖 + 配 udev + 编译 + 自检）

```bash
cd ~/robot_vla_ws
bash restore_real.sh
```

这个脚本是**幂等**的（跑几遍都一样），会依次做 5 件事：

| 步骤 | 做什么 | 失败了怎么看 |
|---|---|---|
| ① | 检查 Ubuntu 20.04 / `/opt/ros/foxy` / 配置文件是否齐全 | 报错就照提示装 ROS2 或重新 clone |
| ② | `apt install` Python 依赖（yaml / numpy / opencv / pyserial） | 网络问题换 apt 源 |
| ③ | 写 udev 规则，把夹爪固定成 `/dev/motor_serial`；把用户加进 `dialout` 组 | **加组要注销重登才生效** |
| ④ | 先编三个 C++ 包，再编 `arm_grasp` | 见下方说明 |
| ⑤ | 跑 `preflight.py`（不接硬件，全链路自检） | 期望**大部分 ✅、0 个 ❌**；⚠️ 多是待办（未标定等） |

**它明确不做这四件事**（只能人来）：

- ✗ 把夹爪拨码开关 2 号打到 ON
- ✗ 夹爪开口标定
- ✗ 手眼标定
- ✗ 把 `profile.mode` 改成 `real`

> **编译失败也别慌**：`arm_grasp` 是纯 Python 包，**不编译也能跑**，只是不能用 `ros2 launch arm_grasp ...` 这种按包名调用的写法。
> 编译失败时脚本会把"直接用绝对路径跑"的命令打出来。

---

### 第 5 步 · 安全锁：默认是「关」的

公开仓库里提交的配置是**安全默认**：

```yaml
profile:
  mode: dry-run              # 演练：只打印计划，不下发运动
  allow_arm_motion: false    # 二次开关
```

含义：**克隆下来之后，任何运动指令都不会真的执行**。你可以放心先跑自检、演练、看日志，机械臂一动不动。

要真正动臂时（第 11 步）再开锁，一条命令同时改这两项：

```bash
~/robot_vla_ws/robot_code/arm_grasp/scripts/arm_unlock.py on --speed 10
# 装过别名之后更短：
unlock on --speed 10
```

> ⚠️ **开锁 = 机械臂随时可能砸下来**。开之前三件事必须做到：工作区清空、急停按钮在手边、速度 ≤ 20%。
> 用完立刻上锁：`lock`（只改 `allow_arm_motion`，不动 `mode`，不影响只读脚本）。

> 想确认当前状态：直接跑 `arm_unlock.py`（不带参数），它会打印两道锁的值 + 速度。

### 第 6 步 · 改四个必须改的值

打开配置文件：`cfg`（等价于 `nano ~/robot_vla_ws/robot_code/arm_grasp/config/real_params.yaml`）。

| 配置项 | 默认值 | 你要改成 |
|---|---|---|
| `arm.arm_ip` | `192.168.1.18` | 机械臂实际 IP（出厂一般是这个，别乱改） |
| `arm.udp_ip` | `192.168.1.10` | **本机（电脑）的 IP** |
| `camera.prefix` | `/camera/d435i` | 一般不用改 |
| `gripper.port` | `/dev/motor_serial` | udev 配好后不用改；没有就用 `/dev/ttyUSB0` |

**`udp_ip` 为什么不改就一定失败**：机械臂驱动是**主动用 UDP 把关节角发出去**的，发往它配置里的那个 IP。这个 IP 填错（比如填了别人机器的），数据全发到空气里 → `/joint_states` 一条都没有 → 感知算不出坐标 → 整条链全废，而报错信息只会说"收不到关节角"，完全不指向这里。

先查本机 IP：

```bash
hostname -I
```

**更省事的做法**：用脚本一次改全（它同时改驱动配置和本配置文件两处）：

```bash
cd ~/robot_vla_ws
python3 robot_code/arm_grasp/scripts/net_autoconf.py --arm-ip 192.168.1.18 --udp-ip 192.168.1.10
# 或者让它自动找（会在机械臂网段扫控制端口）
python3 robot_code/arm_grasp/scripts/net_autoconf.py
```

> **一个很阴的坑**：如果电脑同时连着外网和机械臂网段，且**两块网卡都是 192.168.1.x**，UDP 回包会走错网卡，`/joint_states` 依然为空。查网卡：`ip addr`，把其中一块改到别的网段。

还有一项**只在换夹爪 / 重装夹爪时才改**，但错了会让下降位置整体偏：

```yaml
robot:
  tcp_offset: [0.0, 0.0, 0.153]
```

含义是「法兰面 → 两指尖连线中点」的偏移（沿工具 z 轴），**用尺子实量**。工厂装配值是 0.177，仓库里填的 0.153 是为了抵消一个 24 mm 的系统偏差（见 `docs/` 里 38 号问题册），**照抄可以，换硬件必须重测**。

---

### 第 7 步 · 硬件自检（不动臂，随时可跑）

**① 一键体检，给 GO / NO-GO**

```bash
hw                # 等价于 python3 robot_code/arm_grasp/scripts/check_real_hw.py
```

它会检查：机械臂能否 ping 通、`/joint_states` 有没有数、相机话题在不在、夹爪串口有没有、配置文件里锁的状态。

**② 夹爪的三个硬前提**（不满足则串口收不到任何数据）

- 供电 10~25V 接 XH2.54 4Pin
- USB-C 接电脑（**不支持热插拔**，插好再上电）
- ★ **拨码开关 2 号打到 ON**（1 号保持 OFF）

确认设备节点存在：

```bash
ls -l /dev/motor_serial
```

**③ 相机与关节角**

```bash
camhz     # 相机帧率，期望 ≈30Hz
jos       # 关节角在刷
```

相机话题名不对（比如你看到的是 `/d435i/...` 而不是 `/camera/d435i/...`），就改 `camera.prefix`：

```bash
rtl       # 列出所有相关话题，直接看真实名字
```

---

### 第 8 步 · 夹爪开口标定

**为什么必须做**：夹爪从角度到开口是**两段**——先有一段机械空程（丝杠间隙），过了空程才严格线性。空程值错了，预夹开口会静默偏小，表现是"抓取失败但不报错"。

**先看它怎么算，不动硬件**：

```bash
cg --demo
```

**再验一次，确认串口通**：

```bash
cg --verify
```

**正式标定**（按提示一步步来）：

```bash
cg
# 等价于 python3 robot_code/arm_grasp/scripts/calibrate_gripper.py --port /dev/motor_serial
```

标定完成后脚本会打印四行配置，贴回 `real_params.yaml` 的 `gripper` 段：

```yaml
gripper:
  max_width_mm: 130.0
  min_width_mm: 10.0
  deg_per_mm: 15.2022
  deadzone_deg: 12.2
```

**判据**：残差 **< 3 mm** 才算过；超过就别输出配置，重标。

> 真机实测：机械空程约 **105°**，之后 **15°/mm**。这两个数直接决定"预夹开口"和"夹持开口"，错了会夹空。

---

### 第 9 步 · 手眼标定

**先看仓库里有没有现成的**：

```bash
grep -nE "calibrated|marker_size" ~/robot_vla_ws/robot_code/arm_grasp/config/real_params.yaml
```

- `calibrated: true` 且 `matrix_4x4` 不是 `null` → **已经标过**，先按下面第 ④ 步验证，通过就不用重做
- `calibrated: false` → 必须做

**① 准备标定板**

现场用的是 **A4 纸上打印的两个 ArUco 标记**（不是棋盘格）。贴硬板、四边用胶带固定，`--marker-size` 必须用尺子**实量黑框外沿到外沿**（本项目是 5.95 cm）。

**② 采数据（15~20 个不同姿态）**

```bash
sweepeh    # 等价于 python3 recovery/collect_handeye.py --sweep
```

关键：**姿态要有差异**（俯仰、偏航都覆盖）。只平移不转姿态，标定方程会退化，算出来的结果看着收敛但坐标是错的。

**③ 求解并写回配置**

```bash
hecal                    # python3 recovery/handeye_calib.py
hecal --write-config     # 直接写进 real_params.yaml，不用手抄 4×4
```

**④ 验证（这步不能省）**

```bash
veh     # python3 robot_code/arm_grasp/scripts/verify_handeye.py
```

**残差小 ≠ 坐标对**。残差只证明"数据自己跟自己一致"，必须再用**尺子量一次**真实目标点：误差 < 3 mm 才算通过。如果整体偏了一个固定方向，把补偿填进 `perception.xy_bias_m`。

---

### 第 10 步 · 起链路，确认四个"有数"

```bash
link0     # 只起相机/夹爪/感知，不自动跑抓取任务
```

它会自动处理好 `source`（不用你手动敲两行，也不用写进 `~/.bashrc`）。

**另开一个终端**，逐条确认：

| 检查 | 命令 | 期望 |
|---|---|---|
| 相机 | `camhz` | ≈30 Hz |
| 关节角 | `jos` | 一直在刷 |
| 夹爪 | `timeout 3 ros2 topic echo /gripper/state` | 有角度和状态 |
| 感知目标 | `tgt` | 有 `[x, y, z, 宽, 高]` 五个数 |

**`tgt` 没数**：先确认方块在相机视野里，颜色对不对（默认找绿色），阈值可以在现场调：

```bash
tune      # 可视化调颜色阈值，调完 --write-config 写回
```

---

### 第 11 步 · 先演练，再真机

**演练（不动硬件，走完 11 个阶段）**

```bash
graspd     # python3 robot_code/arm_grasp/scripts/grasp_task.py --dry-run
```

它会打印每个阶段的规划结果。**这一步通过，说明配置、运动学、状态机全对**，剩下的只是硬件。

**真机抓取**

```bash
unlock on --speed 10     # 开两道锁，速度 10%
grasp 2>&1 | tee ~/grasp_1.log
```

**成功判据（重要）**：看**抬升时指尖是否跟随**，不是看夹爪报的状态位。

> 本机夹爪是**软橡胶指**，"到位 / 堵转"两个标志在真机上**不能区分夹住和夹空**——空夹时软指自己贴合，也会报"到位"。刚上手的人最容易在这里被自己骗。

**换目标颜色**（不用重启）：

```bash
ask "抓蓝色的方块"      # 一句话切换，运行期生效
ask 绿色                # 切回来
```

**放置**：默认开启，会走"竖抬 → 水平移 → 竖降 → 张嘴 → 撤离"五段。注意**放置留了安全间隙，不贴台面**——贴台面下放会让夹爪吃到反力。

---

### 第 12 步 · 记录与固化

**录制一份完整数据**（只订阅不发布，不影响运行）：

```bash
python3 robot_code/arm_grasp/scripts/record_run.py --tag first_run
# 输出 ~/grasp_data/run_时间戳_first_run.csv（20 列，约 200 Hz）
```

**把标定结果提交掉**（只提交配置，不动代码）：

```bash
save      # bash save_config.sh
gst       # 看一眼状态和最近提交
gpush     # push 到远端
```

> **为什么标定一成功就要提交**：这个项目在 11-09 丢过一次手眼标定（配置被回滚，而 git 里从来没有那份标定），重做了两小时。**标定结果比代码值钱，因为它不可复现。**

---

### 附 A · 常用别名速查（`rhelp` 看全部）

| 类别 | 别名 |
|---|---|
| 环境 | `ws` 进项目目录 · `renv` 载入 ROS 环境 · `cfg` 编辑配置 |
| 体检 | `pf` 全验（不接硬件）· `hw` 硬件体检 · `rnet` 网络自配 · `wmap` 可达区 · `hq` 手眼质量 |
| 动臂 | `unlock` 开锁 · `lock` 上锁 · `jog` 键盘示教 · `grasp` 真机抓取 · `graspd` 演练 |
| 标定 | `cg` 夹爪开口 · `sweepeh` 扫位采样 · `hecal` 手眼求解 · `veh` 打点验证 |
| 感知 | `tune` 调阈值 · `ask` 一句话换目标色 · `tgt` 看目标位姿 |
| 启动 | `link` 起整机 · `link0` 只起链路 · `camhz` 相机帧率 · `jos` 关节角 · `rtl` 话题列表 |
| Git | `gst` 状态 · `upd` 拉取 · `save` 提交配置 · `gpush` 推送 |
| 清僵尸 | `kcam` 相机占用 · `krsp` robot_state_publisher · `knodes` 抓取三节点 |

通用执行器：`r 脚本名 参数`（例：`r preflight.py`）、`rr 脚本名`（跑 `recovery/` 下的脚本）。

---

### 附 B · 十个必踩的坑

| 现象 | 原因 | 处理 |
|---|---|---|
| `colcon build` 报"找不到包" | 没建 `src` 软链 | `ln -s robot_code src` |
| `Package 'arm_grasp' not found` | 新终端没 source | 用 `link0`；或手动 source 两行（**别写进 ~/.bashrc**） |
| `/joint_states` 永远是空 | `udp_ip` 填错，或两块网卡同网段 | `rnet` 重配；`ip addr` 查网卡 |
| 动臂命令全被拒 | 两道锁是 false | `unlock on --speed 10` |
| 夹爪串口没数据 | 拨码开关 2 号没 ON / USB 不支持热插拔 | 断电重插，2 号拨到 ON |
| 抓取"成功"但方块还在桌上 | 软指导致堵转判据失效 | 看**抬升时指尖是否跟随** |
| 感知报的 z 偏高几毫米 | 顶面取法被棱边串色骗了 | 已改为直方图取最密一层，别退回旧算法 |
| 改了配置没生效 | 配置只在**启动时**读一次 | 重启 `link0` |
| `git pull` 后别名/命令失效 | 已开终端不重读 | `source ~/robot_vla_ws/aliases.sh` |
| 相机打不开 `RS2_USB_STATUS_BUSY` | 被别的进程占着 | `kcam` 清掉再起 |

---

### 附 C · 更新与换机器

**只更新代码**：

```bash
cd ~/robot_vla_ws
upd          # git pull --rebase
source ~/robot_vla_ws/aliases.sh
```

拉完先看 `real_params.yaml` 有没有被改回模板值（`allow_arm_motion` 可能变回 `false`），要动臂再 `unlock on`。

**换一台新机器**：把那台机器按本文第 1~9 步走一遍即可，`restore_real.sh` 和标定脚本都是幂等的。但要重新做两件事：**夹爪开口标定** 和 **手眼标定**（这两项跟具体机械结构绑定，换机器必须重测）。

**铁律**：任何情况下都**不要**把 `install/setup.bash` 写进 `~/.bashrc`——会污染其它 ROS 工作空间，导致"另一个项目突然跑不了"这种极难排查的问题。用 `link`/`link0` 或内联 `source`。
