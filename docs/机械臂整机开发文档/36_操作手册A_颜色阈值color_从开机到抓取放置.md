# 36 · 操作手册 A：颜色阈值（color）— 从开机到「抓取 + 放置」

> 2026-11-04 晚　适用：睿尔曼 ECO65-6FB + MS42DC 夹爪 + D435i 腕装 + Ubuntu 20.04/Foxy  
> **这是明天的主线**。目标：8 小时内把「抓起来 → 放到固定区」整套跑通。  
> 配套：34 号（为什么这么做）、35 号（两种模式对比）、30 号（移植细节）。  
> 本手册是**照着做就行**的版本，不需要先读别的文档。

**符号约定**

| 符号 | 含义             |
| -- | -------------- |
| ▶  | 你要敲的命令 / 要做的动作 |
| ✅  | 做到什么算过（判据）     |
| ❌  | 不过怎么办（按顺序试）    |
| ⏱  | 这一步的预计时间       |

**两个止损点（到点就走，别恋战）**

- **⏱ 3:20** 手眼打点还没过 → 立刻走文末的「兜底」路径，先把任务做完
- **⏱ 6:30** 抓取还没到 5 次过 3 次 → 停止调参，用最有把握的一组参数固化 + 录视频

---

## 今天（出发前）要准备的事

### 1.1 硬件清单（照着装箱）

| 物品              | 规格                   | 用途             | 没有会怎样         |
| --------------- | -------------------- | -------------- | ------------- |
| 机械臂 + 控制器 + 急停盒 | ECO65-6FB            | 主体             | 干不了           |
| 夹爪 + 电源         | MS42DC，10~25V        | 抓              | 干不了           |
| 夹爪 USB 串口线      | 芯片 CH340（1a86:55d4）  | 通信             | 夹爪不动          |
| RealSense D435i | 已固定在腕上               | 看东西            | 感知全废          |
| **USB3 线**      | 必须 5000M（蓝口）         | 相机             | USB2 会掉帧、深度断流 |
| 4cm 方块          | 绿（主）/ 红 / 浅蓝 各 1~2 个 | 目标物            | 干不了           |
| **游标卡尺**        | 0~150mm              | 量夹爪开口          | 无法校准夹爪锚点（关键）  |
| 卷尺 / 钢板尺        | 1m                   | 量坐标、量 ArUco 边长 | 手眼尺度会错        |
| A4 ArUco 标定纸    | 已贴在硬板上（你已做好）         | 手眼             | 干不了           |
| 胶带 / 压块         | —                    | 固定标定板          | 板一动整组数据作废     |
| 尖头记号笔           | —                    | 在台面标记放置区       | 放置点靠猜         |
| 笔记本 + 笔         | —                    | 记实验表           | 失败原因分不清       |
| 笔记本电源 / 充电器     | —                    | 8 小时           | —             |

### 1.2 软件（今晚就能做完，现场省 30 分钟）

▶ 在真机上：

```bash
cd ~/robot_vla_ws
git pull gitee main                     # 应拉到 f7efb14 或更新
source /opt/ros/foxy/setup.bash
python3 robot_code/arm_grasp/scripts/preflight.py
```

✅ 最后一行是 `自检汇总：✅ 2x 项通过   ⚠️  3 项待办   ❌ 0 项失败`（待办是标定相关，正常）

▶ 在云机上（没在真机旁边也能做）：

```bash
ssh ubuntu@你的服务器IP
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash
python3 robot_code/arm_grasp/scripts/preflight.py
python3 robot_code/arm_grasp/scripts/grasp_task.py --dry-run
```

✅ 0 ❌，且 dry-run 跑完 8 或 11 个阶段。**这保证代码没回归，明天的问题都在硬件/标定上。**

▶ 做一份离线备份（现场没网也不怕）：

```bash
cd ~ && tar czf ~/eco65_offline_$(date +%m%d).tar.gz \
  --exclude='robot_vla_ws/build' --exclude='robot_vla_ws/install' \
  --exclude='robot_vla_ws/log' --exclude='.git' robot_vla_ws
```

### 1.3 今晚就要定下来的 4 个数（写在纸上）

| 要定的事       | 怎么定                                   | 写什么                               |
| ---------- | ------------------------------------- | --------------------------------- |
| 抓哪个颜色      | 4cm 方块哪几个最显眼、桌面不撞色                    | `target_color: green`（或 red/blue） |
| 方块尺寸       | 必须是 **4cm**，小了大了都要改 `box_half_size_m` | 实测边长 \___ cm                      |
| 放置区大概在哪    | 台面上离抓取区 15~25cm、不贴边的位置                | 大概方向（左/右/远端）                      |
| ArUco 外框边长 | 卡尺量黑边到黑边，**单位米**。本机实测 **0.0595**（59.5mm） | 已写进配置，不用再填 |

> ⚠️ ArUco 边长是**手眼尺度的锚**。量错 1mm，整个坐标就带 1mm 系统误差，而且求解残差照样很小。  
> 你的纸已贴硬板，直接量就行（别量白边，要量**黑框外沿**）。

### 1.4 打印这两张表

**实验记录表**（每次抓取记一行）

| # | 感知 xyz | 夹爪 [开口/角度/到位/堵转] | 方块动了吗 | 结果 | 改了什么 |
| - | ------ | ---------------- | ----- | -- | ---- |
| 1 |        |                  |       |    | 基线   |

**时间表**（打勾用）

| 时段        | 步骤                      | 完成 |
| --------- | ----------------------- | -- |
| 0:00–0:25 | Step 1 上电与链路            | ☐  |
| 0:25–0:50 | Step 2 夹爪两把尺            | ☐  |
| 0:50–0:55 | Step 3 作业区实测            | ☐  |
| 0:55–1:10 | Step 4 相机与深度            | ☐  |
| 1:10–1:30 | Step 5 颜色阈值             | ☐  |
| 1:30–1:45 | Step 6 摆臂：观察位           | ☐  |
| 1:45–3:15 | Step 7 手眼标定（含重启后出数验收）   | ☐  |
| 3:15–3:35 | Step 8 打点验证             | ☐  |
| 3:35–3:45 | Step 9 放置点示教            | ☐  |
| 3:45–4:25 | Step 10 第一次抓取（先关 place） | ☐  |
| 4:25–5:05 | Step 11 开 place 跑全套     | ☐  |
| 5:05–5:45 | Step 12 重复与固化           | ☐  |

---

## 现场总览

```
上电 → 链路(25m) → 夹爪校准(25m) → 作业区/相机/颜色(40m) → 摆臂(15m)
   → 手眼标定(90m) → 出数验收(15m) → 打点(20m) → 放置点(10m)
   → 抓取(40m) → 开放置(40m) → 固化(40m)
```

**整条链只有 4 个"锚点"**，其余都是流程：

1. `max_width_mm`（夹爪开口锚点）—— Step 2 用卡尺量
2. `tcp_offset = 0.177`（法兰→指尖）—— 已实测，别改
3. 手眼矩阵 X（相机→法兰）—— Step 7
4. 深度尺度（D435i 出厂内参）—— Step 4 粗验

---

## Step 0：确认代码是最新的（2 分钟，别跳）

**这一步不通过，后面的命令会"找不到文件"** —— 因为手册里用到的工具是分批加进仓库的。

```bash
cd ~/robot_vla_ws
git log --oneline -1
ls robot_code/arm_grasp/scripts/net_autoconf.py
```

✅ 第一行是 `1f9737b …`（或比它更新的），第二行**打印出这个文件路径**

❌ `git log` 显示的是更旧的提交（比如 `9524d3e` / `e0eb0ac`）：
```bash
# ① 先备份本机配置（唯一可能被更新覆盖的东西）
cp robot_code/arm_grasp/config/real_params.yaml ~/params_backup_$(date +%m%d).yaml

# ② 看看本机改过什么（如果只列出 real_params.yaml，可以放心继续）
git status --short

# ③ 丢弃本地改动，拉最新
git checkout -- .
git pull gitee main

# ④ 确认
git log --oneline -1
```

> ⚠️ 若 ② 里除了 `real_params.yaml` 还有别的文件，**先别执行 ③**，把那行输出发我。
> ⚠️ 拉完配置会回到模板值（IP 变回 192.168.1.x、`allow_arm_motion` 变回 false）——
> 这正是下一步 `net_autoconf.py` 要处理的事，别慌。手眼/夹爪还没做，不会丢东西。

❌ `ls` 说文件不存在，但 `git log` 已经是最新：
```bash
git ls-files robot_code/arm_grasp/scripts | grep net_autoconf     # 仓库里有吗
git status --short                                                 # 是不是被删了
```

## Step 0.5：先认清「线缆包络」这条硬限制（3 分钟）

这台机器的连接线不够长，**法兰（Link6）只能在下面这个长方体里活动**，超出去就是拽线：

| 方向 | 允许范围 | 说明 |
|---|---|---|
| x 前后 | 0 ~ **0.50m** | 从基座中心量，正前方 |
| y 左右 | **±0.20m** | 共 40cm |
| z 高度 | 0 ~ **0.50m** | 从台面量（臂直接装在台面上） |

> ⚠️ **约束的是「法兰」，不是指尖。** 工具竖直向下时法兰比指尖高 17.7cm，
> 所以**指尖最高只能到 0.32m**。观察位就是被这条卡住的——想再高，必须先换线。
> J1 转动时这个盒子**不跟着转**，它是基座系里的固定盒子。

这个限制已经写进代码（`safety.cable_envelope_m`），四个地方会自动检查：

| 在哪 | 超了会怎样 |
|---|---|
| `preflight.py` | 列出 home / observe 的法兰坐标与"离包络最近多少 mm" |
| `grasp_task.py` 回 home、去观察位 | **拒绝执行**并报超了多少（不会拽线） |
| `planner` 下降/抬升/转运/撤离 | 逐点检查，任何一点越界就中止 |
| `jog.py` 每一步 | 从盒内往外走 → 拒绝；已在盒外往里收 → 放行并提示 |
| `workspace_map.py` | 按盒子裁剪，报"有效可抓区" |

▶ 想看一眼当前配置算不算合规：

```bash
python3 robot_code/arm_grasp/scripts/preflight.py 2>&1 | sed -n '/\[3b\]/,/\[4\/7\]/p'
```

✅ 看到类似：

```
允许范围：x +0.00~+0.50  y -0.20~+0.20  z +0.00~+0.50 m
✅ home 法兰 [0.200 0.000 0.450]，离包络最近 50mm
✅ observe 法兰 [0.200 0.000 0.450]，离包络最近 50mm
```

> 📌 **11-09 已把 home 换成合规姿势**：原来 `[0,0.2,0.3,0,0,0]` 的法兰在 0.724m，
> 超了 224mm。现在的新 home 是"工具竖直向下、法兰 (0.20,0,0.45)、指尖离台面 27cm"，
> 顺带把首次姿态切换从 1.571 rad 降到了 0.415 rad（不再整臂翻腕）。
> 换线之后只要改 `safety.cable_envelope_m` 三行，其余不用动。

---

## Step 1：上电与链路　⏱ 25 分钟

### 3.1 接线顺序（别搞反）

1. 夹爪电源（10~25V）接上 → 夹爪 USB-C 插电脑（**不支持热插拔，插好再上电**）
2. 机械臂网线接电脑有线口
3. 相机 USB3 插蓝色口
4. 最后给机械臂控制器上电

▶ 上电后确认急停盒是**松开**状态（按下会断使能）。

### 3.2 配网络（`/joint_states` 没数 90% 是这里）

```bash
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash
python3 robot_code/arm_grasp/scripts/net_autoconf.py
```

✅ 看到 `✅ ping 192.168.1.18 通` 和 `✅ 应写入的 udp_ip = 192.168.1.xxx（来自本机网卡，可信）`

❌ **`ping 192.168.1.18 不通`（2026-11-09 真机实际遇到）**

先看 `[1/5]` 那行「本机 IPv4」。如果里面已经有 `192.168.1.xx`，
说明**网络本身没问题，是机械臂不在 .18 上**，别去改网卡（`nmcli` 那三行不用管）。

新版脚本会**自动扫网段**：在 `192.168.1.0/24` 里逐个试 8080 端口
（那是 rm_driver 的控制端口），扫到谁就认定谁：

```
✅ 找到 192.168.1.19 / xx:xx:xx:xx:xx:xx（8080 开放）→ 判定为机械臂，改用它
```

- 扫到 1 个 → 脚本自动改用它并写配置，你重跑一次就通过
- 扫到多个 → `--arm-ip` 指定哪个是机械臂
- **一个都没有** → 是物理层的事，按顺序查：
  1. 网线是否插在机械臂的**网口**（不是示教器口），网口指示灯闪不闪
  2. 控制器上电了吗、急停松开没有（急停按下时会断网/断使能）
  3. **看机械臂自己的屏幕/示教器上显示的 IP**——那是唯一权威值
  4. 机械臂和本机是否接在同一台交换机上（有人把网线接到了墙上的办公网）

拿到真实 IP 后：`python3 robot_code/arm_grasp/scripts/net_autoconf.py --arm-ip 那个IP`

❌ 只有 `ping` 通、8080 连不上：机械臂没使能或端口被占（先解决使能，见 Step 1.1）。

❌ **本机有两个 `192.168.1.x` 地址**（比如有线 .10 + USB 网卡 .26）：
脚本会告警"同一网段出现在多块网卡上"。这种时候 UDP 回包可能走错网卡，
表现就是 **`/joint_states` 永远空**，而且 `ping` 还是通的（最容易误判）。
处理：只保留一块（拔掉另一根线，或把另一块改到别的网段），然后重跑脚本。
脚本已经改成问内核（`ip route get 机械臂IP`）来决定 `udp_ip`，比"挑一个同网段 IP"可靠。
❌ 报 `本机没有 192.168.1.x 同网段的 IP`（本机真的不在这个网段）：这才需要配静态 IP，
按脚本打印的 `nmcli` 命令做（脚本现在会用你机器上**真实的网卡名/连接名**，不再写死
`Wired connection 1`），配完重跑本脚本。

### 3.3 起链路

▶ 终端 A（这个窗口接下来一直开着，别关）：

```bash
bash ~/robot_vla_ws/start_link.sh start_task:=false
```

就这样一行。**不用记 source** —— `start_link.sh` 里已经做好两件事：

1. `source /opt/ros/foxy/setup.bash` + `source ~/robot_vla_ws/install/local_setup.bash`
2. 判断包有没有编译过：编过用包名，没编过用绝对路径（纯 Python，功能一样）

> ⚠️ 之所以要这个脚本：`ros2 launch` 只认**已经 source** 的包，新开终端不 source 就报
> `Package 'arm_grasp' not found`（不是包坏了）。而项目铁律不允许把 source 写进
> `~/.bashrc`（会污染其它工作空间），封进脚本是唯一两头都满足的办法。

❌ 报 `Package 'arm_grasp' not found` → 说明你直接敲了 `ros2 launch`，改用上面的
`bash ~/robot_vla_ws/start_link.sh ...` 就行。想以后也能用短名字，补一次编译：
`cd ~/robot_vla_ws && colcon build --packages-select arm_grasp --symlink-install`。

▶ 终端 B：

```bash
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash
timeout 3 ros2 topic echo /joint_states
```

✅ 输出里有 6 个关节角，且**一直在刷**（每秒好几次）。

❌ 一条都没有：

0. **先确认 rm_driver 真的起来了**：看终端 A 有没有报错、`ros2 node list | grep rm_driver` 有没有输出。
   （launch 因为没 source 而失败时，这里当然一条都没有 —— 那是前一步的问题）
1. `hostname -I` 看有没有 `192.168.1.x`
2. 有 → Ctrl+C 停终端 A，重新起一次（rm_driver 要重读配置）
3. 还没有 → 网线/网口灯、机械臂是否使能、`ping 192.168.1.18`
4. 最后才怀疑代码（代码这边已经验过很多次了）

### 3.4 在线体检

```bash
python3 robot_code/arm_grasp/scripts/check_real_hw.py --seconds 8
```

✅ 最后一行 `结论：GO —— 链路就绪，可以开始第 1 段（手眼标定）。`  
（此时"手眼标定未完成"会算 ⚠️ 不算 ❌，正常）

❌ 有 ❌ 就按它上面那行提示修，**别带着 ❌ 往下走**。

---

## Step 2：夹爪两把尺　⏱ 25 分钟

**这一步决定"能不能夹住"，比手眼更容易被忽视。**

> ⚠️ 标定脚本要**独占串口**。先 Ctrl+C 停掉终端 A 的整机 launch（夹爪节点会占串口），  
> 或者单独起夹爪节点。下面用"单独起"的方式。

### 4.1 张到底，量开口

▶ 终端 A（只起夹爪）：

```bash
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash
python3 robot_code/arm_grasp/scripts/gripper_node.py &
ros2 topic pub --once /gripper/cmd std_msgs/msg/String "data: 'home'"
```

▶ 等 8 秒，**用卡尺量两指内侧距离**（指尖根部，两指互相平行的位置）。

✅ 量到的数 ≈ `max_width_mm`（配置里是 120.0）。**量到多少就把配置改成多少。**

❌ 打不开 / 没反应：

1. 拨码开关 **2 号打到 ON**（1 号保持 OFF）—— 不打 ON 串口收不到数据
2. `ls -l /dev/motor_serial` 有没有这个设备名
3. 被占用会直接报 `打不开串口…`，脚本会列出三条原因

### 4.2 空夹一次，验证锚点

```bash
ros2 topic pub --once /gripper/cmd std_msgs/msg/String "data: 'grip:40'"
```

▶ 等 8 秒，再量开口。

✅ **37 ± 2 mm**（40mm 方块 − 3mm 过盈）

❌ 量到 47mm（常见）：说明 `max_width_mm` 比真实值**大 10mm**。  
改成 `max_width_mm = 120 + (47 − 37) = 130`，保存，重跑 4.1~4.2。  
通用公式：**新 max_width = 旧 max_width + (实测 − 37)**

❌ 量到 27mm：方向相反，`max_width_mm = 120 + (27 − 37) = 110`。

### 4.3 顺手记住"到位 / 堵转"这两个标志

```bash
timeout 3 ros2 topic echo /gripper/state
```

它输出 `[开口mm, 电机角度°, 到位, 堵转]`。**现在（空夹）记下这两个数**：

| 场景              | 期望        |
| --------------- | --------- |
| 空夹（指间没东西）       | 到位=1，堵转=0 |
| 夹住 4cm 方块（下一步做） | 到位=0，堵转=1 |

▶ 用手把 4cm 方块塞进两指之间，再发一次 `grip:40`，看 state 是否变成 `到位=0 堵转=1`。

✅ 两个场景的读数都和你预期一致（或至少**你知道了它们在本机的真实含义**）

> 为什么非要确认：后面"抓空了"只能靠这两个标志判断。如果语义和上面相反，  
> 抓取日志里那句 `⚠️ 夹爪报『到位』而不是『堵转』→ 很可能是夹空了` 就要反着读。

### 4.4 把标定结果写进配置

`config/real_params.yaml` 的 `gripper:` 段：

```yaml
  calibration_done: true       # ← 改 true
  max_width_mm: 130.0          # ← 用 4.1 量到的真实值
  min_width_mm: 10.0
  deg_per_mm: 15.0             # 2026-11-03 实测，一般不用改
  deadzone_deg: 105.0          # 同上
```

▶ 然后重启终端 A 的整机 launch（`bash ~/robot_vla_ws/start_link.sh start_task:=false`）。

---

## Step 3：作业区实测（不动机器人）　⏱ 5 分钟

**别再按 1m×1m 摆东西**——理论可达只有 0.19 m²，扣掉线缆包络后**只剩 0.101 m²**。

```bash
python3 robot_code/arm_grasp/scripts/workspace_map.py
```

✅ 第一行会打印裁剪依据，然后出图 + 结论：

```
已按线缆包络裁剪：x ≤ 0.50、|y| ≤ 0.20、法兰 z ≤ 0.50（指尖 ≤ 0.32）

三层都可达的区域（这才是「稳定作业区」）
   x: 0.120 ~ 0.400 （宽 280mm）
   y: -0.200 ~ 0.200 （宽 400mm）      ← 被线缆从 ±340 裁到 ±200
   面积 ≈ 0.101 m²
```

▶ 用记号笔在台面上按这个范围**画个框**（标定板、方块、放置区都要落在这个框里）。
▶ 图上还会给三个放置点候选，选一个离抓取区 10~15cm 的填进 `place.xy`。

❌ 如果提示 `三层都可达的点一个都没有` → `--surface-z` 填错了（桌面比底座高多少）。先量一下台面相对底座的高度差。

---

## Step 4：相机与深度　⏱ 15 分钟

```bash
timeout 5 ros2 topic hz /camera/d435i/color/image_raw
timeout 3 ros2 topic echo /camera/d435i/aligned_depth_to_color/image_raw
python3 robot_code/arm_grasp/scripts/check_real_hw.py --seconds 8
```

✅ 彩色 ≈30Hz；深度有数；体检里 **有效深度点 >30%**、距离中值 **0.3~0.6m**

|                                                             |                                    |
| ----------------------------------------------------------- | ---------------------------------- |
| ❌ 相机话题查不到 → \`ros2 topic list                               | grep d435i`看真实前缀，改`camera.prefix\` |
| ❌ 有彩图没深度 → launch 里 `align_depth.enable` 没开（默认是开的，检查是否被人改过） |                                    |
| ❌ 深度大面积空洞 → 目标太近（<0.28m）或阳光直射；拉窗帘、把观察位抬高                    |                                    |

---

## Step 5：颜色阈值对准　⏱ 20 分钟

### 7.1 先看 5 道门

```bash
python3 robot_code/arm_grasp/scripts/tune_perception.py --check-only --seconds 8
```

✅ 门 1~4 全绿（彩色图 / 对齐深度 / 内参 / 关节角）；门 5（手眼）现在不亮是正常的。

### 7.2 把方块摆好，调阈值

▶ 把 4cm 绿方块放到作业区：**底座正前方 25~35cm、左右偏离不超过 15cm**（即 x 0.25~0.35、y ±0.15），  
让相机能看到它。

▶ 用键盘把臂挪到"能俯视方块"的姿势：

```bash
python3 robot_code/arm_grasp/scripts/jog.py --step 20 --speed 5
```

键位：

```
w/s 前后   a/d 左右   r/f 升降       1~6 步长 0.5/1/5/10/20/50mm
i/k 俯仰   u/o 滚转   j/l 偏航       g 打印位姿   c 打印配置片段
h 回home   x 急停     q 退出
```

✅ 摆到「方块在画面中间、完整可见、夹爪不挡」→ 按 `q` 退出。

### 7.3 对准（可视化）

```bash
python3 robot_code/arm_grasp/scripts/tune_perception.py --color green --write-config
```

窗口里三个滑块：

| 滑块      | 作用     | 调法                           |
| ------- | ------ | ---------------------------- |
| `min`   | 绿色通道下限 | 调高 → 只留最绿；调低 → 变宽容（别把桌面也吃进来） |
| `lead1` | 比红色高多少 | 调高 → 排除偏黄                    |
| `lead2` | 比蓝色高多少 | 调高 → 排除偏青                    |

✅ **方块被整块高亮、背景一片黑**。按 `s` 保存（会**直接写进** `real_params.yaml`），按 `q` 退出。

❌ 命中 0 像素 → `min` 调低、`lead` 调低  
❌ 命中一大片（桌面也算）→ `min` 调高、`lead` 调高；或换一个和桌面颜色差别更大的方块  
❌ 没有显示器 → 加 `--headless --seconds 20`，只看统计

### 7.4 重启 + 验收出数

```bash
# 终端 A：Ctrl+C 停掉，重起（配置只在启动时读一次）
bash ~/robot_vla_ws/start_link.sh start_task:=false

# 终端 B
timeout 5 ros2 topic echo /grasp/target_xyz
```

✅ 终端 A 里看到类似：

```
目标顶面中心 base_link = [0.312 0.008 0.041] | 颜色命中 1832 px / 有效深度 1755 点 / 距离中值 0.412 m | 样本12 抖动0.8mm
```

四条都要对：数值刷新 / 与卷尺量的水平差 **<5mm** / 命中几百~~几千 px / 距离中值 0.3~~0.6m

❌ 一直不发布 → 看终端 A 里的诊断行，它现在会直接说卡在哪一道门：  
`颜色分割只命中 N 像素` / `mask 命中 N 像素但有效深度只有 M 个` / `收不到 /joint_states` / `手眼标定未完成`

---

## Step 5.4：用关节模式摸出「线不绷」的范围　⏱ 15 分钟

> 为什么必须做：2026-11-09 实测发现——**同一个末端位置有 4 个 IK 解，J6 能差 90°**；
> 而且**关节空间插值的中间姿态可能比目标姿态更糟**（走到一半线就绷了，只能按急停）。
> 所以别指望公式算安全，**用单关节小步试出来**。

```bash
python3 robot_code/arm_grasp/scripts/jog.py --joint --speed 5 --step 2
```

键位：

```
1~6   选关节         w/s   该关节 +2° / -2°（步长按 2 可改 1/5mm 等效为度）
g     打印当前 6 个关节角（记下来）      q 退出
```

**做法**（从一个你现在确认线是松的姿势出发）：

1. 按 `1` 选 J1，`w` 走几下，看线的松紧；记下"开始绷"的角度，退回
2. 按 `2` 选 J2，同样往正负各试
3. 依次 J3 / J5 / J6（**J4 和 J6 最影响夹爪那边的线，优先试这两个**）
4. 把每个关节"线还松"的正负极限记到纸上

得到的结果就是真正的安全区，比如：

| 关节 | 安全范围（相对当前） |
|---|---|
| J2 | −10° ~ +20° |
| J3 | 0 ~ +40° |
| J5 | 0 ~ −30° |
| J6 | ±25° |

> ⚠️ 这个范围**因线而异**，每台机器不同。记下来之后，`robot.home_joints`
> 和 `task.observe_joints` 都必须落在里面——而且要留 10° 余量。
>
> ⚠️ 关节模式一次只动一个关节，所以**永远不会走出"奇怪"的中间姿态**，这是它比 `--goto` 安全的地方。

---

## Step 5.5：开两道锁 + 把速度降到 5%（第一次动臂之前）　⏱ 3 分钟

真机有**两道锁**，只有同时满足才会真的下发运动指令。`git pull` 之后配置回到模板，
第二道锁是关着的 —— 这时候任何动臂的命令都会报：

```
❌ 本工具会动机械臂，需要两道锁：
   profile.mode: real   +   profile.allow_arm_motion: true
```

不用手改 YAML（缩进容易改错），一条命令：

```bash
# 只看当前状态
python3 robot_code/arm_grasp/scripts/arm_unlock.py

# 开锁 + 速度设成 5%（最慢）；会让你输入 yes 确认
python3 robot_code/arm_grasp/scripts/arm_unlock.py on --speed 5
```

✅ 看到：

```
✅ 已开锁：mode=real、allow_arm_motion=true、speed=5%
 ✅ 两道锁都开了 —— 现在 jog / 标定扫位 / 抓取任务都会真的动
```

**速度怎么定**（这是安全的第一道闸）：

| 阶段 | 速度 | 说明 |
|---|---|---|
| 第一次 jog / 第一次扫位 | **5%** | 慢到手能跟上，撞击力也小 |
| 标定、抓取调参 | 10% | 配置文件里的默认值 |
| 稳定后 | ≤20% | 代码里 `speed_hard_max: 20`，配再大也只按 20 走 |

> ⚠️ 不用了就立刻上锁：`python3 robot_code/arm_grasp/scripts/arm_unlock.py off`
> （只关第二道锁，`mode` 保持 real 不影响只读脚本）

> 💡 只读脚本（`preflight.py` / `workspace_map.py` / `tune_perception.py` /
> `check_real_hw.py`）**不需要开锁**，它们不下发运动指令。

---

## Step 6：摆臂定观察位　⏱ 15 分钟

观察位 = 一个**固定关节角**，作用只是"抬手看一眼"，让相机完整俯视作业区。

> 📌 **11-09 已经预填了一个能用的观察位**（和 home 同一个姿势）：
> 法兰 (0.20, 0, 0.45)、工具竖直向下 → 相机离台面约 0.45m，视野约 55×31cm，
> 刚好覆盖整个光学板，**且在线缆包络内**。
> 所以这一步现在变成"**确认/微调**"，不是"从零摆一个"。

```bash
python3 robot_code/arm_grasp/scripts/jog.py --step 20 --speed 5
```

▶ 按 `h` 回 home（也就是现在的观察位）→ 看相机画面（另开终端 `rqt_image_view`）：
方块、标定板是不是都在画面里？
- **都在** → 什么都不用改，直接 `q` 退出
- **看不全 / 夹爪挡视野** → 用 `w/s/a/d` 微调（步长按 `2` 改 1mm），
  调好按 `c` 打印，把 `observe_joints` 那行抄进配置

✅ 改完后跑 dry-run，日志里 `首次姿态切换` 应 **< 0.6 rad**（现在实测 0.415）

❌ `jog` 报 `超线缆包络`：说明这一步会让法兰出盒。**往盒子里退，别硬顶**——
真实原因是线不够长，改姿势只能绕开，绕不开就得换线。

> 为什么不能把观察位抬得很高：法兰上限 0.50m，工具竖直时指尖最多 0.32m，
> 相机最多到台面上方 ~0.45m。想看得更高/更远，只能先换线。

---

## Step 7：手眼标定　⏱ 90 分钟（**唯一可能吞掉半天的一步**）

### 9.1 前置

- 标定板**贴在硬板上、放在台面里、全程不许动**（你已做好）；位置建议**底座正前方 20~35cm、左右 ±25cm 内**
- **外框边长已存配置**（`handeye.marker_size_m: 0.0595`，59.5mm），不用再传参
- 终端 A 的整机 launch 在跑（相机 + `/joint_states`）
- **两道锁要打开**（扫位会让机械臂自己动）：

```yaml
profile:
  mode: real
  allow_arm_motion: true     # ← 标定阶段就要开（因为没示教器，扫位靠程序动臂）
```

### 9.2 先把臂领到起点

```bash
python3 robot_code/arm_grasp/scripts/jog.py --step 20 --speed 5
```

▶ 摆到「A4 板完整在画面中间、距离 0.35~0.5m」→ 按 `q` 退出。

> 这一步替代了"用示教器摆 12 个位姿"。后面 15 个位姿由脚本自己走。

### 9.3 自动扫位采集（约 1.5 分钟）

```bash
python3 recovery/collect_handeye.py --sweep --marker-size 0.0595 --out ~/handeye_samples.json
```

> 这个数已存在配置里（`handeye.marker_size_m: 0.0595`），**不传 `--marker-size` 也会自动读它**。
> 换一张纸必须重量：A4 打印有 0.5~2% 缩放误差，1mm 的错会让整体坐标偏 1mm 且残差看不出来。

✅ 一行行往下滚，最后：

```
✅ 已保存 12 组到 /home/xxx/handeye_samples.json
```

每组都带 `dict=DICT_xxx id=0 重投影 0.8px`。**重投影 >2px 的组会标"建议重拍"，但扫位版本没法重拍单组，忽略即可**。

❌ `采到 3 组` 或更少 → 起点位姿就看不清板子。回到 9.2 重新摆（离近一点、对准一点），再来  
❌ `没检测到 ArUco` → 看：板是否完整入镜 / 光照是否均匀 / `--marker-size` 是否真量过  
❌ `cv2.aruco 不可用` → `pip3 install --user opencv-contrib-python==4.2.0.34`

### 9.4 质量判据

```bash
python3 robot_code/arm_grasp/scripts/handeye_quality.py --input ~/handeye_samples.json --focused
```

✅ 没有 `❌`（`⚠️ 未传 --probe` 是正常的，打点在 Step 9 做）

❌ 有 ❌（样本太少 / 旋转激励不足）→ 加大幅度重采：  
`--sweep-span 0.05 --sweep-tilt 15`


### 9.5 求解 + 自动写配置

```bash
python3 recovery/handeye_calib.py --input ~/handeye_samples.json --pairs all --write-config
```

✅ 看到：

```
✅ 已写入 .../real_params.yaml（改动了 calibrated / matrix_4x4 / tf_xyz / tf_quat_xyzw）
   备份：.../real_params.yaml.bak.20261105-xxxxxx
```

> ⚠️ **写进去之后立刻提交**（这一步很容易忘，忘了会丢）：
>
> ```bash
> cd ~/robot_vla_ws
> git add robot_code/arm_grasp/config/real_params.yaml
> git commit -m "real: 手眼标定结果（现场）"
> ```
>
> 为什么：标定结果是写在这个**被 git 跟踪**的配置文件里的。下次跑 `update.sh`
> 或任何 `git checkout --`，未提交的标定就会被还原掉（2026-11-09 真踩过）。
> 现在 `update.sh` 会检测到“非 IP 的本地改动”并拦住你，但**提交才是根治**。

同时它会打印 `相机在法兰系的位置 = [...] m`。  
**自己看一眼这个数**：应该在 5~25cm 量级（相机装在腕上就是这个距离）。如果是 5 或 130 这种数 → 单位/定义错了，别往下走。

❌ 报 `结果可疑，先别写进配置` → 按它给的提示查（单位、A/B 定义、四元数没归一化）

### 9.6 重启 + 出数

```bash
# 终端 A：Ctrl+C 重起
bash ~/robot_vla_ws/start_link.sh start_task:=false
# 终端 B
timeout 5 ros2 topic echo /grasp/target_xyz
```

✅ 有数，且与卷尺量的方块位置水平差 **<5mm**（这一步只能粗判，精判在 Step 9）

---

## Step 8：打点验证（**别跳**）　⏱ 20 分钟

残差小只说明数据自洽，不说明坐标对。这一步用**机械臂自己的编码器**当基准，不用尺子。

```bash
python3 robot_code/arm_grasp/scripts/verify_handeye.py --points 5 --marker-size 0.0595
```

### 每个点做什么

1. 脚本让你确认"相机能看见 A4 板"→ 回车
2. 它算出标记中心，把指尖开到**正上方 2cm**（低速）
3. **你用键盘把指尖微调到正好压住标记中心**：

```
w/s = x±   a/d = y±   r/f = z±       1/2/3 = 步长 0.5 / 1 / 5 mm
调好按 Enter 记录；按 q 放弃这一点
```

1. 回车后它读关节角、反算指尖真实位置，算出偏差
2. 第 2~~4 个点：**把 A4 板整体挪到别的位置**（臂不用动，挪 10~~15cm），重复

✅ 最后：

```
平均偏差  dx=+1.8  dy=-0.7 mm
去均值后最大残差 1.2 mm（这就是「补不掉的那部分」）
✅ 结论：偏差基本是【常数】，可以补偿
   perception:
     xy_bias_m: [-0.0018, 0.0007]
```

▶ 把 `xy_bias_m` 填进 `real_params.yaml` 的 `perception` 段，重启终端 A。

❌ `❌ 结论：偏差随位置变化（最大 X mm）` → 是旋转/尺度问题，补偿救不了：

1. `--marker-size` 是不是真量过（最常见）
2. `tcp_offset` 是不是 0.177
3. 采集时的旋转激励够不够（Step 7.4）
4. 都不行 → **走第 12 节兜底**

---

## Step 9：放置点示教　⏱ 10 分钟

```bash
python3 robot_code/arm_grasp/scripts/jog.py --step 5 --speed 5
```

▶ 把**指尖**移到放置区中心、**刚好碰到台面**（不是悬空、不是压进去）→ 按 `q` 退出。

```bash
python3 robot_code/arm_grasp/scripts/teach_place_point.py
```

✅ 打印：

```
    xy: [0.3200, -0.1500]
    surface_z: 0.0200
  ✅ 数值范围看起来合理
```

▶ 填进 `config/real_params.yaml` 的 `place` 段：

```yaml
place:
  enabled: true          # ← 先别急，Step 11 抓取稳了再改 true
  xy: [0.3200, -0.1500]
  surface_z: 0.0213
```

❌ 报 `z=0.7xx 看起来不像桌面高度` → 指尖还在半空，重新摆到底  
❌ `水平距离超出工作半径` → 放置区选远了，往里挪

---

## Step 10：第一次抓取　⏱ 40 分钟

### 12.1 先把"抓"跑通（place 关闭）

`config/real_params.yaml`：

```yaml
place:
  enabled: false            # ★ 先关：只跑 8 阶段，抓完停在抬升终点
```

锁和速度用一条命令（别手改，容易改错缩进）：

```bash
python3 robot_code/arm_grasp/scripts/arm_unlock.py on --speed 5
```

> 第一次一律 **5%**。跑顺了两三次之后，再 `arm_unlock.py on --speed 10`。

### 12.2 空跑 + 空爪预演

```bash
python3 robot_code/arm_grasp/scripts/preflight.py
python3 robot_code/arm_grasp/scripts/grasp_task.py --dry-run
```

✅ 0 ❌；dry-run 跑完 `[8/8]`（place 关时是 8 阶段）

▶ 空爪预演一次（夹爪不夹东西，但整条路径真的走）。

**链路用 launch 起（只起节点，不跑任务）：**

```bash
bash ~/robot_vla_ws/start_link.sh start_task:=false
```

**任务单独跑，加 `--slow`（强制 5%）——第一次千万别用满速：**

```bash
python3 robot_code/arm_grasp/scripts/grasp_task.py --real --slow 2>&1 | tee ~/grasp_$(date +%H%M).log
```

> `--slow` 会打印 `⚠️ --slow：速度强制 5%（配置里是 10%）`。
> 亲眼确认它比配置更慢，再往下做。

✅ 机械臂走完"回 home → 观察位 → 下降到方块上方 → 预夹 → 下降 → 夹持 → 抬升"

**看两件事**：

1. 路径是不是你预期的那条（没有横扫、没有蹭桌面）
2. 落点是不是方块正上方（差多少？记下来）

### 12.3 正式抓 5 次

▶ 放好方块，再跑一次同样的命令。每跑一次记一行表：

| # | 感知 xyz | 夹爪 [开口/角度/到位/堵转] | 方块动了吗 | 结果 | 改了什么 |
| - | ------ | ---------------- | ----- | -- | ---- |

✅ **5 次里 ≥3 次**抓起来

❌ 抓空（日志 `⚠️ 夹爪报『到位』而不是『堵转』`）：

1. 落点每次都往同一方向偏 → 偏差是常数，回 Step 9 重做打点、补 `xy_bias_m`
2. 偏得很随机 → 看感知那行日志的"抖动"（>2cm 说明光照/阈值不稳）
3. 一点不偏但夹不住 → Step 2 的 37mm 对不对？`interference_mm` 3 改成 4

❌ 下降时把方块推走：

1. 方块摆歪了 → 换正一点；预夹余量从 12 调到 16
2. 每次都往同一方向推 → 常数偏置没补（回 Step 9）

❌ 夹住了但抬起来掉了：`interference_mm` 3→4，抬升速度降（`lift_steps` 5→8）

---

## Step 11：开 place，跑完整任务　⏱ 40 分钟

### 13.1 空爪先过一遍转运路径

```yaml
place:
  enabled: true
```

```bash
python3 robot_code/arm_grasp/scripts/grasp_task.py --real --slow 2>&1 | tee ~/grasp_$(date +%H%M)b.log
```

✅ 日志走到 `[9/11] 转运到放置区` → `[10/11] 张开夹爪释放` → `[11/11] 撤离` → `✅ 抓取 + 放置流程执行完毕`

▶ 空爪这一遍**只看路径**：抬—移—降是不是在台面上方、落点对不对。

### 13.2 真抓真放

▶ 放好方块，再跑一次。

✅ 方块被放到了放置区（允许偏 1~2cm，第一版不追求毫米级）

❌ 转运蹭桌面 → `place.approach_height_m` 0.16 → 0.18（先跑 preflight 看可达性）  
❌ 落点每次偏同一方向 → `place.xy` 没对准，重做 Step 9  
❌ 方块落地翻倒 → `release_gap_m` 0.008 → 0.004  
❌ 撤离把方块带走 → `open_width_mm` 90 → 100  
❌ `[9/11] 无 IK 解` → 放置点太远或 `retreat_height_m` 太大（z 上限约 0.20）

---

## Step 12：重复与固化　⏱ 40 分钟

1. 换个位置再抓 3 次（作业区内），统计成功率
2. 每次只改一个参数（记录表最后一列），别同时改两个
3. 固化：

```bash
cd ~/robot_vla_ws
git add robot_code/arm_grasp/config/real_params.yaml
git commit -m "real: 现场标定结果（手眼/夹爪/放置点/观察位）"
git push gitee main
cp ~/handeye_samples.json ~/handeye_samples_final.json
```

- [ ] `real_params.yaml` 已提交（含 matrix_4x4 / 夹爪四行 / place / observe_joints）
- [ ] `handeye_*` 留档
- [ ] 所有 `grasp_*.log` 留存
- [ ] 拍一段成功视频
- [ ] 写一页复盘：今天卡在哪、明天先做什么

> **做到这里，A 就完成了。** 想换一种识别方式（开放词汇、认苹果/杯子）时，  
> 不用重做任何标定——看 `37_操作手册B_识别大模型brain` 的**第 0 节**（约 45 分钟增量）。

---

## 兜底：手眼不过，也要把任务做完（30 分钟）

**触发条件**（任一）：3:20 打点还不过 / 打点残差 >5mm / 抓 5 次只成 1 次且排除了夹爪问题。

**做法：不用相机、不用手眼，用示教点跑完整流程。**

```bash
# ① 把方块放到作业区固定位置（以后就放这）
# ② 用键盘把指尖移到方块【顶面中心】
python3 robot_code/arm_grasp/scripts/jog.py --step 1 --speed 5
# ③ 读坐标（这个脚本只做 FK，不依赖手眼）
python3 robot_code/arm_grasp/scripts/teach_place_point.py
#    → 打印 xy / surface_z，这三个数就是方块顶面中心
```

▶ 填进配置：

```yaml
task:
  fixed_target_xyz: [0.3120, 0.0080, 0.0410]     # ← 用打印出来的三个数
place:
  enabled: true
```

▶ 照常跑：

```bash
bash ~/robot_vla_ws/start_link.sh start_task:=true 2>&1 | tee ~/grasp_$(date +%H%M)fb.log
```

✅ 日志抬头是 `目标源=示教固定点`，然后照常走完抓取 + 放置。

> 这不是作弊：坐标变换、IK、夹持、转运、释放全都在真跑，**只是"方块在哪"改成人告诉它**。  
> 汇报时要明说这一点。缺点是方块换位置就得重新示教。

---

## 故障速查（按现象找）

| 现象                    | 先查                      | 处理                                                |
| --------------------- | ----------------------- | ------------------------------------------------- |
| `/joint_states` 一条都没有 | `udp_ip`                | `net_autoconf.py` 重跑                              |
| 相机话题查不到               | 前缀                      | `ros2 topic list \| grep d435i` 改 `camera.prefix` |
| `Foxy echo --once` 报错 | Foxy 的 echo 没有 `--once` | 用 `timeout 5 ros2 topic echo`（`pub` 有 `--once`）   |
| 夹爪不动                  | 串口/拨码/占用                | `打不开串口` 三条排查；标定要独占串口                              |
| 夹爪读数总差 10mm           | `max_width_mm`          | Step 2 用卡尺量真值                                     |
| 颜色命中 0 像素             | 阈值                      | `tune_perception --color xxx --write-config`      |
| 有彩图没深度                | `align_depth` / 距离      | 查 launch 参数；抬高观察位                                 |
| 感知有数但抓空               | 手眼/偏置                   | Step 8/9                                          |
| 下降时推走方块               | 方块歪 / 预夹余量              | 摆正；`pregrip_margin_mm` 12→16                      |
| 夹住抬起来掉                | 过盈 / 指面                 | `interference_mm` 3→4；指面贴防滑胶                      |
| `[9/11] 无 IK 解`       | 放置点太高/太远                | 降 `retreat_height_m`，往里挪                          |
| 手眼矩阵平移动 5 / 130       | 单位 / A·B 定义             | 别写配置，回去查                                          |
| 机械臂完全不动、movej 无回应     | **使能 / 急停没松**           | 急停盒松开；控制器重新使能（见 Step 1）                           |

---

## 一页纸速查卡（打印这一页）

```bash
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash

# 1 链路
python3 robot_code/arm_grasp/scripts/net_autoconf.py
bash ~/robot_vla_ws/start_link.sh start_task:=false      # 终端A
timeout 3 ros2 topic echo /joint_states                            # 终端B：有数
python3 robot_code/arm_grasp/scripts/check_real_hw.py --seconds 8  # GO

# 2 夹爪（独占串口）
python3 robot_code/arm_grasp/scripts/gripper_node.py &
ros2 topic pub --once /gripper/cmd std_msgs/msg/String "data: 'home'"
#   → 卡尺量 = max_width_mm 真值
ros2 topic pub --once /gripper/cmd std_msgs/msg/String "data: 'grip:40'"
#   → 卡尺 37±2mm；差 N mm 就 max_width += N

# 3 作业区
python3 robot_code/arm_grasp/scripts/workspace_map.py

# 4 颜色阈值
python3 robot_code/arm_grasp/scripts/tune_perception.py --check-only --seconds 8
python3 robot_code/arm_grasp/scripts/jog.py --step 20 --speed 5          # 摆到能看见方块
python3 robot_code/arm_grasp/scripts/tune_perception.py --color green --write-config
#   → 滑块对准 → s 保存 → q 退出 → 重启终端A
timeout 5 ros2 topic echo /grasp/target_xyz                    # 与尺子差 <5mm

# 5 观察位
python3 robot_code/arm_grasp/scripts/jog.py --step 20 --speed 5          # 摆好 → c 打印 → 抄进配置

# 6 手眼（先开 allow_arm_motion）
python3 recovery/collect_handeye.py --sweep --marker-size 0.0595 --out ~/handeye_samples.json
python3 robot_code/arm_grasp/scripts/handeye_quality.py --input ~/handeye_samples.json --focused
python3 recovery/handeye_calib.py --input ~/handeye_samples.json --pairs all --write-config
#   → 重启终端A → target_xyz 有数

# 7 打点
python3 robot_code/arm_grasp/scripts/verify_handeye.py --points 5 --marker-size 0.0595
#   → 键盘微调 → 抄 xy_bias_m 进配置 → 重启

# 8 放置点
python3 robot_code/arm_grasp/scripts/jog.py --step 5           # 指尖碰台面
python3 robot_code/arm_grasp/scripts/teach_place_point.py

# 9 抓取（place.enabled 先 false）
python3 robot_code/arm_grasp/scripts/preflight.py
bash ~/robot_vla_ws/start_link.sh start_task:=true 2>&1 | tee ~/grasp_$(date +%H%M).log

# 10 开 place 跑全套（place.enabled: true）

# 兜底（手眼不过）
#   jog.py 指尖对齐方块顶面 → teach_place_point.py 读 xyz → task.fixed_target_xyz
```

**六条判据（全过才算成）**

1. `/joint_states` 有数　　2. `grip:40` 空夹 = 37±2mm
2. 深度有效 >30%、距离 0.3~0.6m　　4. `handeye_quality --focused` 无 ❌
3. `verify_handeye` 去均值残差 <5mm　　6. 5 次抓取 ≥3 次放到位
