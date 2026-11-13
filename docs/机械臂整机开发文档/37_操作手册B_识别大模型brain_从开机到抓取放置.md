# 37 · 操作手册 B：识别大模型（brain / Qwen2.5-VL-3B）— 从开机到「抓取 + 放置」

> 2026-11-04 晚　适用：同上，另加 **RTX 3070 8GB + Ollama**
> **这是"进阶版"**：把"目标在哪"从颜色阈值换成端到端识别大模型（开放词汇）。
> 其余（深度、手眼、IK、夹持、放置）**和手册 A 完全同一套**。
>
> ⚠️ 建议顺序：**先用 36 号手册（color）把整套跑通**，再来这本。
> 如果只有一天，且没跑过任何一遍 → **直接看 36 号**，这本留着做第二次/加分项。

**符号约定**：▶ 要做 / ✅ 算过 / ❌ 不过怎么办 / ⏱ 时间

---

## 0. 已经按 36 号（color）跑通了？只看这一节（约 45 分钟）

> 你说得对：**A 已经做完的部分，B 不用重做。**
> 这本手册的 Step 1~14 是"从零开始"的完整版（留给换机器/重装/给别人复现时用）。
> 如果 A 已经跑通抓取+放置，你只需要下面 5 步。

### 0.1 先确认：A 的成果哪些能直接复用

| A 里做完的东西 | B 里还用吗 | 说明 |
|---|---|---|
| `handeye.matrix_4x4` / `calibrated` | ✅ **完全复用** | 大脑只给 2D 框，3D 还是 `FK·X·p_cam` |
| `perception.xy_bias_m`（打点补的偏置） | ✅ **完全复用** | 偏置加在感知输出上，两种模式共用 |
| 夹爪四行（`max_width_mm` / `deg_per_mm` / `deadzone_deg`） | ✅ 完全复用 | 与感知无关 |
| `place.xy` / `surface_z` | ✅ 完全复用 | 与感知无关 |
| `task.observe_joints` | ✅ 复用，但**建议重看一眼** | 大脑要求目标在画面里够大（见正文 **Step 6**） |
| `perception.color_thresholds` | ✅ 留着当兜底 | 大脑挂了自动回退时用它 |
| `task.fixed_target_xyz` | ⚠️ 如果填了要**清成 null** | 否则状态机根本不走感知（日志会写"目标源=示教固定点"） |

**结论：手眼、夹爪、放置点、偏置一样都不用重做。B 只是换掉"找目标像素"这一步。**

### 0.2 会动到的配置只有 3 行（其余一律别碰）

| 键 | 从 | 到 |
|---|---|---|
| `perception.target_source` | `color` | `brain` |
| `brain.enabled` | `false` | `true` |
| `perception.target_name` | `"green cube"` | 你要认的东西（先保持 `"green cube"`） |

▶ 一条命令改全（自动备份 + 一致性检查）：

```bash
python3 robot_code/arm_grasp/scripts/perception_mode.py brain --target "green cube"
```

### 0.3 五步增量（每步都有判据）

**① 装模型（10~20 分钟，今晚就能做完）**

```bash
ollama pull qwen2.5vl:3b
nvidia-smi --query-gpu=memory.used,memory.total --format=csv
```

✅ `ollama list` 里有 `qwen2.5vl:3b`；加载后显存 **5~6GB / 8GB**

❌ 接近 8GB → 换更小的量化版本；❌ 连不上 → `ollama serve` / 端口

**② 自检：让模型认一次（5 分钟）**

```bash
# 相机当前帧（终端 A 的 launch 要开着）
python3 robot_code/arm_grasp/scripts/brain.py --check
```

✅ 框落在方块上 + 标签对 + **延迟 <1.5s**。三条都过才继续。

**③ 切换 + 重启（2 分钟）**

```bash
python3 robot_code/arm_grasp/scripts/perception_mode.py brain --target "green cube"
# 终端 A：Ctrl+C，重新起
ros2 launch arm_grasp real_grasp.launch.py start_task:=false
```

✅ `perception_mode` 打印三行 ✅，且 `大脑服务端 127.0.0.1:11434 可连接`

**④ 验收"真的在走大脑"（5 分钟）**

```bash
timeout 5 ros2 topic echo /grasp/target_xyz
```

✅ 日志出现 **`大脑框命中`**，且坐标与 A 时**同一位置差 <5mm**（差得多说明框没框准）

❌ 还是 `颜色命中` → 看上一行 `大脑还没给出目标（原因）`，按原因处理

**⑤ 重跑 5 次抓取（20 分钟，**必须重跑**）**

```bash
ros2 launch arm_grasp real_grasp.launch.py start_task:=true 2>&1 | tee ~/grasp_$(date +%H%M)brain.log
```

✅ 5 次 ≥3 次（和 A 的判据一样）。
这一次跑的是**完整 11 阶段**（因为 A 里已经把 `place.enabled` 打开了），
所以顺便验证了"大脑识别 + 转运放置"整条链。

> **为什么必须重跑**：大脑的延迟是 0.3~2s，而 color 是 0ms。
> 感知的"12 帧均值 + 抖动检查"在时间轴上被拉长了，落点可能比 A 时更"跟手慢一点"。
> 如果这一轮掉到 3/5 以下，先调这两个参数（**不要动标定**）：
> `brain.interval_s: 0.5 → 0.3`、`perception.jitter_max_m: 0.02 → 0.03`。

### 0.4 回切（一行）

```bash
python3 robot_code/arm_grasp/scripts/perception_mode.py color
ros2 launch arm_grasp real_grasp.launch.py start_task:=false
```

### 0.5 什么情况下"只做增量"不成立

| 情况 | 要做完整版 |
|---|---|
| 换了机器 / 重装了系统 | 是（手眼、夹爪、放置点全要重来） |
| 8GB 装不下 3B，要换 Florence-2 或别的服务 | 是（提示词/接口/坐标系响应格式可能要改） |
| 要认的目标**不是 4cm 立方体**（苹果/杯子） | 是，而且还要改几何假设（见 Step 13） |
| 只是"换个词认另一个方块" | 否，增量 ③ 里改 `--target` 就行 |

---

## 今天（出发前）要准备的事

### 1.1 硬件（在 A 的清单上多两样）

| 物品 | 规格 | 为什么 |
|---|---|---|
| 同 A 的全套 | 机械臂/夹爪/D435i/方块/卡尺/ArUco/卷尺 | 一样都不能少 |
| **RTX 3070 那台机器能上网** | 装 Ollama + 拉模型 | 没网就先在别的机器下好再拷 |
| 一张**测试图片** | 用手机拍一张"方块在桌上的照片" | `brain.py --check --image` 用，不依赖相机 |

### 1.2 软件：今晚就把模型拉好（现场最费时间的一步）

▶ 在 3070 那台机器上：

```bash
# 装 Ollama（一条命令；装完 ollama serve 会随系统起）
curl -fsSL https://ollama.com/install.sh | sh

# 拉模型：Qwen2.5-VL-3B，Q4 量化 ≈1.9GB + 视觉投影 ≈1.3GB
ollama pull qwen2.5vl:3b

# 确认
ollama list
curl -s http://127.0.0.1:11434/v1/models | head -c 300
```

✅ `ollama list` 里有 `qwen2.5vl:3b`；`curl` 返回一段 JSON（含 `"id":"qwen2.5vl:3b"`）

▶ **看显存**（关键）：

```bash
nvidia-smi --query-gpu=memory.used,memory.total --format=csv
```

✅ 模型加载后占用 **5~6 GB / 8 GB**（留 2GB 给相机与 ROS）

❌ 已经接近 8GB → 换更小的：`ollama pull qwen2.5vl:3b-instruct-q4_K_M`，或改用 Florence-2

> **别用 7B**：Q4 就占 ~7.1GB（90%），抓取时和相机抢资源，会让你以为是"手眼不准"。
> **也别试 π0.5/openpi**：3.3B 模型推理要 9~12GB，8GB 放不下（想用只能把 policy 放云端）。

### 1.3 今晚跑一次"离线自检"（不需要机械臂）

▶ 把手机拍的测试图拷到真机，比如 `~/test_cube.jpg`：

```bash
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash
python3 robot_code/arm_grasp/scripts/brain.py --check --image ~/test_cube.jpg
```

✅ 打印出：`标签：green cube`、`框：[x1 y1 x2 y2] 中心 (u, v)`、`耗时：xxx ms`

❌ 连不上 → `ollama serve` 起了吗？`curl` 通吗？
❌ 没框 → 提示词/模型名；先确认 `brain.py` 里 url 与 `ollama list` 里的名字一致
❌ 耗时 >1500ms → 记录这个数，现场可以先把 `brain.interval_s` 调大、或降分辨率

### 1.4 今晚定下来的 5 个数

| 要定的事 | 例 |
|---|---|
| 要认什么（英文） | `"green cube"`（也可以 `"red block"` / `"apple"` / `"white cup"`） |
| 夹爪 `max_width_mm` | 现场量（Step 2） |
| ArUco 外框边长 | _____ m |
| 放置区方向 | 左 / 右 / 远端 |
| 模型加载后显存 | _____ GB（1.2 里看到的值） |

### 1.5 打印

- A 手册里的**实验记录表**（多一列"大脑标签"，见下）
- **时间表**（比 A 多 40 分钟）

| # | 感知 xyz | 大脑标签 | 夹爪 [开口/角度/到位/堵转] | 方块动了吗 | 结果 | 改了什么 |
|---|---|---|---|---|---|---|

| 时段 | 步骤 | 完成 |
|---|---|---|
| 0:00–0:25 | Step 1 上电与链路 | ☐ |
| 0:25–0:50 | Step 2 夹爪两把尺 | ☐ |
| 0:50–0:55 | Step 3 作业区实测 | ☐ |
| 0:55–1:10 | Step 4 相机与深度 | ☐ |
| **1:10–1:45** | **Step 5 大脑上线与验收**（比 A 多 20 分钟） | ☐ |
| 1:45–2:00 | Step 6 摆臂：观察位 | ☐ |
| 2:00–3:30 | Step 7 手眼标定 | ☐ |
| 3:30–3:45 | Step 8 出数验收 | ☐ |
| 3:45–4:05 | Step 9 打点验证 | ☐ |
| 4:05–4:15 | Step 10 放置点示教 | ☐ |
| 4:15–5:00 | Step 11 第一次抓取 | ☐ |
| 5:00–5:40 | Step 12 开 place 跑全套 | ☐ |
| 5:40–6:30 | Step 13 换词汇泛化实验 | ☐ |
| 6:30–7:00 | Step 14 固化 | ☐ |

---

> ⬇️ **以下 Step 1~14 是「从零开始」的完整版。**
> 已按 36 号跑通 A 的，**跳过 Step 1~4 和 Step 6~12**（那些你已经做完了），
> 只读 **Step 5（大脑上线）** 和 **Step 13（泛化实验）/ Step 14（固化与回切）**。
> 增量版的 5 步清单在上面第 0 节。

## 两条路的唯一区别（先看清再动手）

```
彩色图 ──► ① 目标像素 ──► 对齐深度反投影 ──► FK·X ──► p_base
              ▲
      color: 颜色阈值          （手册 A）
      brain: Qwen2.5-VL 给框    （本手册）
```

**大脑只做一件事：图 → 一个框 + 一个名字。** 3D 坐标仍由深度 + 手眼算。
所以：**手眼没标定好，换成大脑一样抓不准**。Step 7 一点都不能省。

**为什么不让模型直接"看图给坐标/动作"**：3D 定位精度远不如"框中心 + 对齐深度"，
而且它错了你无法归因——是认错了？还是坐标算错了？分开就都能查。

---

## Step 1：上电与链路　⏱ 25 分钟

**与手册 A 完全一样**，命令照抄：

```bash
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash
python3 robot_code/arm_grasp/scripts/net_autoconf.py
```

```bash
# 终端 A
ros2 launch arm_grasp real_grasp.launch.py start_task:=false
# 终端 B
timeout 3 ros2 topic echo /joint_states
python3 robot_code/arm_grasp/scripts/check_real_hw.py --seconds 8
```

✅ `/joint_states` 6 个角在刷；体检 `结论：GO`
❌ 按 A 手册 Step 1 的四步排查（IP → 网线 → 使能 → 才怀疑代码）

> brain 模式下还要多确认一件事：`nvidia-smi` 里模型占的显存没把卡吃满（<7GB）。

---

## Step 2：夹爪两把尺　⏱ 25 分钟

**与手册 A 完全一样**（这一步和感知无关，但决定"夹不夹得住"）：

```bash
python3 robot_code/arm_grasp/scripts/gripper_node.py &
ros2 topic pub --once /gripper/cmd std_msgs/msg/String "data: 'home'"
#   → 卡尺量开口 = max_width_mm 真值，改配置
ros2 topic pub --once /gripper/cmd std_msgs/msg/String "data: 'grip:40'"
#   → 卡尺应 37±2mm；差 N mm → max_width_mm += N
timeout 3 ros2 topic echo /gripper/state      # 记下 [开口/角度/到位/堵转]
```

✅ `grip:40` 空夹 = 37±2mm；空夹时"到位=1 堵转=0"（或至少你知道本机的真实含义）
❌ 详见 A 手册 Step 2（47mm 的现象与公式那里写全了）

---

## Step 3：作业区实测　⏱ 5 分钟

```bash
python3 robot_code/arm_grasp/scripts/workspace_map.py
```

✅ 打印出稳定作业区（云机实测 x 0.12~0.40、y ±0.34、≈0.19 m²）
▶ 用记号笔在台面画出这个范围，方块/标定板/放置区都要落进去。
> **x/y 怎么对应到台上**：从**底座中心沿正前方**量出去是 +x，左右是 y（面向机械臂时左手为 +y）。
> 例：`x=0.30` = 底座正前方 30cm。

---

## Step 4：相机与深度　⏱ 15 分钟

```bash
timeout 5 ros2 topic hz /camera/d435i/color/image_raw
timeout 3 ros2 topic echo /camera/d435i/aligned_depth_to_color/image_raw
python3 robot_code/arm_grasp/scripts/check_real_hw.py --seconds 8
```

✅ 彩色 ≈30Hz；深度有数；有效深度 >30%、距离中值 0.3~0.6m

> **brain 模式下深度更要紧**：大脑只给 2D 框，3D 全靠这一路深度。
> 深度有效点少 → 框里没有可用深度 → 感知直接不发布。判据和 A 一样，但别将就。

---

## Step 5：大脑上线与验收　⏱ 35 分钟（**本手册的核心差异**）

### 7.1 确认服务端在跑

```bash
curl -s http://127.0.0.1:11434/v1/models | head -c 300
nvidia-smi --query-gpu=memory.used,memory.total --format=csv
```

✅ 有 JSON 返回；显存 5~6GB / 8GB

❌ `Connection refused`：
1. `ollama serve` 没起 → 手动起一个（或 `systemctl status ollama`）
2. 端口不是 11434 → 改配置 `brain.url` 的端口（LM Studio 是 1234）

### 7.2 自检：让模型认一次

```bash
# 用相机当前帧（终端 A 的 launch 要开着）
python3 robot_code/arm_grasp/scripts/brain.py --check
```

或者用存图（更省事，可以反复试）：

```bash
python3 robot_code/arm_grasp/scripts/brain.py --check --image ~/test_cube.jpg
```

✅ 打印：

```
 结果：OK 860ms
 标签：green cube
 框  ：[212, 168, 398, 332]  中心 (305, 250)
 耗时：860 ms（含网络）
```

**三条判据**：

| 项 | 合格线 | 不合格怎么办 |
|---|---|---|
| 框落在方块上 | 自己看图对一眼 | 换提示词 / 换更大模型（但 8GB 就 3B 了） |
| 延迟 | **<1.5s** | 降分辨率、`interval_s` 调大、换 Florence-2 |
| 标签 | 说得对 | `target_name` 写得更具体（"the small green cube on the table"） |

❌ `❌ 没拿到框`：模型说"没找到" → 图里可能真有多个/太小/太暗；换张图再试
❌ `看不穿服务端返回` → 你接的不是 OpenAI 兼容接口；确认 url 以 `/v1/chat/completions` 结尾

### 7.3 切到 brain（一条命令，自动备份）

```bash
python3 robot_code/arm_grasp/scripts/perception_mode.py brain --target "green cube"
```

它会同时改三处并检查一致性：

```
 ✅ perception.target_source → brain
 ✅ brain.enabled → true
 ✅ perception.target_name → "green cube"
 ✅ 大脑服务端 127.0.0.1:11434 可连接
```

❌ 报 `target_source=brain 但 brain.enabled=false` → 就是这个工具在帮你抓假配置，重跑一次

▶ 等价手工改法（`config/real_params.yaml`）：

```yaml
perception:
  target_source: brain          # ★
  target_name: "green cube"     # ★
brain:
  enabled: true                 # ★ 少这行会自动退回 color
  url: "http://127.0.0.1:11434/v1/chat/completions"
  model: "qwen2.5vl:3b"
  interval_s: 0.5
  max_age_s: 3.0
  fallback_to_color: true
```

### 7.4 重启 + 验收"真的在走大脑"

```bash
# 终端 A：Ctrl+C 重起（配置只在启动时读一次）
ros2 launch arm_grasp real_grasp.launch.py start_task:=false
# 终端 B
timeout 5 ros2 topic echo /grasp/target_xyz
```

✅ 终端 A 的日志里出现 **`大脑框命中`**：

```
目标顶面中心 base_link = [0.312 0.008 0.041] | 大脑框命中 15200 px / 有效深度 1402 点 / 距离中值 0.410 m | 样本12 抖动0.9mm
```

❌ 还是 `颜色命中` → 说明**退回了**。往上看一行 `大脑还没给出目标（...）`：
- `连不上 http://...` → 服务端没起
- `模型说没找到` → 认不出，换 `target_name` 或换图
- `最后一帧 5.2s 前` → 延迟太高，`interval_s` 调大或换小模型

> 退回不是错误，`fallback_to_color: true` 就是让任务别停。但**你要知道自己在用哪条**。

### 7.5 顺便做个颜色兜底（强烈建议）

即使主线用 brain，也请把颜色阈值调一次（5 分钟）：

```bash
python3 robot_code/arm_grasp/scripts/tune_perception.py --color green --write-config
```

这样万一现场大脑出问题，`perception_mode.py color` 一条命令就能继续干。

---

## Step 6：摆臂定观察位　⏱ 15 分钟

```bash
python3 robot_code/arm_grasp/scripts/jog.py --step 20
```

▶ 摆到「整片作业区在画面里、夹爪不挡、相机离桌面 0.35~0.5m」→ 按 `c` → 抄 `observe_joints` → `q` 退出

✅ dry-run 的 `最大关节跳变 < 1.0 rad`

> **brain 模式下观察位更重要**：方块必须在画面里够大（建议占画面 1/10 以上），
> 太小 VLM 容易认不出。摆的时候顺手看一眼方块在画面里的比例。

---

## Step 7：手眼标定　⏱ 90 分钟

**和手册 A 一模一样，一步都不能省**（大脑只给 2D，3D 全靠 X）。

```yaml
profile:
  mode: real
  allow_arm_motion: true       # 扫位要程序动臂
```

```bash
# 用键盘把臂领到「A4 板完整在画面中间、35~50cm」
python3 robot_code/arm_grasp/scripts/jog.py --step 20

# 自动扫位（约 1.5 分钟，15 个位姿）
python3 recovery/collect_handeye.py --sweep --marker-size 0.0595 --out ~/handeye_samples.json

# 质量
python3 robot_code/arm_grasp/scripts/handeye_quality.py --input ~/handeye_samples.json --focused

# 求解 + 自动写配置
python3 recovery/handeye_calib.py --input ~/handeye_samples.json --pairs all --write-config
```

✅ 采到 ≥8 组；quality 无 ❌；写配置成功；`相机在法兰系的位置` 在 5~25cm 量级
❌ 详见 A 手册 Step 7（每条错误都有对应动作）

---

## Step 8：出数验收　⏱ 15 分钟

```bash
ros2 launch arm_grasp real_grasp.launch.py start_task:=false    # 重启
timeout 5 ros2 topic echo /grasp/target_xyz
```

✅ 有数 + 与卷尺水平差 <5mm + 日志显示 `大脑框命中`

> **brain 特有的一点**：数字可能比 color 抖一点（每次推理的框不完全一样）。
> 看"抖动"那个数：**<2cm 就正常**，靠 `samples: 12` 的均值压住。>2cm 就把 `interval_s` 调小到 0.3。

❌ 数在跳但均值不收敛 → 框在两个物体之间来回，收窄 `target_name`
❌ 有框但没数 → 框里没深度（框太大包含了远处背景），把 `perception.depth_max_m` 调小到 0.6

---

## Step 9：打点验证　⏱ 20 分钟

```bash
python3 robot_code/arm_grasp/scripts/verify_handeye.py --points 5 --marker-size 0.0595
```

▶ 每点：确认能看到板 → 回车 → **键盘微调**（w/s/a/d，1/2/3 换步长）让指尖压住标记中心 → 回车
▶ 第 2~4 点：**挪 A4 板**（臂不动），重复

✅ 去均值残差 <5mm，输出一行 `xy_bias_m` → 填进 `perception` 段 → 重启终端 A

❌ 残差 >5mm → 按 A 手册 Step 8 的四步查（板尺寸 / tcp / 姿态激励 / 兜底）

> 注意：`xy_bias_m` 是加在**感知输出**上的，brain 和 color 共用同一个补偿，切模式不用重做。

---

## Step 10：放置点示教　⏱ 10 分钟

```bash
python3 robot_code/arm_grasp/scripts/jog.py --step 5      # 指尖碰台面中心
python3 robot_code/arm_grasp/scripts/teach_place_point.py
```

▶ 抄 `xy` / `surface_z` 进 `place` 段；`place.enabled` 先 **false**

✅ `✅ 数值范围看起来合理`

---

## Step 11：第一次抓取（先关 place）　⏱ 45 分钟

```yaml
place:
  enabled: false       # 只跑 8 阶段
arm:
  speed: 20
```

```bash
python3 robot_code/arm_grasp/scripts/preflight.py
python3 robot_code/arm_grasp/scripts/grasp_task.py --dry-run

# 空爪预演一次（看路径）
ros2 launch arm_grasp real_grasp.launch.py start_task:=true 2>&1 | tee ~/grasp_$(date +%H%M).log
```

✅ 走到 `[8/8]` 结束；路径没有横扫

▶ 放方块，正式抓 5 次，每次记一行（含"大脑标签"那一列）

✅ **5 次 ≥3 次**抓起来

**brain 特有的抓空原因**（和 color 不同，按这个顺序查）：

| 现象 | 原因 | 处理 |
|---|---|---|
| 框框住了方块+旁边的杯子 | 模型认宽了 | `target_name` 写更具体；或用 `--check` 看图确认框 |
| 框在方块上但中心偏一边 | 框不准 | 靠 `xy_bias_m` 补不了方向性偏差 → 换更大模型或退回 color |
| 框是对的但数在跳 | 推理抖动 | `interval_s` 0.5→0.3；`samples` 12→16 |
| 有时有数有时没数 | 延迟 > 帧时间 | 降分辨率 / 换 Florence-2 / `max_age_s` 3→5 |

❌ 一模一样的抓空（每次同向偏）→ 还是手眼/偏置问题，回 Step 9

---

## Step 12：开 place 跑完整任务　⏱ 40 分钟

```yaml
place:
  enabled: true
```

```bash
# 空爪先看转运路径
ros2 launch arm_grasp real_grasp.launch.py start_task:=true 2>&1 | tee ~/grasp_$(date +%H%M)b.log
# 再夹方块跑
```

✅ 看到 `[9/11] 转运到放置区` → `[10/11] 张开夹爪释放` → `[11/11] 撤离` → `✅ 抓取 + 放置流程执行完毕`

❌ 见 A 手册 Step 11 的六条（转运蹭桌面/落点偏/翻倒/带走/无 IK 解）

---

## Step 13：换词汇做泛化实验（这是 brain 的价值）　⏱ 50 分钟

⚠️ **只在抓取 5 次过 3 次、完整流程跑通过之后做**，否则你分不清是"识别坏了"还是"抓取坏了"。

```bash
# ① 换目标词，不动机器人，先看框对不对
python3 robot_code/arm_grasp/scripts/brain.py --check --image ~/test_red_block.jpg
python3 robot_code/arm_grasp/scripts/perception_mode.py brain --target "red block"
#   重启终端 A → 看 target_xyz 是否落在红方块上

# ② 换目标物（红方块 / 浅蓝方块 / 苹果 / 白色杯子）
```

**记录这张表**（这才是"泛化实验"的产出）：

| 目标词 | 场景 | 认出来 | 3D 落点误差 | 抓住 | 备注 |
|---|---|---|---|---|---|
| green cube | 绿方块居中 | ✅ | 4mm | ✅ | 基线 |
| red block | 红方块偏左 | | | | |
| apple | 苹果 | | | ⚠️ 尺寸假设不成立 | |

> ⚠️ **非立方体要改几何假设**：`box_half_size_m` 只描述 4cm 立方体。
> 认苹果可以，但"顶面中心 − 2cm"这个算法对球就不对，抓取点会偏。要真抓苹果，
> 得给它单独的尺寸/形状参数——那是下一次的事，别在今天做。

---

## Step 14：固化与回切　⏱ 30 分钟

### 16.1 固化

```bash
cd ~/robot_vla_ws
git add robot_code/arm_grasp/config/real_params.yaml
git commit -m "real: brain 模式现场参数（target_name / 手眼 / 夹爪 / 放置点）"
git push gitee main
cp ~/handeye_samples.json ~/handeye_samples_final.json
```

- [ ] `real_params.yaml`（含 `target_source: brain`、`target_name`、手眼、夹爪、place）
- [ ] 实验记录表（含"大脑标签"列）
- [ ] 成功视频（至少一段完整抓取+放置）
- [ ] 写清：哪些目标词能认、哪些不能

### 16.2 随时能回切（重要）

```bash
python3 robot_code/arm_grasp/scripts/perception_mode.py color
ros2 launch arm_grasp real_grasp.launch.py start_task:=false    # 重启
```

✅ 日志变回 `颜色命中`；`brain.enabled` 被一起关掉，显存释放

> 建议：**演示/交付时用 color**（延迟 0、可解释、不依赖模型服务）；
> brain 用于"换目标物"的探索。两条路随时一条命令切换。

---

## 故障速查（brain 专有 + 通用）

| 现象 | 先查 | 处理 |
|---|---|---|
| `Connection refused` | 服务端没起 | `ollama serve`；`curl .../v1/models` |
| 显存接近 8GB / 掉帧 | 模型太大 | 用 3B Q4；关掉不用的程序；别用 7B |
| 框不出现但模型有回复 | 返回不是 JSON | `brain.py` 已容错解析；仍失败就换提示词（要求只回 JSON） |
| 框住了错的物体 | `target_name` 太泛 | 写具体："the small green cube on the table" |
| 有数但一直跳 | 推理抖动 | `interval_s` 0.5→0.3；`samples` 12→16 |
| 延迟 >1.5s | 模型/分辨率 | 降分辨率、换 Florence-2、把 `max_age_s` 放宽 |
| 只有一侧有数（时有时无） | 延迟导致 `max_age_s` 过期 | 3.0 → 5.0，或 `interval_s` 调小 |
| 日志仍是 `颜色命中` | 退回 color 了 | 看上一行 `大脑还没给出目标（原因）` |
| 手眼/夹爪/放置相关 | — | **直接看手册 A 的第 16 节**（完全一样） |

---

## 一页纸速查卡（brain 版）

```bash
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash

# 0 大脑服务端（今晚就该拉好）
ollama pull qwen2.5vl:3b
curl -s http://127.0.0.1:11434/v1/models | head -c 200
nvidia-smi --query-gpu=memory.used,memory.total --format=csv     # 5~6GB / 8GB

# 1 链路
python3 robot_code/arm_grasp/scripts/net_autoconf.py
ros2 launch arm_grasp real_grasp.launch.py start_task:=false     # 终端A
timeout 3 ros2 topic echo /joint_states                           # 有数
python3 robot_code/arm_grasp/scripts/check_real_hw.py --seconds 8 # GO

# 2 夹爪（独占串口）
python3 robot_code/arm_grasp/scripts/gripper_node.py &
ros2 topic pub --once /gripper/cmd std_msgs/msg/String "data: 'home'"
ros2 topic pub --once /gripper/cmd std_msgs/msg/String "data: 'grip:40'"   # 卡尺 37±2

# 3 作业区
python3 robot_code/arm_grasp/scripts/workspace_map.py

# 4 大脑自检（不依赖相机也能测）
python3 robot_code/arm_grasp/scripts/brain.py --check --image ~/test_cube.jpg

# 5 切模式 + 验收
python3 robot_code/arm_grasp/scripts/perception_mode.py brain --target "green cube"
#   → 重启终端A → 调 target_xyz → 日志必须是「大脑框命中」
python3 robot_code/arm_grasp/scripts/tune_perception.py --color green --write-config   # 兜底

# 6 观察位
python3 robot_code/arm_grasp/scripts/jog.py --step 20            # 摆好 → c 打印 → 抄配置

# 7 手眼（先开 allow_arm_motion）
python3 recovery/collect_handeye.py --sweep --marker-size 0.0595 --out ~/handeye_samples.json
python3 robot_code/arm_grasp/scripts/handeye_quality.py --input ~/handeye_samples.json --focused
python3 recovery/handeye_calib.py --input ~/handeye_samples.json --pairs all --write-config
#   → 重启 → target_xyz 有数

# 8 打点
python3 robot_code/arm_grasp/scripts/verify_handeye.py --points 5 --marker-size 0.0595

# 9 放置点
python3 robot_code/arm_grasp/scripts/jog.py --step 5             # 指尖碰台面
python3 robot_code/arm_grasp/scripts/teach_place_point.py

# 10 抓取（place.enabled 先 false）→ 11 开 place 跑全套
python3 robot_code/arm_grasp/scripts/preflight.py
ros2 launch arm_grasp real_grasp.launch.py start_task:=true 2>&1 | tee ~/grasp_$(date +%H%M).log

# 回切（随时）
python3 robot_code/arm_grasp/scripts/perception_mode.py color
```

**判据（比 A 多一条）**

1. `/joint_states` 有数　　2. `grip:40` 空夹 37±2mm
3. 深度有效 >30%、0.3~0.6m　　4. `handeye_quality` 无 ❌
5. `verify_handeye` 残差 <5mm　　6. **`brain.py --check` 有框且 <1.5s**
7. 5 次抓取 ≥3 次放到位
