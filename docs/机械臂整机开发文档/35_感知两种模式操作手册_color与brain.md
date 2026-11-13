# 35 · 感知两种模式的操作手册（color 主线 / brain 可选）

> 2026-11-04 晚　配套：34 号（8 小时路线 v2）、30 号（移植手册）
>
> **想直接上手照着敲命令 → 看这两本：`36_操作手册A_颜色阈值color`、`37_操作手册B_识别大模型brain`。**
> 本文是"两种模式对比与原理"，36/37 是"逐步骤操作 + 每步判据"。
> 目标：**先用 color 把「抓取 + 放置」整套跑通**；brain（Qwen2.5-VL-3B）只做接通与自检，
> 等闭环稳了再切过去做开放词汇的泛化实验。

---

## 0. 先看清：两条路 90% 是一样的

```
                    ┌──────────────────────────────┐
   彩色图 ──────────►│  ① 找出目标像素（唯一分叉）  │
                    └──────────┬───────────────────┘
             color 模式        │        brain 模式
     颜色阈值 color_mask()     │     Qwen2.5-VL 给 bbox
                    └──────────┬───────────────────┘
                               ▼
   对齐深度 + 彩色内参 → 反投影成 p_cam → FK(q)·X → 基座系 p_base
                               ▼
                  顶面中心 → planner.plan_grasp → IK 直线下降
                               ▼
                   预夹 → 堵转夹持 → 抬升 → 转运 → 释放 → 撤离
```

**除了①，后面全部共用同一套代码。** 所以：
- color 能跑通 = 几何链、手眼、夹爪、放置全部验证过
- 切 brain 只是换掉①，**不会动** IK / 夹爪 / 放置
- brain 挂了会自动退回 color（`fallback_to_color: true`），任务不会中断

### 文件对应表（谁需要改）

| 文件 | 作用 | color | brain |
|---|---|---|---|
| `config/real_params.yaml` | 唯一配置源 | **改 3~5 行**（见 A1） | **改 3 行**（见 B2） |
| `scripts/perception_node.py` | 感知节点 | 不用改 | 不用改（自动起大脑线程） |
| `scripts/tune_perception.py` | 现场对准阈值 + 5 道门 | **要用**（唯一要人调的步骤） | 可选（只看 5 道门） |
| `scripts/perception_mode.py` | 一键切换 color/brain | 用一次 | 用一次 |
| `scripts/brain.py` | 大脑客户端 + 自检 | 不用 | **要用**（`--check`） |
| 模型服务（Ollama/LM Studio/vLLM） | 跑 Qwen2.5-VL | 不用 | **要装**（不是仓库文件） |
| `scripts/check_real_hw.py` | 在线体检 | 用 | 用 |
| `scripts/grasp_task.py` / `planner.py` | 规划与状态机 | 不用改 | 不用改 |

---

## 1. 路线 A：color（明天主线，先跑通这条）

### A1 要改的配置（就这几行）

打开 `config/real_params.yaml`，`perception:` 段：

```yaml
perception:
  target_source: color          # ★ 必须是 color
  target_color: green           # 抓哪个颜色：green | red | blue
  color_thresholds:             # 现场对准后由 tune_perception --write-config 自动写
    green: {min: 100, diff_r: 40, diff_b: 40}
    red:   {min: 100, diff_g: 40, diff_b: 40}
    blue:  {min: 100, diff_r: 30, diff_g: 20}
  box_half_size_m: 0.02         # 方块必须是 4cm，别改
  min_pixels: 30
  depth_min_m: 0.20
  depth_max_m: 0.90
  samples: 12
  jitter_max_m: 0.02
```

另外两处（跟感知无关，但影响"能不能跑通"）：

| 位置 | 值 | 为什么 |
|---|---|---|
| `profile.allow_arm_motion` | `true` | 第二道锁，不开臂不动 |
| `place.enabled` | 先 `false`，抓取稳了再 `true` | 第一次跑先把"抓"跑通，别一上来就走完整 11 阶段 |

> 一次性切换：`python3 robot_code/arm_grasp/scripts/perception_mode.py color`
> 它会同时把 `target_source` 设成 color、`brain.enabled` 设成 false，
> 并检查有没有"配了但没生效"的状态。

### A2 起链路，确认 5 道门（10 分钟）

```bash
cd ~/robot_vla_ws && source /opt/ros/foxy/setup.bash

# 终端 A：起链路（不自动抓）
ros2 launch arm_grasp real_grasp.launch.py start_task:=false

# 终端 B：先看 5 道门（彩色图 / 对齐深度 / 内参 / 关节角 / 手眼）
python3 robot_code/arm_grasp/scripts/tune_perception.py --check-only --seconds 8
```

门 1~4 必须全绿；门 5（手眼）在做完标定前不亮是正常的——**color 模式也一样要手眼**，
因为 `p_base = FK·X·p_cam` 里的 X 就是手眼矩阵。颜色只解决"哪些像素是目标"。

### A3 对准颜色阈值（唯一需要人调的步骤，5 分钟）

```bash
# 把绿方块放到作业区，让它在彩色图里完整可见
python3 robot_code/arm_grasp/scripts/tune_perception.py --color green --write-config
```

窗口里三个滑块：

| 滑块 | 含义 | 怎么调 |
|---|---|---|
| `min` | 绿色通道最小值 | 调高 → 只留最绿的；调低 → 变宽容 |
| `lead1` / `lead2` | 比红/蓝高多少 | 调高 → 排除偏黄/偏青的像素 |

目标：**方块被完整高亮、背景一片干净**。按 `s` 保存 ← 加了 `--write-config` 会
**直接写进 real_params.yaml 的 `color_thresholds.green`**（自动备份），不用手抄。
按 `q` 退出。

> 没有显示器的场合：`--headless --seconds 20`，它只打印统计，不给滑块。

**然后必须重启感知**（配置只在启动时读一次）：

```bash
# 终端 A：Ctrl+C 停掉，再起
ros2 launch arm_grasp real_grasp.launch.py start_task:=false
```

### A4 验收"认出来了"（2 分钟）

```bash
timeout 5 ros2 topic echo /grasp/target_xyz
```

同时看终端 A 里感知节点那行日志：

```
目标顶面中心 base_link = [0.312 0.008 0.041] | 颜色命中 1832 px / 有效深度 1755 点 / 距离中值 0.412 m | 样本12 抖动0.8mm
```

四条都要对：

| 检查 | 合格线 |
|---|---|
| 数值在刷新 | 每秒几次，不跳 |
| 与卷尺量的水平位置 | 差 **<5mm** |
| `颜色命中` | 几百~几千 px（<30 会报"只命中 N 像素"） |
| `距离中值` | 0.3~0.6m（<0.28 是 D435i 盲区） |

### A5 跑完整任务（抓 + 放）

```bash
# ① 先空跑状态机（不动硬件），确认 11/8 阶段都能过
python3 robot_code/arm_grasp/scripts/preflight.py
python3 robot_code/arm_grasp/scripts/grasp_task.py --dry-run

# ② 空爪预演一次（夹爪不夹东西，但整条路径会真的走）
ros2 launch arm_grasp real_grasp.launch.py start_task:=true 2>&1 | tee ~/grasp_$(date +%H%M).log

# ③ 放上方块，正式跑（place.enabled=false 时是 8 阶段：抓完停在抬升终点）
ros2 launch arm_grasp real_grasp.launch.py start_task:=true 2>&1 | tee ~/grasp_$(date +%H%M)a.log

# ④ 抓取稳定后，把 place.enabled 改 true，再跑一次（11 阶段，含转运/释放/撤离）
```

> 为什么先 `place.enabled: false`：放置段会多走三段笛卡尔转运，如果放置点还没示教好，
> 会把"抓不到"和"放不好"两个问题混在一起。**先单独把抓跑稳，再叠加放置。**

### A6 color 模式专有的故障（其它故障见 34 号故障树）

| 现象 | 原因 | 修 |
|---|---|---|
| `颜色分割只命中 0 像素` | 阈值不对 / `target_color` 选错 / 光照变了 | 重跑 `tune_perception.py --color xxx --write-config` |
| 命中很多像素但目标是背景 | 桌面有大片同色 | 钉死颜色（别用 auto）+ 调高 `lead` |
| 命中 N 像素但有效深度 0 | 目标 <0.28m / 反光 / 对齐深度没开 | 抬高观察位 / 查 `align_depth` |
| 数字一直跳 | 自动曝光在变 | 固定光源、目标别放画面边缘、`jitter_max_m` 放宽到 0.03 |
| 红/浅蓝方块认不出 | `target_color` 还是 green | 改 `target_color` + 对准那套阈值（三种颜色各一套） |

### A7 color 模式要用到的文件（就这 3 个）

```
config/real_params.yaml            ← 改 perception 那几行
scripts/tune_perception.py         ← 现场对准（--write-config 写回配置）
scripts/perception_node.py         ← 不用改，读配置跑
```

---

## 2. 路线 B：brain（Qwen2.5-VL-3B，等 A 稳了再做）

### B1 装模型（三条命令，10 分钟）

```bash
# Ollama 最省事；装完确认显存占用（应 ≈5~6GB，8GB 卡留得下相机）
ollama pull qwen2.5vl:3b
ollama list
curl -s http://127.0.0.1:11434/v1/models | head -c 300
```

> 用 LM Studio：起 Local Server（端口 1234），url 换成
> `http://127.0.0.1:1234/v1/chat/completions`。
> 用 vLLM：`vllm serve Qwen/Qwen2.5-VL-3B-Instruct --port 8000`（要 conda py3.10）。
> **别在本机跑 7B**：Q4 就占 ~7.1GB（90% 显存），抓取时会和相机抢资源。

### B2 一键切换（改 3 行，自动备份）

```bash
# 切到大脑，并指定要认什么
python3 robot_code/arm_grasp/scripts/perception_mode.py brain --target "green cube"

# 它会同时改：perception.target_source=brain、brain.enabled=true、perception.target_name
# 并打印一致性检查（服务端连不连得上）
```

等价的手工改法（`config/real_params.yaml`）：

```yaml
perception:
  target_source: brain          # ★
  target_name: "green cube"     # ★ 要认什么；英文通常更准
brain:
  enabled: true                 # ★ 少了这行，启动会自动退回 color
  url: "http://127.0.0.1:11434/v1/chat/completions"
  model: "qwen2.5vl:3b"
  fallback_to_color: true
```

### B3 自检（切之前就能跑，不需要机械臂）

```bash
# 用相机当前帧
python3 robot_code/arm_grasp/scripts/brain.py --check
# 或用一张存图（相机没起也能测）
python3 robot_code/arm_grasp/scripts/brain.py --check --image ~/test_cube.jpg
```

它打印：能不能连通、模型回了什么、**框的坐标**、**耗时**。
判据：

| 项 | 合格线 |
|---|---|
| 框中心 | 落在方块上（自己看一眼图） |
| 耗时 | **<1.5s**；>1.5s 抓取会跟不上（降分辨率或换更小模型） |
| 标签 | 说得对（"green cube" / "cube"） |

### B4 起链路验收

```bash
ros2 launch arm_grasp real_grasp.launch.py start_task:=false
timeout 5 ros2 topic echo /grasp/target_xyz
```

日志里会变成：

```
大脑：green cube 框 [212 168 398 332]（0.6s 前）| 大脑框命中 15200 px / 有效深度 ...
```

看到 **`大脑框命中`** 才说明真的走了大脑；仍显示 `颜色命中` 就是退回了（看 `brain_miss` 那行原因）。

### B5 brain 与 color 的差异（心里要有数）

| 维度 | color | brain |
|---|---|---|
| 延迟 | ~0ms | 0.3~2s（所以 `samples` 均值会慢半拍） |
| 稳定性 | 阈值定死就稳 | 每次推理可能有抖动，靠 `jitter_max_m` 兜 |
| 能认什么 | 三种纯色 | 开放词汇："red block" / "apple" / "a white cup" |
| 出错怎么查 | 看 mask 对不对 | 看框对不对（`--check` 存图看） |
| 显存 | 0 | 5~6GB（相机 pipeline 也在同一张卡上） |

> brain 模式下 `box_half_size_m` **仍然是 4cm 立方体假设**（用来算中心与预夹开口）。
> 换成苹果/杯子这类非立方体，要另给尺寸参数——那是"泛化实验"的事，别在明天做。

### B6 切回 color（一条命令）

```bash
python3 robot_code/arm_grasp/scripts/perception_mode.py color
ros2 launch arm_grasp real_grasp.launch.py start_task:=false    # 重启才生效
```

`brain.enabled` 会被一起关掉，释放显存。

### B7 brain 模式要用到的文件

```
config/real_params.yaml            ← perception.target_source / target_name + brain 段
scripts/brain.py                   ← 客户端 + --check 自检
scripts/perception_mode.py         ← 一键切换（避免改了这头忘那头）
scripts/perception_node.py         ← 不用改（自动起线程、掉线回退）
模型服务（Ollama/LM Studio/vLLM）  ← 仓库外，单独装
```

---

## 3. 三种"看起来配了、其实没生效"

| 状态 | 表现 | 怎么发现 |
|---|---|---|
| `target_source: brain` 但 `brain.enabled: false` | 启动时自动退回 color，**任务照跑**，你以为在用大脑 | `perception_mode.py`（无参数）会直接标 ❌；感知启动日志有一行 warn |
| 只改 `brain.enabled: true`，`target_source` 还是 color | 大脑连上了但没人调用，白占 5~6GB 显存 | 同上，标 ⚠️ 建议一起关 |
| 改完配置没重启 launch | 一切看起来没变化 | 配置只在启动时读一次；改完必须重启 |

`perception_mode.py` 不带参数就是专门查这三个：

```bash
python3 robot_code/arm_grasp/scripts/perception_mode.py
```

---

## 4. 明天的顺序（一句话版）

```bash
# ---------- color 主线 ----------
python3 robot_code/arm_grasp/scripts/net_autoconf.py
ros2 launch arm_grasp real_grasp.launch.py start_task:=false      # 终端 A
python3 robot_code/arm_grasp/scripts/perception_mode.py color
python3 robot_code/arm_grasp/scripts/tune_perception.py --check-only --seconds 8
python3 robot_code/arm_grasp/scripts/tune_perception.py --color green --write-config
#   ↑ 按 s 保存 → 重启终端 A
timeout 5 ros2 topic echo /grasp/target_xyz                        # 必须和尺子差 <5mm
# 然后按 34 号第 8 节继续（手眼 → 抓取 → 放置）

# ---------- 等抓取 5 次里过 3 次之后，再碰 brain ----------
ollama pull qwen2.5vl:3b
python3 robot_code/arm_grasp/scripts/brain.py --check
python3 robot_code/arm_grasp/scripts/perception_mode.py brain --target "green cube"
#   ↑ 重启终端 A → 看日志里是不是「大脑框命中」
```

**记住一句话**：color 和 brain 只差"哪几个像素是目标"，
后面的深度、手眼、IK、夹持、放置**完全同一套**——
所以 color 跑通的那一刻，整套任务的 90% 就已经验证完了。
