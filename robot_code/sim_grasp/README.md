# sim_grasp —— 硬件对齐仿真抓取闭环（可重建资产）

> 目标物：边长 4cm 绿色方块　对象：ECO65-6FB + MS42DC 二指夹爪 + 腕部深度相机
> 环境：Ubuntu 20.04 + ROS2 Foxy + Gazebo 11
> 状态：**闭环已跑通一次**（dz = +0.0978 m），但**尚不可复现**（见文末已知问题）

---

## 两条命令

```bash
# ① 恢复（换机器 / 重装 ros2_rm_robot 后必跑）
bash ~/robot_vla_ws/sim_grasp/scripts/apply_patches.sh

# ② 运行（约 4~5 分钟）
bash ~/robot_vla_ws/sim_grasp/scripts/run_eff.sh
tail -20 /tmp/eff_run.log        # 看 dz
```

成功判据：`dz = +0.09xx m -> ★ 夹起来了！`

---

## 目录结构

```
sim_grasp/
├── scripts/
│   ├── apply_patches.sh   ← 一键恢复（最重要，换机器先跑这个）
│   ├── run_eff.sh         ← 一键跑完整闭环：清场→起仿真→生成方块→抓取
│   ├── eff_test.py        ← 主力测试：感知→规划→下降→夹持→抬升
│   ├── cleanup.sh         ← 彻底清场（含 zombie robot_state_publisher 处理）
│   ├── build_plugin.sh    ← 编译 gazebo_grasp_fix 插件
│   ├── tau_probe.py       ← 力矩探针：验证 effort 接口是否真的能动
│   ├── eff_check.sh       ← 只起仿真，打印硬件接口/控制器状态
│   └── adapt_test.py      ← 旧版（position 驱动），留作对照
├── config/
│   ├── grasp.world        ← 调好的物理世界（步长 0.0002s、ODE iters 300）
│   └── eco65_6fb.urdf     ← IK 用的运动学模型
└── patches/
    ├── gazebo_eco65_description.urdf.xacro   ← 手指 effort 接口
    ├── ros2_controllers.yaml                 ← effort 控制器
    └── gazebo_grasp_fix.patch                ← 插件源码 4 处适配
```

---

## ⚠️ 为什么有 `patches/` 这个目录（必读）

仓库根的 `.gitignore` 第 1 行是 `src/ros2_rm_robot/`，
**整个第三方包目录被 git 忽略**，所以下面两处关键改动 git 永远存不下来：

1. **手指 effort 接口**（`gazebo_eco65_description.urdf.xacro`）
2. **effort 控制器类型**（`ros2_controllers.yaml`）

它们以补丁形式存在 `patches/`，靠 `apply_patches.sh` 回填。
**新增任何对 `src/ros2_rm_robot/` 的改动，都必须同时复制一份到 `patches/` 并在恢复脚本里加一行。**

---

## 关键参数（在 `scripts/eff_test.py` 顶部）

| 参数 | 值 | 说明 |
|---|---|---|
| `OPEN_T` | 0.20 rad | 张开目标角（开口 58mm） |
| `PRE_T` | 0.04 rad | 预夹角（开口 42mm，单边留 1mm） |
| `TAU_MOVE` | 0.020 N·m | 移动手指的恒定力矩 |
| `TAU_GRIP` | 0.030 N·m | **夹持力矩** → 指尖 0.6 N/侧 |
| `KD` | 0.030 | 阻尼项，限速 ≈ 0.67 rad/s |
| `LIFT` | 0.10 m | 抬升高度 |

夹持力余量核算：方块重力 0.59 N，μ≈1.5 → 需要 0.20 N/侧，当前 0.6 N/侧（3 倍余量）。

---

## 三个必须记住的坑

1. **僵尸 robot_state_publisher**：Linux comm 截断到 15 字符，`pkill -x robot_state_publisher` **从未生效**。
   旧进程用粘滞 QoS 抢发旧 URDF → 改什么都不生效。必须 `pkill -9 -f "[/]robot_state_publisher/robot_state_publisher"`。
2. **position 接口不是控制，是瞬移**：关节坐标被直接改写，`<limit effort>` 管不住，ODE 不产生接触力 → 夹不住也触发不了插件。
3. **URDF→SDF 会合并固定关节**：`gripper_base` 在 Gazebo 里不存在，手指父 link 是 `Link6`，
   所以 `palm_link` 必须写 `Link6`。

---

## 已知问题

| 问题 | 状态 |
|---|---|
| 抓取结果不可复现（成功 1 次 / 失败 1 次） | ⏳ 怀疑 `disable_collisions_on_attach=true` 自相矛盾：吸附后禁碰撞 → 接触消失 → 被松开 |
| 吸附后手指会"穿过"方块关到限位 | 插件伪影（真机不会），因吸附后禁用了碰撞 |

详细分析与面试素材：`~/../22_仿真抓取问题汇总与面试资料.md`（Mac 本地 work1 目录）。
