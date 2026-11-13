# RECOVER —— 关键改动的手动恢复指令（最后一道防线）

> 为什么需要这份文档：
> 仓库根 `.gitignore` 第 1 行是 `src/ros2_rm_robot/`，**整个第三方包目录被 git 忽略**。
> 所以「手指 effort 接口」和「effort 控制器」这两处最关键的改动，git 永远存不住。
>
> 三道防线：
> 1. **自动**：`bash sim_grasp/scripts/apply_patches.sh`（首选）
> 2. **半自动**：`sim_grasp/patches/` 里有改好的完整文件，`cp` 过去即可
> 3. **纯手写**（本文档）：不依赖任何备份文件，照着改 / 粘贴脚本即恢复 ← 你现在看的这条
>
> 本文档本身在 git 里（`sim_grasp/RECOVER.md`），所以只要仓库在，恢复方法就在。

---

## 快速判断：需不需要恢复

```bash
grep -A1 'finger_left_joint' ~/robot_vla_ws/src/ros2_rm_robot/rm_gazebo/config/gazebo_eco65_description.urdf.xacro | grep command_interface
```

- 输出 `command_interface name="effort"`  → **正常，不用恢复**
- 输出 `command_interface name="position"` → **需要恢复**

---

## 改动 A：手指关节改 effort 接口

**文件**：`~/robot_vla_ws/src/ros2_rm_robot/rm_gazebo/config/gazebo_eco65_description.urdf.xacro`

⚠️ **只改 `finger_left_joint` 和 `finger_right_joint` 两个块，joint1~joint6 必须保持 position！**

把这两个块从：

```xml
        <joint name="finger_left_joint">
            <command_interface name="position"/>
            <state_interface name="position">
                <param name="initial_value">0.35</param>
            </state_interface>
            <state_interface name="velocity"/>
        </joint>
```

改成（右手同理，`initial_value` 是 `-0.35`）：

```xml
        <joint name="finger_left_joint">
            <command_interface name="effort"/>
            <state_interface name="position">
                <param name="initial_value">0.35</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
```

**一键版**（复制粘贴即可，只动 finger 块）：

```bash
python3 - <<'EOF'
import re
P = '/home/ubuntu/robot_vla_ws/src/ros2_rm_robot/rm_gazebo/config/gazebo_eco65_description.urdf.xacro'
s = open(P).read()
def fix(m):
    b = m.group(0)
    b = b.replace('<command_interface name="position"/>', '<command_interface name="effort"/>')
    if '<state_interface name="effort"/>' not in b:
        b = b.replace('<state_interface name="velocity"/>',
                      '<state_interface name="velocity"/>\n            <state_interface name="effort"/>')
    return b
s2 = re.sub(r'<joint name="finger_(?:left|right)_joint">.*?</joint>', fix, s, flags=re.S)
assert s2 != s, '没有改动，检查文件是否已改过'
open(P, 'w').write(s2)
print('OK: 手指已改为 effort 接口')
EOF
```

---

## 改动 B：控制器改 effort 类型

**文件**：`~/robot_vla_ws/src/ros2_rm_robot/rm_moveit2_config/rm_eco65_config/config/ros2_controllers.yaml`

两处替换 + 一处新增：

| 位置 | 原 | 改为 |
|---|---|---|
| `controller_manager` 段 | `type: position_controllers/JointGroupPositionController` | `type: effort_controllers/JointGroupEffortController` |
| `gripper_controller` 段 | `command_interfaces:` 下 `- position` | `- effort` |
| `gripper_controller` 段 | `state_interfaces:` 只有 `- position` | 追加一行 `- effort` |

改完的 `gripper_controller` 两段应该是这样：

```yaml
    gripper_controller:
      type: effort_controllers/JointGroupEffortController
```

```yaml
gripper_controller:
  ros__parameters:
    joints:
      - finger_left_joint
      - finger_right_joint
    command_interfaces:
      - effort
    state_interfaces:
      - position
      - effort
```

**一键版**：

```bash
python3 - <<'EOF'
P = '/home/ubuntu/robot_vla_ws/src/ros2_rm_robot/rm_moveit2_config/rm_eco65_config/config/ros2_controllers.yaml'
s = open(P).read()
s = s.replace('type: position_controllers/JointGroupPositionController',
              'type: effort_controllers/JointGroupEffortController')
i = s.find('gripper_controller:\n  ros__parameters:')
seg = s[i:]
seg = seg.replace('command_interfaces:\n      - position', 'command_interfaces:\n      - effort')
seg = seg.replace('state_interfaces:\n      - position', 'state_interfaces:\n      - position\n      - effort')
open(P, 'w').write(s[:i] + seg)
print('OK: 控制器已改为 effort')
EOF
```

---

## 改动 C：夹爪几何 / 惯量 / 插件（此文件 git 已跟踪，列出来是为了完整性）

**文件**：`~/robot_vla_ws/src/arm_perception/urdf/eco65_camera.urdf.xacro`

| 项 | 原值 | 应为 | 为什么 |
|---|---|---|---|
| 手指惯量 | `ixx="1.607e-6" iyy="5.667e-6" izz="4.273e-6"` | `ixx="6.0e-4" iyy="6.0e-4" izz="1.0e-3"` | 必须含**电机经减速器折算惯量** `J=N²·J_rotor`（N=50）。原值只有裸连杆，0.03 N·m 下 α≈7000 rad/s²，100Hz 控制必然数值振荡 |
| 关节阻尼 | `<dynamics damping="0.002" friction="0.01"/>` | `damping="0.01"` | 齿轮箱粘性摩擦 |
| grasp_fix 的 palm_link | `<palm_link>gripper_base</palm_link>` | `<palm_link>Link6</palm_link>` | **URDF→SDF 会合并固定关节**，`gripper_base` 在 Gazebo 里不存在 |
| grasp_fix 插件块 | 无 | 见下 | 吸附用 |

插件块（插到 `</robot>` 之前）：

```xml
<gazebo>
  <plugin name="gazebo_grasp_fix" filename="libgazebo_grasp_fix.so">
    <arm>
      <arm_name>rm_eco65_gripper</arm_name>
      <palm_link>Link6</palm_link>
      <gripper_link>finger_left</gripper_link>
      <gripper_link>finger_right</gripper_link>
    </arm>
    <forces_angle_tolerance>120</forces_angle_tolerance>
    <update_rate>4</update_rate>
    <grip_count_threshold>2</grip_count_threshold>
    <max_grip_count>8</max_grip_count>
    <release_tolerance>0.005</release_tolerance>
    <disable_collisions_on_attach>true</disable_collisions_on_attach>
    <contact_topic>__default_topic__</contact_topic>
  </plugin>
</gazebo>
```

---

## 改动 D：gazebo_grasp_fix 插件源码（4 处，在 `~/gazebo-pkgs`）

如果 `~/gazebo-pkgs` 丢了：`git clone https://github.com/JenniferBuehler/gazebo-pkgs.git ~/gazebo-pkgs`
然后应用补丁 `sim_grasp/patches/gazebo_grasp_fix.patch`，或手工改这 4 处：

| # | 文件 | 改什么 | 为什么 |
|---|---|---|---|
| 1 | `src/GazeboGraspGripper.cpp` | `link->GetChildCount()` → `link->GetCollisions().size()` | 前者取的是子 link 数（常为 0），收集不到碰撞体 |
| 2 | `src/GazeboGraspFix.cpp` | 订阅话题改为 `"/gazebo/" + world->Name() + "/physics/contacts"` | 本环境 `CreateFilter` 永远收不到接触（实测 hits=0） |
| 3 | `src/GazeboGraspFix.cpp` | 两侧都不是夹爪碰撞体时补 `else { continue; }` | 否则空指针 + 除零（改订阅全量话题后必加） |
| 4 | `src/GazeboGraspFix.cpp` | 力判据之外补几何判据：两个不同夹爪 link 同时接触 = 对捏成立 | 本版本 ODE 接触力反馈常有一侧恒为 0 |

编译：

```bash
cd ~/gazebo-pkgs/gazebo_grasp_plugin
protoc --cpp_out=msgs -I msgs msgs/grasp_event.proto      # 只需一次
bash ~/robot_vla_ws/sim_grasp/scripts/build_plugin.sh
```

---

## 恢复后验证（4 条命令，必须全绿）

```bash
cd ~/robot_vla_ws
# 1. effort 接口在文件里
grep -q 'command_interface name="effort"' src/ros2_rm_robot/rm_gazebo/config/gazebo_eco65_description.urdf.xacro && echo "✅ 手指 effort 接口" || echo "❌"
# 2. effort 控制器在文件里
grep -q 'JointGroupEffortController' src/ros2_rm_robot/rm_moveit2_config/rm_eco65_config/config/ros2_controllers.yaml && echo "✅ effort 控制器" || echo "❌"
# 3. 手指等效惯量
grep -q 'izz="1.0e-3"' src/arm_perception/urdf/eco65_camera.urdf.xacro && echo "✅ 手指等效惯量" || echo "❌"
# 4. palm_link 是 Link6
grep -q 'palm_link>Link6' src/arm_perception/urdf/eco65_camera.urdf.xacro && echo "✅ palm_link=Link6" || echo "❌"
```

**运行时验证**（起仿真后）：

```bash
ros2 control list_hardware_interfaces | grep finger      # 应有 finger_left_joint/effort [claimed]
ros2 control list_controllers | grep gripper             # 应是 effort_controllers/JointGroupEffortController
gz topic -l | grep grasp_events                          # 插件在工作
```

最后跑一遍闭环：`bash ~/robot_vla_ws/sim_grasp/scripts/run_eff.sh`
