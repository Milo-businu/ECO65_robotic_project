# 24｜RealSense D435i 真机安装方案（版本钉死 + 分步验证）

> 决策：**选 D435i**（用户 2026-10-28 定，要通用性 + RGB 1080p 利于 π0）
> 环境：Ubuntu 20.04 + ROS2 Foxy（EOL）　前置：`23_RealSense选型对照与真机准备清单.md`
> 编写时间：2026-10-28

---

## 一、最重要的一条：锚点是「固件版本」，不是型号

型号定了（D435i）还不够，因为**同一型号不同批次出厂固件不同**。
三个组件必须互相咬合：

```
   相机固件（出厂就定了，是锚）
        │
        ├──▶ librealsense SDK 版本（必须支持该固件）
        │
        └──▶ realsense-ros Wrapper 版本（必须支持该 SDK + 兼容 Foxy）
```

**所以第一步不是装软件，是先读出出厂固件版本。**

```bash
# 在真机上执行（还没装 SDK 也能看 USB 设备）
lsusb | grep -i intel
# 装了 SDK 后：
rs-enumerate-devices 2>/dev/null | grep -i firmware || echo "SDK 未装，先看 lsusb"
```

把输出告诉我，我据此给你**唯一的**版本组合。

---

## 二、候选组合（两个，按固件版本选）

### 组合 A：官方 Foxy 原生版（最稳，推荐先试）

来源：IntelRealSense/realsense-ros **release 3.1.3**，标题明确写着
"ROS2-Foxy Wrapper for Intel RealSense Devices"，官方声明：

| 组件 | 版本 | 官方说明 |
|---|---|---|
| librealsense | **v2.41.0** | "Supported RealSense SDK: LibRealSense v2.41.0" |
| realsense-ros | **3.1.3**（`foxy` 分支） | "Supported ROS Distributions: Foxy" |
| D400 固件 | **5.12.10.0** | 官方推荐；且注明该固件需要 librealsense ≥ v2.39.0 |

优点：官方唯一明确标注 Foxy 的版本，踩坑概率最低。
缺点：较老（2021 年前后），某些新特性没有。

### 组合 B：社区实践验证版（功能更新）

来源：多篇实践帖（2024 前后）验证过的 D435i 组合：

| 组件 | 版本 | 备注 |
|---|---|---|
| librealsense | **2.50.0** 或 **2.51.1** | 必须从源码编译 |
| realsense-ros | **2.3.2** / **4.51.1** | 注意 ROS1/ROS2 分支别拿错 |
| D400 固件 | **5.13.0.50** | 多篇帖子的共同推荐值 |

⚠️ **已知冲突**：固件 **5.16.0.1** + librealsense **2.51.1** 会报
`hwmon command failed (HW not ready)`（深度自动曝光/增益设置失败）。
解决二选一：**固件降到 5.13.0.50**（推荐，改动小）或 **SDK 升到 2.55.1+**。

---

## 三、安装步骤（Ubuntu 20.04，RSUSB 后端）

> **为什么用 RSUSB**：Ubuntu 20.04 上走 RSUSB 用户态后端，
> 不需要给内核打 uvcvideo 补丁，**不动系统内核 = 不破坏已验收的机械臂环境**。
> 这条是硬约束。

### 步骤 1：装依赖

```bash
sudo apt-get update
sudo apt-get install -y git cmake build-essential libssl-dev libusb-1.0-0-dev \
    pkg-config libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev
```

### 步骤 2：源码编译 librealsense（按第一、二节选定的版本）

```bash
cd ~
git clone https://github.com/IntelRealSense/librealsense.git
cd librealsense
git checkout v2.41.0          # ← 换成你选定的版本标签
mkdir build && cd build
cmake .. -DFORCE_RSUSB_BACKEND=ON \
         -DCMAKE_BUILD_TYPE=Release \
         -DBUILD_EXAMPLES=true \
         -DBUILD_GRAPHICAL_EXAMPLES=false
make -j$(nproc)
sudo make install
```

### 步骤 3：每步都验证（别"装完以为好了"）

```bash
rs-enumerate-devices          # 必须能看到 D435i + 固件版本号
realsense-viewer              # 能看到深度和彩色画面（有桌面时）
```

**只要这一步看不到设备，后面全白做，先解决它。**

### 步骤 4：装 realsense-ros Wrapper

```bash
mkdir -p ~/realsense_ws/src && cd ~/realsense_ws/src
git clone https://github.com/IntelRealSense/realsense-ros.git -b foxy
cd ~/realsense_ws
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
source install/local_setup.bash
```

> 注意：`ros2-master` 分支也可，但 `-b foxy` 分支与 Foxy 对齐性更好。
> 若 colcon build 报错（新版弱化 Foxy 支持），回退到 release 3.1.3 标签。

### 步骤 5：验证出流

```bash
ros2 launch realsense2_camera rs_launch.py enable_pointcloud:=true

# 另开终端
ros2 topic hz /camera/color/image_raw            # 应有 ~30Hz
ros2 topic hz /camera/depth/image_rect_raw       # 应有 ~30Hz
timeout 5 ros2 topic echo /camera/depth/image_rect_raw   # 数值不能全 0
```

**深度全 0 = 固件/SDK 不匹配**，回到第二节换组合。

---

## 四、与本项目相关的配置要点

| 项 | 设置 | 原因 |
|---|---|---|
| IMU | **关闭** | 本项目不用 IMU，且 D435i 的 IMU 话题在 Foxy 下易出配置问题 |
| 多相机 | 用 `serial_no` 区分 | 数字串序列号要加下划线前缀，如 `_831612073525` |
| TF 发布 | 发 `flange → d435i_link`，**不要发到 `optical_frame`** | 否则与 realsense 自己那条边形成双父冲突 |
| 自动曝光 | **锁定** | 否则机械臂移动时图像分布漂移，影响 VLA 输入 |
| 点云 | `enable_pointcloud:=true` | 定位需要 |
| USB | 必须 USB3，走拖链 + 应力释放 | 腕装线缆随臂弯折是主要故障源 |

---

## 五、故障速查

| 现象 | 大概率原因 | 处理 |
|---|---|---|
| `rs-enumerate-devices` 看不到设备 | 用了 USB2 线 / 集线器供电不足 | 换 USB3 直插，检查供电 |
| 能看到设备但深度全 0 | 固件与 SDK 不匹配 | 降固件到 5.13.0.50，或升 SDK 到 2.55.1+ |
| `hwmon command failed` | 固件 5.16.x + 旧 SDK | 同上 |
| `colcon build` 报 Foxy 不兼容 | wrapper 版本太新 | 回退 release 3.1.3 |
| 出流但 `/tf_static` 冲突 | TF 发到了 optical_frame | 改发 `flange → d435i_link` |
| 图像忽明忽暗 | 自动曝光未锁 | 锁定曝光/白平衡 |

---

## 六、需要你反馈的信息（我据此给你唯一组合）

在真机上跑下面两条，把输出贴给我：

```bash
lsusb | grep -i intel
cat /etc/os-release | grep -E 'VERSION=' ; echo "ROS: $ROS_DISTRO"
```

如果是新买的 D435i，出厂固件多半是 **5.15/5.16 或更高**。
那样我会建议你走 **librealsense 2.55.1+**（不用降级固件），因为降级固件有风险。
