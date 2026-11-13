# 安全说明

## 这个仓库里有什么、没有什么

**没有**：密码、SSH 私钥、GitHub/Gitee 令牌、云服务器地址、个人邮箱、运行日志。
**有**：机器人控制代码、配置模板、标定脚本、部署文档。

如果发现仍有敏感信息残留，请开 Issue 说明位置（**不要在 Issue 里贴出内容本身**）。

## 安全默认值

| 项 | 默认 | 为什么 |
|---|---|---|
| `profile.mode` | `dry-run` | 克隆即可安全试跑，不会动硬件 |
| `profile.allow_arm_motion` | `false` | 第二道锁；两道锁同时为真才会下发运动 |
| `arm.speed` | `10`（硬上限 20） | 首次调试用低速，防撞 |
| `gripper.port` | `/dev/motor_serial` | udev 固定名，权限 0660 + `dialout` 组，**不是 0666** |
| 相机/VNC | 不涉及 | 仓库不含任何远程桌面组件 |

开锁要用 `arm_unlock.py on`（会要求你确认工作区清空、急停在手边），不要手改 YAML——改错缩进是常见事故。

## 已知的边界（不是漏洞，但要知道）

1. **ROS2 话题没有认证**：同一局域网内任何人只要知道话题名，就能给 `/rm_driver/movej_cmd` 发指令。**不要把机器臂控制器暴露到公网**，也不要在公共 WiFi 下跑。
2. **夹爪走串口明文**：MS42DC 是 RS485/USB 转串口，没有加密也没有校验。物理接触 = 完全控制。
3. **标定结果存在 YAML 里**：`handeye.matrix_4x4` 是按某一台机器的安装位置实测的。换机器必须重标，否则坐标会整体偏——这不是安全问题，但会静默出错。
4. **`config/real_params.yaml` 里的 `arm_ip` / `udp_ip`** 是**局域网地址**（`192.168.x.x`），按你的现场填写，不要填公网 IP。

## 部署前建议的机器加固

```bash
# 只允许密钥登录（先确认自己的公钥能登，否则会把自己锁在外面）
sudo sed -i 's/^PasswordAuthentication yes/PasswordAuthentication no/' /etc/ssh/sshd_config
sudo systemctl restart ssh

# 开防火墙，只放 SSH
sudo ufw allow 22/tcp
sudo ufw --force enable

# 防爆破
sudo apt install -y fail2ban && sudo systemctl enable --now fail2ban
```

## 报告问题

发现安全问题时请开 Issue 并只描述**文件路径 + 风险类型**，例如"`recovery/xxx.sh` 里写了明文地址"。
不要贴出密钥、密码或完整的敏感文件内容。
