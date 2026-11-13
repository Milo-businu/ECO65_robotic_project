#!/bin/bash
# =====================================================================
# aliases.sh —— 真机 / 云机常用命令短别名（2026-11-10 · v2）
#
# 【安装】把下面这一行加到 ~/.bashrc 末尾，然后 source ~/.bashrc：
#     [ -f ~/robot_vla_ws/aliases.sh ] && source ~/robot_vla_ws/aliases.sh
#
# 【v2 修的是什么】
#   v1 在真机上 source 时报 `line 96: syntax error near unexpected token '('`。
#   原因：真机 shell 里本来就有 `alias js=...`，而函数定义 `js() { ... }` 在
#   交互式 shell 里会先做 alias 展开 → 变成 `xxxx() {...}` → 语法错。
#   `bash -n` 在非交互 shell 里不做 alias 展开，所以查不出来（这是教训）。
#   修法两条：① 定义前先把同名 alias 清掉；② 最容易撞名的几个改掉（js→jos 等）。
#
# 【设计原则】两条，都是踩过坑换来的：
#   1. 本文件【绝不】在当前 shell 里 source install/setup.bash。
#      项目铁律：污染 .bashrc 会让其它工作空间（ros_tutorial_ws 等）串味。
#      所以所有需要 ROS 环境的命令都放在 ( 子 shell ) 里 source，作用域只有那一条命令。
#   2. 别名只负责"缩短路径"，不改语义；要带参数的用函数，参数原样透传。
#
# 【常用】直接敲  rhelp  看全部别名。
# =====================================================================

export RWS="${RWS:-$HOME/robot_vla_ws}"
export RPKG="$RWS/robot_code/arm_grasp"
export RSCRIPTS="$RPKG/scripts"
export RCFG="$RPKG/config/real_params.yaml"

# ---------- 0. 先清掉可能同名的既有 alias（否则下面的函数定义会语法错） ----------
for _n in r rr rhelp renv ws cfg pf hw rnet pv wmap hq unlock lock jog grasp graspd \
          tpp cg veh sweepeh hecal tune pmode ask brainck link link0 tgt jos camhz rtl \
          rctl kcam krsp knodes gst upd save gpush gpull glog gtail; do
    unalias "$_n" 2>/dev/null
done
unset _n

# ---------- 内部：载入 ROS 环境（只应出现在子 shell 里） ----------
_rsrc() {
    source /opt/ros/foxy/setup.bash
    [ -f "$RWS/install/setup.bash" ] && source "$RWS/install/setup.bash"
    return 0
}

# ---------- 内部：在正确环境里跑 $RSCRIPTS 下的脚本 ----------
r() {
    local f="$RSCRIPTS/$1"
    if [ -z "$1" ]; then echo "用法: r <脚本名> [参数...]  例: r preflight.py"; return 1; fi
    if [ ! -f "$f" ]; then
        echo "❌ 没有这个脚本：$f"
        echo "   现有：$(ls "$RSCRIPTS" 2>/dev/null | tr '\n' ' ')"
        return 1
    fi
    ( _rsrc && cd "$RWS" && python3 "$f" "${@:2}" )
}

# ---------- 内部：跑 recovery/ 下的脚本 ----------
rr() {
    local f="$RWS/recovery/$1"
    if [ ! -f "$f" ]; then echo "❌ 没有这个脚本：$f"; return 1; fi
    ( _rsrc && cd "$RWS" && python3 "$f" "${@:2}" )
}

# =====================================================================
# 一、环境与导航
# =====================================================================
alias ws='cd "$RWS"'
alias cfg='${EDITOR:-nano} "$RCFG"'
renv() { _rsrc && echo "✅ ROS 环境已载入【仅当前 shell】—— 这是唯一会污染当前终端的命令"; }

# =====================================================================
# 二、体检（不动机器人，随时可跑）
# =====================================================================
alias pf='r preflight.py'
alias hw='r check_real_hw.py'
alias rnet='r net_autoconf.py'
alias pv='r pose_view.py'
alias wmap='r workspace_map.py'
alias hq='r handeye_quality.py'

# =====================================================================
# 三、动臂相关（都要两道锁 + 急停在手边）
# =====================================================================
alias unlock='r arm_unlock.py'
alias lock='r arm_unlock.py off'
alias jog='r jog.py'
alias grasp='r grasp_task.py --real'
alias graspd='r grasp_task.py --dry-run'
alias tpp='r teach_place_point.py'

# =====================================================================
# 四、标定
# =====================================================================
alias cg='r calibrate_gripper.py'
alias veh='r verify_handeye.py'
alias sweepeh='rr collect_handeye.py --sweep'
alias hecal='rr handeye_calib.py'

# =====================================================================
# 五、感知
# =====================================================================
alias tune='r tune_perception.py'
alias pmode='r perception_mode.py'
alias ask='r ask_target.py'
alias brainck='r brain.py --check'

# =====================================================================
# 六、启动与话题（都带 ROS 环境）
# =====================================================================
alias link='bash "$RWS/start_link.sh"'
alias link0='bash "$RWS/start_link.sh" start_task:=false'
tgt()   { ( _rsrc && timeout 5 ros2 topic echo /grasp/target_xyz ); }
jos()   { ( _rsrc && timeout 3 ros2 topic echo /joint_states ); }
camhz() { ( _rsrc && timeout 5 ros2 topic hz /camera/d435i/color/image_raw ); }
rtl()   { ( _rsrc && ros2 topic list | grep -E "d435i|grasp|gripper|rm_driver" ); }
rctl()  { ( _rsrc && ros2 control list_controllers ); }

# =====================================================================
# 七、清僵尸（三个历史高频，见 38 号问题册）
# =====================================================================
alias kcam='pkill -9 -f "[r]ealsense2_camera_node"; echo "已清相机占用（RS2_USB_STATUS_BUSY 的解法）"'
alias krsp='pkill -9 -f "[/]robot_state_publisher/robot_state_publisher"; echo "已清 RSP（comm 截断 15 字符，别用 pkill -x）"'
alias knodes='pkill -9 -f "[g]rasp_task"; pkill -9 -f "[p]erception_node"; pkill -9 -f "[g]ripper_node"; echo "已清抓取三节点（运行中别敲）"'

# =====================================================================
# 八、Git 双机流程
# =====================================================================
alias gst='git -C "$RWS" status -sb && git -C "$RWS" log --oneline -3'
alias upd='bash "$RWS/update.sh"'
alias save='bash "$RWS/save_config.sh"'
alias gpush='git -C "$RWS" push gitee main'
alias gpull='git -C "$RWS" pull --rebase gitee main'

# =====================================================================
# 九、日志
# =====================================================================
alias glog='ls -lt ~/grasp_*.log 2>/dev/null | head -5'
gtail() {
    local f
    f=$(ls -t ~/grasp_*.log 2>/dev/null | head -1)
    [ -z "$f" ] && { echo "还没有 grasp_*.log"; return 1; }
    echo "== $f =="; tail -n "${1:-40}" "$f"
}

# =====================================================================
# 十、帮助
# =====================================================================
rhelp() {
cat <<'EOF'
┌─ 环境 ──────────────────────────────────────────────
│ ws   进项目目录        renv  载入 ROS 环境（会污染当前 shell）    cfg  编辑配置
├─ 体检（不动机器人）─────────────────────────────────
│ pf preflight 全验   hw 硬件体检   rnet 网络/udp_ip 自配
│ pv 侧视图看姿态     wmap 可达区   hq 手眼质量
├─ 动臂（要两道锁 + 急停在手）───────────────────────
│ unlock 看锁/开锁    lock 上锁     jog 键盘示教
│ grasp 真机抓取      graspd 演练   tpp 示教放置点
├─ 标定 ──────────────────────────────────────────────
│ cg 夹爪开口标定     veh 手眼打点验证
│ sweepeh 自动扫位     hecal 手眼求解
├─ 感知 ──────────────────────────────────────────────
│ tune 调颜色阈值     pmode 切 color/brain    brainck 大脑自检
│ ask  一句话换目标色   例：ask "抓蓝色的方块"
├─ 启动与话题 ────────────────────────────────────────
│ link 起整机         link0 只起感知/夹爪
│ tgt 目标位姿   jos 关节角   camhz 相机帧率   rtl 相关话题   rctl 控制器
├─ 清僵尸 ────────────────────────────────────────────
│ kcam 相机占用    krsp robot_state_publisher    knodes 抓取三节点
├─ Git ───────────────────────────────────────────────
│ gst 状态+日志    upd 拉取    save 提交配置    gpush 推送    gpull 变基拉取
└─ 日志 ──────────────────────────────────────────────
  glog 列日志    gtail [行数] 看最新日志尾部（默认 40 行）

  通用执行器：r <脚本名> [参数]     例：r grasp_task.py --real --slow
              rr <recovery 脚本>    例：rr collect_handeye.py --help
EOF
}
