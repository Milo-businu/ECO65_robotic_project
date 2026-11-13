#!/bin/bash
# =====================================================================
# update.sh —— 一条命令更新代码，自动处理"本机配置会被覆盖"的冲突
#
# 为什么需要它：
#   net_autoconf.py 会把 arm_ip / udp_ip 写进两个【被 git 跟踪】的配置文件：
#     robot_code/arm_grasp/config/real_params.yaml
#     robot_code/ros2_rm_robot/rm_driver/config/rm_eco65_config.yaml
#   于是每次 git pull 都会报
#     error: Your local changes to the following files would be overwritten by merge
#   而这两个文件的本机差异只有两行 IP —— 记下来、丢掉、拉完再写回去就行。
#
# 用法：
#   bash ~/robot_vla_ws/update.sh
#   bash ~/robot_vla_ws/update.sh --check     # 只看差多少，不动
# =====================================================================
set -u
WS="${WS:-$HOME/robot_vla_ws}"
# 远端名各机器不同：真机叫 gitee，云机叫 gitee_cloud2 → 自动挑
if [ -z "${REMOTE:-}" ]; then
    for r in gitee gitee_cloud2 origin; do
        if git -C "$WS" remote 2>/dev/null | grep -qx "$r"; then REMOTE="$r"; break; fi
    done
fi
REMOTE="${REMOTE:-origin}"
BRANCH="${BRANCH:-main}"
ARM_CFG="$WS/robot_code/ros2_rm_robot/rm_driver/config/rm_eco65_config.yaml"
PARAMS="$WS/robot_code/arm_grasp/config/real_params.yaml"

cd "$WS" || { echo "❌ 找不到 $WS"; exit 1; }

echo "============================================================"
echo " 更新 $WS  ($REMOTE/$BRANCH)"
echo "============================================================"

# ---------- 只检查 ----------
if [ "${1:-}" = "--check" ]; then
    git fetch "$REMOTE" -q
    echo " 本地 HEAD : $(git rev-parse --short HEAD) $(git log -1 --format=%s | cut -c1-40)"
    echo " 远端 HEAD : $(git rev-parse --short "$REMOTE/$BRANCH")"
    n=$(git rev-list --count "HEAD..$REMOTE/$BRANCH")
    echo " 落后      : $n 个提交"
    echo
    echo " 本地改动："
    git status --short | sed 's/^/   /'
    exit 0
fi

# ---------- 记住本机 IP ----------
grab() { grep -m1 -E "^[[:space:]]*$1:" "$2" 2>/dev/null | sed 's/.*"\([^"]*\)".*/\1/'; }
ARM_IP=$(grab arm_ip "$ARM_CFG")
UDP_IP=$(grab udp_ip "$ARM_CFG")
echo " 记录本机 IP: arm_ip=${ARM_IP:-空}  udp_ip=${UDP_IP:-空}"

# ---------- 保护：real_params.yaml 里可能有【标定结果】 ----------
# ★ 2026-11-09 事故：real_params.yaml 同时装着 IP 和 handeye/gripper 标定结果，
#   一句 checkout 把没提交的标定结果也丢了。所以先看 diff：
#   只有 arm_ip/udp_ip 的改动 → 静默还原；还有别的 → 拦住，让人先 commit。
cp -f "$PARAMS" "$HOME/real_params_backup_$(date +%Y%m%d-%H%M%S).yaml" 2>/dev/null
NON_IP=$(git diff --unified=0 -- "$PARAMS" 2>/dev/null \
         | grep -E '^[+-][^+-]' | grep -vE 'arm_ip|udp_ip' | wc -l)
if [ "$NON_IP" -gt 0 ] && [ "${1:-}" != "--force" ]; then
    echo
    echo "⚠️  real_params.yaml 里除了 IP 还有 $NON_IP 行本地改动 —— 很可能是你的标定结果"
    git diff --stat -- "$PARAMS" | sed 's/^/    /'
    echo "    已备份到 ~/real_params_backup_*.yaml"
    echo "    请先提交，再更新（推荐）："
    echo "      git add $PARAMS && git commit -m 'real: 现场标定结果'"
    echo "    确认要丢弃这些改动，就加 --force：bash update.sh --force"
    exit 1
fi

# ---------- 丢掉这两个文件的本地改动 ----------
git checkout -- "$ARM_CFG" "$PARAMS" 2>/dev/null && echo " 已还原两个配置文件（本机 IP 已记录）"

# ---------- 快进 ----------
git fetch "$REMOTE" -q || { echo "❌ fetch 失败：检查网络 / remote"; exit 1; }
if ! git merge --ff-only "$REMOTE/$BRANCH"; then
    echo
    echo "❌ 不是快进（本地有分叉）。先看："
    echo "     git log --graph --oneline -8"
    echo "   若确认本地没有要留的提交，可以：git reset --hard $REMOTE/$BRANCH"
    exit 1
fi

# ---------- 把 IP 写回去 ----------
if [ -n "${ARM_IP:-}" ]; then
    sed -i "s|^\([[:space:]]*arm_ip:\).*|\1 \"$ARM_IP\"|" "$ARM_CFG" "$PARAMS" 2>/dev/null
fi
if [ -n "${UDP_IP:-}" ]; then
    sed -i "s|^\([[:space:]]*udp_ip:\).*|\1 \"$UDP_IP\"|" "$ARM_CFG" "$PARAMS" 2>/dev/null
fi
[ -n "${ARM_IP:-}${UDP_IP:-}" ] && echo " 已把 IP 写回两个配置文件"

echo
echo " 新 HEAD: $(git rev-parse --short HEAD) $(git log -1 --format=%s | cut -c1-48)"
echo " 本地改动："
git status --short | sed 's/^/   /' || true
echo
echo " 下一步：重启链路（配置只在启动时读一次）"
echo "   bash ~/robot_vla_ws/start_link.sh start_task:=false"
