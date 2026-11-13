#!/bin/bash
# =====================================================================
# save_config.sh —— 把现场改过的配置立刻提交一版
#
# 为什么需要它（2026-11-09 两次事故）：
#   real_params.yaml 里同时装着【机器无关的模板】和【现场标定结果】
#   （手眼矩阵、夹爪四行、颜色阈值、home/放置点、IP）。
#   只要有人跑 `git checkout -- real_params.yaml`（比如 update.sh），
#   还没提交的现场数据就全没了。今天丢了两次：手眼矩阵、颜色阈值。
#
# 用法：
#   bash save_config.sh                 # 提交（消息自动带时间）
#   bash save_config.sh "颜色阈值"      # 自定义说明
#   bash save_config.sh "放置点" --push # 顺便推到 gitee
# =====================================================================
set -u
WS="${WS:-$HOME/robot_vla_ws}"
PARAMS="$WS/robot_code/arm_grasp/config/real_params.yaml"
MSG="${1:-现场配置}"
PUSH="${2:-}"

cd "$WS" || { echo "❌ 找不到 $WS"; exit 1; }
git add "$PARAMS" || exit 1

if git diff --cached --quiet; then
    echo " real_params.yaml 没有变化，不用提交"
    exit 0
fi

git -c user.name="$(git config user.name || echo beavertech)" \
    commit -q -m "real: $MSG ($(date '+%m-%d %H:%M'))"
echo "✅ 已提交：$(git log --oneline -1)"

if [ "$PUSH" = "--push" ]; then
    R=gitee; git remote | grep -qx gitee || R=$(git remote | head -1)
    git push "$R" main && echo "✅ 已推到 $R"
else
    echo "（只本地提交；要推远端：git push gitee main）"
fi
