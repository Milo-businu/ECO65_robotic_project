# 32 · 真机先固化 → 云机基于真机版继续改：双机 Git 协作流程

> 日期：2026-11-03　仓库：`git@gitee.com:Milo-businu/cloud2_eco65_robot.git`（分支 `main`）  
> 云机：`你的服务器IP:/home/ubuntu/robot_vla_ws`　真机：`beavertech:~/robot_vla_ws`  
> **本文只放 Mac 本地，不提交进 Git**（避免干扰真机先推的顺序）



---

## 0. 一句话流程

```
真机：盘点 → 提交 → 拉取云机新提交(rebase) → push Gitee
                    ↓
云机：pull Gitee（拿到真机版）→ 在真机版上改动 → push Gitee + GitHub
```

**当前 Gitee 上的最新提交是 `5f74b32`**（AI 在云机推的）。  
这是分叉判断的锚点：真机 `git log` 里**看不到 `5f74b32`**，就说明真机落后，  
直接 `git push` 一定被拒（`non-fast-forward`），必须先 `pull --rebase`。

---

## 1. ⚠️ 三条铁律（先看，能避免丢改动）

1. **不要用 `git push -f` / `--force`**。分叉时强推会把云机刚推的 5 个提交  
   （`f004b13` … `5f74b32`）全部覆盖掉，包括整套真机集成代码。
2. **不要 `git add -A` 一把梭**。先 `git status --short` 看清改了什么，  
   只 add 该进仓库的文件；`build/ install/ log/` 是编译产物，不该进。
3. **提交前先备份机器专属配置**：  
   `cp ~/robot_vla_ws/robot_code/arm_grasp/config/real_params.yaml ~/real_params.beavertech.bak`  
   （里面填了真机 IP、夹爪标定值、手眼矩阵，是这台机器的实测成果，别丢）

---

## 2. 第 0 步：真机只读盘点（**必须先跑，把输出贴回来**）

在**真机**开终端（`Ctrl+Alt+T`），整段复制执行：

```bash
cd ~/robot_vla_ws

echo "===== 1 是不是 Git 仓库 ====="
git rev-parse --is-inside-work-tree 2>&1

echo "===== 2 分支 / 本地提交 / 状态 ====="
git status -sb 2>&1
git log --oneline -6 2>&1

echo "===== 3 远端 ====="
git remote -v 2>&1

echo "===== 4 具体改了哪些文件 ====="
git status --short 2>&1

echo "===== 5 能不能连 Gitee ====="
timeout 10 ssh -T git@gitee.com 2>&1 || true

echo "===== 6 本机公钥 ====="
cat ~/.ssh/id_ed25519.pub 2>/dev/null || echo "没有公钥"
```

**看输出决定走哪条路线：**

| 第 1 步                         | 第 5 步                                  | 走                   |
| ----------------------------- | -------------------------------------- | ------------------- |
| `true`，且第 3 步有 Gitee 远端       | 出现 `You've successfully authenticated` | **路线 A**            |
| `true`，但连不上 Gitee             | 第 6 步有公钥但没登记 / 没公钥                     | 先做「解锁」，再走 A         |
| `fatal: not a git repository` | —                                      | **路线 B**            |
| 网络完全不通、也不想配密钥                 | —                                      | **路线 C**（改动传回云机代提交） |

### 解锁：让真机能推 Gitee（只做一次）

```bash
# 若第 6 步显示没有公钥，先生成（一路回车，不设密码）
ssh-keygen -t ed25519 -C "beavertech" -f ~/.ssh/id_ed25519 -N ""
cat ~/.ssh/id_ed25519.pub
```

把打印出来的那一整行 `ssh-ed25519 AAAA... beavertech` 复制，  
到 Gitee 网页 → 头像 → 设置 → **SSH 公钥** → 添加。  
加完回到真机验证：`ssh -T git@gitee.com` 应回 `Hi Milo-businu!`。

> 备选（不想配 SSH）：用 HTTPS + 私人令牌  
> `git remote set-url gitee https://<你的Gitee私人令牌>@gitee.com/Milo-businu/cloud2_eco65_robot.git`

---

## 3. 路线 A：真机已是 Git 仓库（最常见，照抄即可）

统一给 Gitee 远端起名 `gitee`，避免真机上原本的远端名不一样：

⚠️ **命令里的 `<...>` 是占位符说明，不是命令的一部分，绝对不要照抄尖括号**  
（照抄会报 `bash: syntax error near unexpected token '<'`，真机 2026-11-03 踩过）。  
下面给的是**可以直接整段复制**的具体路径。

```bash
cd ~/robot_vla_ws

# ① ★ Git 第一次提交必做：配置署名（否则报 "Please tell me who you are"）
git config --global user.name "Milo-businu"
git config --global user.email "you@example.com"

# ② 加/改远端名（若 git remote -v 里已有 gitee 就跳过这行）
git remote add gitee git@gitee.com:Milo-businu/cloud2_eco65_robot.git 2>/dev/null || \
git remote set-url gitee git@gitee.com:Milo-businu/cloud2_eco65_robot.git

# ③ 备份机器专属配置
cp robot_code/arm_grasp/config/real_params.yaml ~/real_params.beavertech.bak

# ④ 确保编译产物被忽略（幂等，重复跑不会写重）
for d in build/ install/ log/; do grep -qxF "$d" .gitignore || echo "$d" >> .gitignore; done

# ⑤ 看清改动
git status --short

# ⑥ ★ 只 add 源码与配置：直接抄这一行，不带尖括号，不含 build/ install/ log/
git add .gitignore robot_code/arm_grasp/
git diff --cached --stat          # 确认列表里没有 build/ install/ log/

# ⑦ 提交
git commit -m "real(beavertech): 真机实测配置与改动"

# ⑧ ★ 关键：先把云机推的 5f74b32 合进来（否则 push 必被拒）
git fetch gitee main
git pull --rebase gitee main
#   ⚠️ 若这里报 CONFLICT（冲突）→ 立刻停下，把输出贴给我，别自己选 ours/theirs

# ⑨ 推送
git push gitee main

# ⑩ 确认
git log --oneline -4
git status -sb
```

> **如果之前已经误 add 过 `build/ install/ log/`**（或 commit 失败导致暂存区有残留），  
> 先清一遍再走上面流程，见第 7 节「`cannot pull with rebase`」。

`pull --rebase` 做了什么（大白话）：把真机这条新提交**挪到** `5f74b32` 的后面，  
像排队一样插到队尾，而不是把两条历史拧成一股。这样 Gitee 上是一条直线，最干净。

---

## 4. 路线 B：真机不是 Git 仓库（U 盘拷的文件夹副本）

不要重新 `git clone`（会把你的改动覆盖掉）。用 `reset --soft` 让 HEAD 对齐远端、  
**但保留工作区文件**：

```bash
cd ~/robot_vla_ws

# ① 初始化并接上 Gitee
git init
git remote add gitee git@gitee.com:Milo-businu/cloud2_eco65_robot.git

# ② ★ 排除编译产物（写进本机 .git/info/exclude，不污染仓库）
printf 'build/\ninstall/\nlog/\n' >> .git/info/exclude

# ③ 取远端历史，把 HEAD 移到 5f74b32，工作区文件原封不动
git fetch gitee main
git reset --soft FETCH_HEAD

# ④ 现在 status 显示的就是「你的版本 vs 5f74b32」的差异
git status --short
git diff --stat

# ⑤ 只提交该提交的（直接抄，不带尖括号）
cp robot_code/arm_grasp/config/real_params.yaml ~/real_params.beavertech.bak
git config --global user.name "Milo-businu"
git config --global user.email "you@example.com"
git add .gitignore robot_code/arm_grasp/
git diff --cached --stat          # 确认没有 build/ install/ log/
git commit -m "real(beavertech): 真机实测配置与改动"
git push gitee main

git log --oneline -4
```

> ⚠️ 第 ④ 步如果列出一大堆 `install/` / `build/` 文件，说明第 ② 步没生效，  
> 先别 add，把输出贴给我。

---

## 5. 路线 C：真机推不了 Gitee（改动传回云机代提交）

真机上打包**只含有改动的文件**：

```bash
cd ~/robot_vla_ws
# 是 git 仓库时，导出改动补丁
git diff --binary > ~/real_changes.patch
git status --short                     # 把未跟踪的新文件也单独拷到 U 盘
```

U 盘/网络把 `real_changes.patch` 交给云机，云机执行：

```bash
cd ~/robot_vla_ws
git apply ~/real_changes.patch
git status --short
git add robot_code/arm_grasp/          # 按实际改动路径调整，别带尖括号
git commit -m "real(beavertech): 真机改动（由云机代提交）"
git push gitee_cloud2 main
git push origin main
```

提交信息里注明来源，历史里才看得出来这版改动出自真机。

---

## 6. 第 2 阶段：云机基于真机这一版继续改

**等真机 `git push gitee main` 成功之后**再做。云机上执行（AI 可代做）：

```bash
cd ~/robot_vla_ws

# ① 从 Gitee 拉真机版（注意：云机默认跟踪的是 GitHub 的 origin，
#    所以要显式指定 gitee_cloud2，别只敲 git pull）
git fetch gitee_cloud2
git status -sb                       # 应显示 "behind" 落后
git pull --rebase gitee_cloud2 main
git log --oneline -4                 # 最上面应是真机那条提交

# ② 此时工作区就是「真机版 + 云机原有代码」，在此基础上改动
#    （改代码 / 写文档 / 修 bug …）
git status --short
git add <改动文件>
git commit -m "<这次改了什么>"

# ③ 两个远端都推，保持对齐
git push gitee_cloud2 main
git push origin main

# ④ 确认两边一致
git log --oneline -1 gitee_cloud2/main
git log --oneline -1 origin/main
```

> 云机的 `origin` 是 GitHub `cloud2_robot_eco65`，`gitee_cloud2` 是 Gitee。  
> 真机只推 Gitee，所以云机这一步顺手把 GitHub 也补齐，两边就不会长期分叉。

---

## 7. 常见报错速查

| 报错                                                                              | 原因                                  | 处理                                                                                                        |
| ------------------------------------------------------------------------------- | ----------------------------------- | --------------------------------------------------------------------------------------------------------- |
| `*** Please tell me who you are` / `fatal: unable to auto-detect email address` | 真机 Git 没配提交署名（第一次提交必现）              | `git config --global user.name "Milo-businu"` + `git config --global user.email "you@example.com"`（与云机一致） |
| `bash: syntax error near unexpected token '<'`                                  | 把文档里的占位符 `<文件>` 原样抄进了命令             | `<...>` 只是说明，删掉尖括号；直接抄第 3 节第 ⑥ 步那行                                                                        |
| `error: cannot pull with rebase: Your index contains uncommitted changes`       | 之前 `git add` 过但 `commit` 失败，暂存区有残留  | 见下方「清理暂存区」                                                                                                |
| `! [rejected] main -> main (non-fast-forward)`                                  | 真机落后于 `5f74b32`                     | `git fetch gitee && git pull --rebase gitee main`，**别用 `-f`**                                             |
| `CONFLICT (content): Merge conflict in ...`                                     | 两边改了同一文件（实测是 `arm_grasp/README.md`） | 见下方「rebase 冲突实战解法」                                                                                        |
| `Permission denied (publickey)`                                                 | 真机公钥没加到 Gitee                       | 走第 2 节「解锁」                                                                                                |
| `fatal: not a git repository`                                                   | 真机是文件夹副本                            | 走路线 B                                                                                                     |
| `error: failed to push some refs`                                               | 同上，或被拒绝                             | 先看 `git status -sb` 是 ahead 还是 behind                                                                     |
| 推完 `git status` 显示 `ahead 1`                                                    | 只推了一个远端                             | 另一个远端再 push 一次                                                                                            |

### 清理暂存区（修复「index 有残留」+ 误 add 的 build/install/log）

```bash
cd ~/robot_vla_ws

# ① 清空暂存区，但【保留工作区文件】（不带 --hard，安全）
git reset

# ② 把编译产物永久排除
for d in build/ install/ log/; do grep -qxF "$d" .gitignore || echo "$d" >> .gitignore; done

# ③ 看清现在的状态（此时应该没有 "A  build/..." 之类）
git status --short

# ④ 重新只 add 源码，然后回到第 3 节第 ⑦ 步继续
git add .gitignore robot_code/arm_grasp/
git diff --cached --stat
```

> `git reset` 只动暂存区，**不会删你改过的任何文件**。  
> 千万别用 `git reset --hard`，那个会把工作区改动一起抹掉。

### rebase 冲突实战解法（真机 2026-11-03 实测，冲突文件 `arm_grasp/README.md`）

现象：

```
First, rewinding head to replay your work on top of it...
Applying: real(beavertech): 真机实测配置与改动
Auto-merging robot_code/arm_grasp/README.md
CONFLICT (content): Merge conflict in robot_code/arm_grasp/README.md
Patch failed at 0001 real(beavertech): 真机实测配置与改动
```

这是**正常**的：真机改了 README（新增 42 行），云机在 `f004b13` 里重写过同一份 README，  
rebase 把真机提交挪到云机版上面时，两边改到同一处，Git 不敢替你决定，就暂停等你处理。  
**此时所有改动都还在，没有丢。**

⚠️ 两个容易踩的坑：

- **不要用 `git checkout --ours/--theirs`**：rebase 里这两个词的意思是**反的**  
  （`--ours` = 上游云机版，`--theirs` = 正在重放的真机提交），极易选反。
- **`git rebase --continue` 默认会弹编辑器**让你确认提交信息，小白会卡在里面。  
  用 `GIT_EDITOR=true` 前缀跳过。

零手工编辑、两边内容都不丢的解法（云机版做主体，真机版整份作为附录接在后面）：

```bash
cd ~/robot_vla_ws

# ① 备份真机版 README（内容绝不丢）
git show 73bd84e:robot_code/arm_grasp/README.md > ~/README.beavertech.md

# ② 冲突文件以【云机最新版】为主体重写
git show gitee/main:robot_code/arm_grasp/README.md > robot_code/arm_grasp/README.md

# ③ 真机版整份作为附录接上（暂时有重复，等云机阶段再整理成一份干净的）
printf '\n\n---\n\n## 附：真机补充文档（来自真机提交，待云机合并整理）\n\n' >> robot_code/arm_grasp/README.md
cat ~/README.beavertech.md >> robot_code/arm_grasp/README.md

# ④ 标记冲突已解决
git add robot_code/arm_grasp/README.md
git status --short

# ⑤ 继续 rebase（GIT_EDITOR=true 避免弹编辑器卡住）
GIT_EDITOR=true git rebase --continue

# ⑥ 成功后推送
git log --oneline -4
git push gitee main
git status -sb
```

> 若第 ⑤ 步又报别的冲突 → 停下贴输出，别自己选。  
> 若想整体退回重来：`git rebase --abort`（回到冲突前，改动仍在工作区）。  
> 第 ⑥ 步 `git log` 里应能看到 `real(beavertech): ...` 排在 `5f74b32` 后面。

**为什么允许 README 暂时重复**：这一步的目标是「真机改动先固化」，  
文档整理属于「云机在真机版基础上再改」那一阶段的活儿 —— 等云机 `pull` 到后，  
由云机把两份 README 合成一份干净的，顺序正好对上。

---

## 8. 一句话总结

**顺序不能反：真机先 `pull --rebase` 再 `push`，云机后 `pull` 再改再 `push`。**  
分叉不可怕，`rebase` 就是让两条历史排队；可怕的是 `push -f`，它会把别人刚推的整段历史抹掉。
