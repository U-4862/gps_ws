# 会话轨迹分析 · session-c0279711

> 数据源：`~/.dsh/sessions/--home-w-projects-projects-gps_ws--/session-c0279711-a6e6-4799-b885-0f31bd89a5dd/session.v4.jsonl.zstd`
> 分析方式：`zstd -dc` 解压后逐事件解析，**全程只读**，未改动任何既有记录。
> 生成时间：2026-11-03 19:00

---

## 0. 概况

| 项 | 值 |
|---|---|
| 会话 ID | `session-c0279711-a6e6-4799-b885-0f31bd89a5dd` |
| 标题 | 查看已有的两个会话都在思考 |
| 跨度 | 17:45:15 → 18:57:35（**72.3 分钟**） |
| 原始体积 | 235,664 B（zstd）；解压后 317 事件 |
| 轮次 | 10 轮 turn / 47 steps / 38 次工具调用 |
| 模型 | `packyapi/deepseek-flash` →（17:57:51）`packycode/gpt-6-astra` |

事件类型分布（前 10）：

```
47  step/start      47  assistant/message    47  step/end
38  tool/call       38  tool/result         31  agent/inbox/spliced
17  user/message    10  turn/start          10  turn/end
 4  llm/retry        4  llm/retry-started
```

工具调用分布：`bash 10 · read 9 · glob 8 · grep 6 · job_output 3 · list_agents 1 · job_kill 1`
**`write 0 · edit 0`** —— 这个数字是后面所有问题的根。

---

## 1. 时间线还原

| 时间 | 用户 | 它做了什么 | 结果 |
|---|---|---|---|
| 17:45:48 | 「查看已有的两个会话都在思考什么」 | 无 | ❌ 连续 4 次 retry，failure 均为 `Service is too busy. We advise users to tempo…`；首条 assistant 文本只有一个 `"` |
| 17:46:18 | — | `bash`: `echo PWD; env \| grep -i dsh; ls -la ~/.dsh` | 拿到环境与目录 |
| 17:46:32 | — | `bash`: `ls -la ~/.dsh/sessions` + 逐目录摘要 | 拿到会话列表 |
| 17:57:51 | `hello` ×3 + 模型切换 | 无 | 「刚才查看会话的操作中断了，只找到会话目录，还没确认那两个会话各自在处理什么。」 |
| 17:58:35 | 「列出最近的目录」 | `bash`: `ls -lt --time-style=long-iso` | 输出 6 个会话目录表 |
| 18:18:21 | 「根据现有 json 里的 action 先完成实时记录的 routine」 | `grep` / `glob` 搜普通 `.json` | ❌ **没找到，反问用户要路径** |
| 18:20:33 | 「查找所有最近会话 json 总结 action」 | 改用 `zstd -dc` + node 解析 | ✅ 读 8 份会话，结论「实时记录项目已存在，另一个会话正在补一键运行入口」 |
| 18:21:24 | — | `node <<'JS'` 批量扫会话 | 汇总各会话 action |
| 18:23:21 | 「关于 timeline 记录有 actions 吗」 | `read` / `grep` 源码 | 列出 `faa6d28f` 与 `61722862` 两个会话的 action 清单 |
| 18:24:01 | 「你能跑一遍吗」 | `python3 - <<PY; import run; run.step(...)` | ✅ 五步全过（见 §2） |
| 18:27:40 | 「你能跑一遍持续 30min 的 record 吗」 | 起后台 `bash-87`（18:28→18:58，每分钟导出） | 写 `continuous_20261103_182800.{jsonl,md,log}` |
| 18:50:47 | 「停一下 可能和另一个会话有冲突」 | `job_kill bash-87`，reason 写明「避免与另一个会话的记录任务冲突」 | ✅ SIGTERM；保留 18:28–18:50:24 的 4 会话 / 116 条主干 / 236 条全量 |
| 18:57:28 | 「这个会话的 id」 | 无 | 回答 id |

### 18:24 那次 `run.py` 五步的实际产出

- 采集：644 次浏览 / 99 次打开 / 27 条文件记录
- 构建：423 段前台活动；校验 1421 个时间戳，未来条目 0
- 服务重扫：页面更新至 18:24
- 事件记录：17:54–18:24，覆盖 5 个会话、387 条主干、835 条全量
- 截图：`_preview.png`

---

## 2. 四个真问题

### 2.1 开局 30 秒烧在重试上

`17:45:48` 与 `17:46:18` 各两次 `llm/retry`，`provider` 都是 `deepseek-official`，`policyKey` 触发条件为 `EMPTY_RESPONSE / MALFORMED_RESPONSE / RATE…`。failure 原文：

```
Service is too busy. We advise users to tempo…
```

对应界面上看到的「已重试模型请求 (2/5)」。这 30 秒内没有任何有效动作。

### 2.2 18:18 卡壳：按字面匹配，而不是按数据形态

你说「现有 json 里的 action」，它执行 `grep -r "action" --include=*.json`。但真正的数据是 `session.v4.jsonl.zstd`——**扩展名是 `.zstd`，不是 `.json`**，所以一次命中都没有。直到 18:20 换指令后它才想到 `zstd -dc`。

同类错误在别的会话也发生过（把「没有 Qt 工程根」当成「没有 Qt 界面」，而实际二进制在 `examples/*/build/`）。共同点：**先按字面关键词筛，再谈验证**。

### 2.3 全程 0 次 `write` / `edit`，所有落盘走 heredoc

10 次 `bash` 中有 4 次是 `cat > file <<EOF` 或 `python3 - <<PY` 形态。由此产生的文件：

```
run.py                          18:11
run.log                         18:24
events_20261103_1655-1716.md/jsonl
events_20261103_1732-1802.md/jsonl
events_20261103_1737-1807.md/jsonl
events_20261103_1742-1812.md/jsonl
events_20261103_1754-1824.md/jsonl
continuous_20261103_182800.md/jsonl/log
```

**heredoc 写文件不经过文件版本守卫**，所以两个会话争 `timeline.json` 时，没有任何一方收到 `FS_STALE_VERSION`，而是直接互相覆盖（我 567 KB ↔ 它 726 KB）。这不是态度问题，是通道选错了。

### 2.4 冲突由用户发现，不是它发现

`list_agents` 只在 17:46 调用过一次，之后 64 分钟内没有任何「同一路径是否有人正在写」的检查：`pgrep` 没跑、`lsof` 没跑、文件 inode 没比对。18:50 你提示「可能和另一个会话有冲突」，它才 `job_kill`。

结论：**并发安全完全靠人兜底。**

---

## 3. 产出归属对照

| 文件 | 归属 | 说明 |
|---|---|---|
| `timeline.json` 726 KB @18:24 | 本会话 `run.build()` | 覆盖了另一会话 567 KB 版本 |
| `user.json` 159 KB @18:24 | 同上 | 另一版本为 153 KB |
| `_preview.png` @18:24 | 本会话截图 | 新增 |
| `run.py` / `run.log` | 本会话 / 61722862 | 新增，把采集-构建-重扫-导出-截图串成五步 |
| `events_*.md` / `events_*.jsonl` ×5 组 | 本会话 | 新增，纯导出产物 |
| `continuous_20261103_182800.*` | 本会话 | 新增，30 分钟连续记录（提前 8 分钟被停） |
| `foreground.jsonl` | **不含本会话** | 自 18:55 起由带 `flock` 的采样器独占写入 |

---

## 4. 如果重来一遍

1. **别用 heredoc 写代码文件**，用 `write` / `edit`：两个会话撞车时至少有一方能拿到 `FS_STALE_VERSION`，而不是静默覆盖。
2. **动手前先问「谁在写」**：`pgrep -af python3` + `lsof <目标文件>`，两秒钟。
3. **找数据先看扩展名全貌**：`ls -la` 一次就能看到 `.jsonl.zstd`，不必 grep 猜。
4. **「看别人在思考什么」应读 `assistant/message` 正文**，而不是列目录；17:45 那轮想清楚这点，第一轮就该有结论。
5. **长任务加互斥**：后台导出类任务先抢一把文件锁，抢不到就退出并报告，别靠用户喊停。

---

## 5. 复核用命令

```bash
S=~/.dsh/sessions/--home-w-projects-projects-gps_ws--/session-c0279711-a6e6-4799-b885-0f31bd89a5dd
zstd -dc "$S/session.v4.jsonl.zstd" > /tmp/s_c027.jsonl
wc -l /tmp/s_c027.jsonl                      # 317

# 事件类型分布
python3 -c "
import json,collections
rows=[json.loads(l) for l in open('/tmp/s_c027.jsonl') if l.strip()]
print(collections.Counter(r['type'] for r in rows).most_common(12))"

# 重试与失败原因
grep -o 'Service is too busy[^"]*' /tmp/s_c027.jsonl | sort -u | head
```
