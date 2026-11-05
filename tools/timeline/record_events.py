#!/usr/bin/env python3
"""把 dsh 会话事件流按时间窗导出成人类可读的事件记录（md + jsonl）。

第一性原理：一次「活动」就是 (时间, 会话, 类型, 摘要) 四元组。事件流的唯一权威
来源是 ~/.dsh/sessions/<ws>/<session>/session.v4.jsonl.zstd，每条都自带 ms 级
`time`，所以不需要任何主观回忆，直接解压 + 过滤 + 排序即可。

用法：
  python3 record_events.py                        # 默认从今天 00:00 到现在
  python3 record_events.py --since 16:55          # 今天 16:55 起
  python3 record_events.py --since 2026-11-03T16:55 --until 2026-11-03T17:20
  python3 record_events.py --since 16:55 --out my_rec.md
"""

import argparse
import collections
import datetime as dt
import glob
import importlib.util
import json
import os
import subprocess
from pathlib import Path

HERE = Path(__file__).parent
SESSIONS = Path.home() / ".dsh" / "sessions"

# 推理层协议事件：主表剔除，jsonl 保留
NOISE = {
    "step/start", "step/end", "session-log-deepseek/delivery-accepted",
    "request/header", "request/context", "agent/inbox/spliced",
    "permission/preset", "sandbox/mode", "approval/policy",
    "session/title-llm-request", "assistant/attempt", "llm/retry",
    "llm/retry-started",
}


def load_build_module():
    """复用 build.py 的 text_of / summarize_args / clip，避免两套摘要逻辑。"""
    spec = importlib.util.spec_from_file_location("tl_build", HERE / "build.py")
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


B = load_build_module()


def parse_when(s: str) -> dt.datetime:
    s = s.strip()
    for fmt in ("%Y-%m-%dT%H:%M:%S", "%Y-%m-%d %H:%M:%S",
                "%Y-%m-%dT%H:%M", "%Y-%m-%d %H:%M"):
        try:
            return dt.datetime.strptime(s, fmt)
        except ValueError:
            pass
    for fmt in ("%H:%M:%S", "%H:%M"):
        try:
            t = dt.datetime.strptime(s, fmt)
            return dt.datetime.combine(dt.date.today(), t.time())
        except ValueError:
            pass
    raise SystemExit(f"无法解析时间：{s}")


def text_of(msg) -> str:
    return B.text_of(msg or {})


def reason_of(msg) -> str:
    out = []
    for blk in ((msg or {}).get("content") or []):
        if isinstance(blk, dict) and blk.get("type") == "reasoning":
            out.append(blk.get("text") or "")
    return "".join(out).strip()


def iter_jsonl(path: Path):
    try:
        raw = subprocess.run(["zstd", "-dc", str(path)], capture_output=True).stdout
    except FileNotFoundError:
        raise SystemExit("缺少 zstd 命令（apt install zstd）")
    for i, line in enumerate(raw.decode("utf-8", "replace").splitlines()):
        line = line.strip()
        if not line:
            continue
        try:
            yield i, json.loads(line)
        except Exception:
            continue


def session_title(path: Path) -> str:
    for _, o in iter_jsonl(path):
        if o.get("type") == "session/title":
            t = (o.get("data") or {}).get("title")
            if t:
                return t
    return path.parent.name[:12]


def describe(o: dict) -> dict:
    """一条原始事件 → 记录条目。"""
    ty = o.get("type")
    d = o.get("data") or {}
    rec = {"type": ty, "name": "", "text": "", "detail": "", "size": 0}
    if ty == "tool/call":
        name = d.get("name") or "?"
        rec["name"] = name
        rec["text"] = B.summarize_args(name, d.get("arguments") or "")
        rec["detail"] = (d.get("arguments") or "")[:4000]
    elif ty == "tool/result":
        body = text_of(d.get("message") or {})
        rec["text"] = body.splitlines()[0][:120] if body else ""
        rec["size"] = len(body)
        rec["detail"] = body[:4000]
    elif ty == "assistant/message":
        msg = d.get("message") or {}
        txt, rs = text_of(msg), reason_of(msg)
        rec["text"] = txt or ("思考：" + rs[:160] if rs else "")
        rec["detail"] = txt or rs
    elif ty == "user/message":
        txt = text_of(d)
        rec["text"] = txt[:200]
        rec["detail"] = txt[:4000]
    elif ty == "session/title":
        rec["text"] = d.get("title") or ""
    elif ty == "workspace/changes":
        ch = d.get("changes") if isinstance(d.get("changes"), list) else []
        rec["text"] = f"{len(ch)} 个文件变更" if ch else ""
        rec["detail"] = json.dumps(ch, ensure_ascii=False)[:4000] if ch else ""
    elif ty == "agent/inbox/spliced":
        ins = d.get("inserted") or []
        rec["text"] = f"插入 {len(ins)} 条消息" if ins else f"移除 {d.get('removedCount', 0)} 条"
        if d.get("outcome"):
            rec["text"] += f" · {d['outcome']}"
    return rec


def collect(t_from: int, t_until: int):
    recs, labels = [], {}
    for path in SESSIONS.glob("*/*/session.v4.jsonl.zstd"):
        rows = list(iter_jsonl(path))
        times = [o["time"] for _, o in rows if o.get("time")]
        if not times or max(times) < t_from or min(times) > t_until:
            continue
        sid = path.parent.name
        labels[sid] = session_title(path)
        for seq, o in rows:
            tm = o.get("time")
            if not tm or not (t_from <= tm <= t_until):
                continue
            rec = describe(o)
            rec.update(t=tm, seq=seq, sid=sid)
            recs.append(rec)
    recs.sort(key=lambda r: (r["t"], r["sid"], r["seq"]))
    for r in recs:
        r["time"] = dt.datetime.fromtimestamp(r["t"] / 1000).strftime("%H:%M:%S")
    return recs, labels


def render(recs, labels, t_from, t_until, src_note):
    main = [r for r in recs if r["type"] not in NOISE]
    span = (f"{dt.datetime.fromtimestamp(t_from/1000):%Y-%m-%d %H:%M:%S} → "
            f"{dt.datetime.fromtimestamp(t_until/1000):%Y-%m-%d %H:%M:%S}")
    out = [
        f"# 事件记录 · {span}\n",
        f"生成时间：{dt.datetime.now():%Y-%m-%d %H:%M:%S}  ",
        f"数据源：{src_note}  ",
        f"覆盖：{len(labels)} 个会话 — " + " / ".join(
            f"`{sid.replace('session-', '')[:8]}`（{title}）" for sid, title in labels.items()) + "  ",
        f"条目：主干 {len(main)} 条 / 全量 {len(recs)} 条"
        f"（含 step、llm/retry、inbox 等协议事件，见同名 .jsonl）\n",
        "## 一、事件流（去协议噪声）\n",
        "| 时间 | 会话 | 类型 | 摘要 |",
        "|---|---|---|---|",
    ]
    for r in main:
        ty = f'工具 · {r["name"]}' if r["type"] == "tool/call" else (
            "工具结果" if r["type"] == "tool/result" else r["type"])
        txt = (r["text"] or "(空)").replace("|", "\\|").replace("\n", " ")
        if len(txt) > 150:
            txt = txt[:150] + " …"
        out.append(f'| {r["time"]} | {labels.get(r["sid"], r["sid"][:8])} | `{ty}` | {txt} |')

    out += ["\n## 二、事件类型统计（全量）\n", "| 类型 | 条数 |", "|---|---|"]
    for k, v in collections.Counter(r["type"] for r in recs).most_common():
        out.append(f"| `{k}` | {v} |")

    out += [
        "\n## 三、说明\n",
        "- 记录的是**事件流**（谁在什么时刻做了什么），不是对话正文；主表摘要截断到 150 字，",
        "  完整内容在同名 `.jsonl` 的 `detail` 字段（截断到 4000 字）。",
        "- `tool/call` 摘要复用 [build.py](build.py) 的 `summarize_args()`；`tool/result` 记首行 + 字节数。",
        "- `llm/retry`、`step/start|end`、`delivery-accepted` 等属推理层协议事件，主表已剔除，jsonl 保留。",
        "- 复现：`python3 record_events.py --since 16:55`（时间窗可任意指定）。",
    ]
    return "\n".join(out) + "\n"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="00:00", help="起始时间：HH:MM[:SS] 或 YYYY-MM-DDTHH:MM")
    ap.add_argument("--until", default=None, help="结束时间，默认现在")
    ap.add_argument("--out", default=None, help="输出 .md 路径（同名 .jsonl 自动生成）")
    args = ap.parse_args()

    t_from = int(parse_when(args.since).timestamp() * 1000)
    t_until = int((parse_when(args.until) if args.until else dt.datetime.now()).timestamp() * 1000)
    if t_until <= t_from:
        raise SystemExit("--until 必须晚于 --since")

    recs, labels = collect(t_from, t_until)
    stem = args.out or str(HERE / (
        f"events_{dt.datetime.fromtimestamp(t_from/1000):%Y%m%d_%H%M}"
        f"-{dt.datetime.fromtimestamp(t_until/1000):%H%M}.md"))
    stem = stem[:-3] if stem.endswith(".md") else stem

    md = render(recs, labels, t_from, t_until,
                "`~/.dsh/sessions/*/*/session.v4.jsonl.zstd`（`zstd -dc` 解压，按 `time` 过滤）")
    Path(stem + ".md").write_text(md, "utf-8")
    Path(stem + ".jsonl").write_text(
        "\n".join(json.dumps(r, ensure_ascii=False) for r in recs) + "\n", "utf-8")

    n_main = sum(1 for r in recs if r["type"] not in NOISE)
    print(f"{stem}.md  主干 {n_main} 条 / 全量 {len(recs)} 条 / 会话 {len(labels)} 个")


if __name__ == "__main__":
    main()
