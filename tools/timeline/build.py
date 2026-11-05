#!/usr/bin/env python3
"""把 dsh 会话事件流 + see-now 看屏记录，压成一份时间线数据。

第一性原理：所谓「活动」不过是 (时间戳, 类别, 摘要) 三元组的有序序列。
数据源全是现成的：
  1. ~/.dsh/sessions/<workspace>/<session>/session.v4.jsonl.zstd
     —— 每条 user/message、tool/call、tool/result、turn/* 都带 ms 时间戳
  2. ~/.dsh/see-now/chat.jsonl —— 我看屏幕时说的每句话（role/at）
  3. ~/.dsh/attachments/v1/objects/** —— 抓屏帧 / 附件字节（sha 命名，用 mtime 定时）

输出 timeline.json，交给 index.html 画泳道图。
"""

import importlib.util
import json
import os
import subprocess
import sys
import time
from pathlib import Path

HOME = Path.home()
SESSIONS = HOME / ".dsh" / "sessions"
SEENOW_CHAT = HOME / ".dsh" / "see-now" / "chat.jsonl"
OBJECTS = HOME / ".dsh" / "attachments" / "v1" / "objects"
FOREGROUND = HOME / ".dsh" / "user-activity" / "foreground.jsonl"
OUT = Path(__file__).with_name("timeline.json")

# 保留最近多少天（None = 全量）
DAYS = float(os.environ.get("TIMELINE_DAYS", "3"))
CUTOFF = int((time.time() - DAYS * 86400) * 1000) if DAYS else 0


def jload_line(line: str):
    try:
        return json.loads(line)
    except Exception:
        return None


def iter_events(zstd_path: Path):
    """解压 zstd jsonl 并逐行产出事件 dict。"""
    try:
        proc = subprocess.Popen(
            ["zstd", "-dc", str(zstd_path)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
    except FileNotFoundError:
        return
    for raw in proc.stdout:
        o = jload_line(raw.decode("utf-8", "replace").strip())
        if o:
            yield o
    proc.wait()


def text_of(msg) -> str:
    """把 message.content 里的文本块拼起来（跳过 reasoning）。"""
    if not isinstance(msg, dict):
        return ""
    out = []
    for blk in msg.get("content") or []:
        if isinstance(blk, dict) and blk.get("type") == "text":
            out.append(blk.get("text") or "")
    return "\n".join(out).strip()


def clip(s: str, n: int = 300) -> str:
    s = (s or "").replace("\r", "")
    s = s.strip()
    return s if len(s) <= n else s[:n] + " …"


def summarize_args(name: str, args: str) -> str:
    """把工具参数压成一行人类能读的摘要。"""
    try:
        a = json.loads(args) if args else {}
    except Exception:
        return clip(args, 120)
    if not isinstance(a, dict):
        return clip(str(a), 120)
    if name == "bash":
        return clip(a.get("description") or a.get("command", ""), 140)
    if name in ("read", "write", "edit"):
        return clip(a.get("file_path", ""), 140)
    if name in ("grep", "glob"):
        return clip(f'{a.get("pattern","")} @ {a.get("path",".")}', 140)
    if name == "see_now":
        bits = [f'{k}={v}' for k, v in a.items() if k != "say"]
        return clip(", ".join(bits), 140)
    if name == "compress":
        return clip(str(a.get("content", ""))[:120], 140)
    keys = ("command", "file_path", "query", "url", "pattern", "prompt", "description")
    for k in keys:
        if a.get(k):
            return clip(str(a[k]), 140)
    return clip(json.dumps(a, ensure_ascii=False), 140)


def build_session(path: Path):
    evs = []
    meta = {}
    titles = {}
    calls = {}          # callId -> (start_ms, name, summary)
    pending = []
    # 先全读进内存（单条会话通常几 MB 解压后）
    for o in iter_events(path):
        t = o.get("type")
        tm = o.get("time")
        if t == "session":
            meta = {
                "id": o.get("id"),
                "cwd": o.get("cwd"),
                "createdAt": o.get("createdAt"),
            }
        elif t == "session/title":
            titles[o.get("seq")] = (o.get("data") or {}).get("title")
        elif t == "user/message":
            d = o.get("data") or {}
            txt = text_of(d)
            if txt:
                evs.append({"t": tm, "kind": "user", "name": "user", "text": clip(txt, 400)})
        elif t == "assistant/message":
            d = o.get("data") or {}
            txt = text_of(d.get("message") or {})
            if txt:
                evs.append({"t": tm, "kind": "assistant", "name": "reply", "text": clip(txt, 400)})
        elif t == "tool/call":
            d = o.get("data") or {}
            cid = d.get("callId")
            name = d.get("name") or "?"
            summ = summarize_args(name, d.get("arguments") or "")
            calls[cid] = {"t": tm, "name": name, "summary": summ}
            evs.append({"t": tm, "kind": "tool", "name": name, "text": summ, "callId": cid})
        elif t == "tool/result":
            d = o.get("data") or {}
            cid = d.get("toolCallId") or ((d.get("message") or {}).get("source") or {}).get("callId")
            body = text_of(d.get("message") or {})
            rec = calls.get(cid)
            if rec:
                rec["dur"] = (tm or 0) - (rec["t"] or 0)
                rec["size"] = len(body)
                rec["out"] = clip(body, 260)
        elif t == "turn/start":
            evs.append({"t": tm, "kind": "turn", "name": "turn", "text": ""})

    if not evs:
        return None
    start = min(e["t"] for e in evs if e.get("t"))
    end = max(e["t"] for e in evs if e.get("t")) + 1000
    if CUTOFF and end < CUTOFF:
        return None
    title = titles.get(min(titles)) if titles else None
    title = title or (path.parent.name[:12])
    # 把时长/大小回填进 tool 事件
    for e in evs:
        if e["kind"] == "tool" and e.get("callId") in calls:
            c = calls[e["callId"]]
            e["dur"] = c.get("dur", 0)
            e["size"] = c.get("size", 0)
            e["out"] = c.get("out", "")
    return {
        "id": meta.get("id") or path.parent.name,
        "title": title,
        "cwd": meta.get("cwd"),
        "start": start,
        "end": end,
        "events": evs,
    }


def collect_sessions():
    out = []
    if not SESSIONS.exists():
        return out
    for zf in SESSIONS.glob("*/*/session.v4.jsonl.zstd"):
        try:
            s = build_session(zf)
        except Exception as ex:  # 单条坏掉不影响整体
            print(f"skip {zf}: {ex}", file=sys.stderr)
            continue
        if s:
            s["ws"] = zf.parents[1].name
            out.append(s)
    out.sort(key=lambda s: s["start"])
    return out


def collect_seenow():
    evs = []
    if not SEENOW_CHAT.exists():
        return evs
    for line in SEENOW_CHAT.read_text("utf-8", "replace").splitlines():
        o = jload_line(line.strip())
        if not o:
            continue
        evs.append({
            "t": o.get("at"),
            "kind": "say" if o.get("role") == "assistant" else "meta",
            "name": o.get("role"),
            "text": clip(o.get("text", ""), 300),
        })
    return evs


def is_image(p: Path) -> bool:
    try:
        head = p.open("rb").read(12)
    except Exception:
        return False
    return head.startswith(b"\x89PNG") or head[:3] == b"\xff\xd8\xff" or head[:4] == b"RIFF"


def collect_frames():
    """抓屏帧/附件：用 mtime 当时间，sha 当 id，前端走 /objects/<sha> 取原图。"""
    evs = []
    if not OBJECTS.exists():
        return evs
    for p in OBJECTS.glob("*/*"):
        try:
            st = p.stat()
        except Exception:
            continue
        ms = int(st.st_mtime * 1000)
        if CUTOFF and ms < CUTOFF:
            continue
        if not is_image(p):
            continue
        evs.append({
            "t": ms,
            "kind": "frame",
            "name": "frame",
            "text": f"{st.st_size // 1024} KB",
            "obj": f"/objects/{p.parent.name}/{p.name}",
        })
    evs.sort(key=lambda e: e["t"])
    return evs


# ---------- 用户侧活动：前台窗口 / 浏览器 / 打开的文件 ----------
def _load_collector():
    cu = Path(__file__).with_name("collect_user.py")
    spec = importlib.util.spec_from_file_location("collect_user", cu)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def collect_foreground():
    """前台窗口采样 → 区间。相邻两条记录的间隔就是「这个窗口占据了多久」。"""
    if not FOREGROUND.exists():
        return []
    recs = []
    for line in FOREGROUND.read_text("utf-8", "replace").splitlines():
        o = jload_line(line.strip())
        if o and o.get("t"):
            recs.append(o)
    recs.sort(key=lambda r: r["t"])
    now = int(time.time() * 1000)
    segs = []
    for i, r in enumerate(recs):
        nxt = recs[i + 1]["t"] if i + 1 < len(recs) else now
        gap = max(0, nxt - r["t"])
        # 采样中断（机器休眠 / 采样器停了十几小时）时 gap 会很长，直接当下一条的时间用
        # 会把一段前台拉成横贯整张图的巨条 → 单段最多按 60 秒算。
        end = r["t"] + min(gap, 60_000)
        if end - r["t"] < 200:
            end = r["t"] + 200
        if segs and segs[-1]["app"] == r["app"] and segs[-1]["title"] == r["title"] \
           and abs(segs[-1]["t1"] - r["t"]) < 1500:
            segs[-1]["t1"] = end                     # 同一窗口连续出现 → 并成一段
        else:
            segs.append({"t": r["t"], "t1": end, "app": r["app"], "title": r.get("title", "")})
    return segs


def collect_user():
    try:
        mod = _load_collector()
        chrome, kde, files = mod.chrome(), mod.kde(), mod.files()
    except Exception as ex:
        print(f"user-collect failed: {ex}", file=sys.stderr)
        chrome, kde, files = [], [], []

    def host(u):
        try:
            return u.split("//", 1)[1].split("/", 1)[0]
        except Exception:
            return ""

    # 时间轴的范围 = 全部事件的 min/max，所以一条时钟错误的「未来」记录
    # （KDE Activity Manager 里出现过 22:59 / 23:00 的条目）就能把横轴拉长
    # 六小时，把真实事件全挤到左边一小段。这里钳掉未来项，并回报条数。
    now = int(time.time() * 1000)
    skew = now + 60_000
    dropped = 0

    def valid(t):
        nonlocal dropped
        if not t or t > skew:
            dropped += 1
            return False
        return True

    web = [{"t": e["t"], "kind": "web", "name": "web", "text": e["title"],
            "detail": e["detail"], "host": host(e["detail"])}
           for e in chrome if valid(e["t"])]
    opens = []
    for e in kde:
        if not valid(e["t0"]):
            continue
        opens.append({"t": e["t0"], "t1": min(e["t1"], now), "kind": "open",
                      "name": e["app"], "text": e["detail"], "detail": e["detail"]})
    fse = [{"t": e["t"], "kind": "file", "name": "file", "text": e["title"],
            "detail": e["detail"]} for e in files if valid(e["t"])]
    fg = [{"t": s["t"], "t1": s["t1"], "kind": "fg", "name": s["app"],
           "text": s["title"], "detail": s["app"]} for s in collect_foreground()]
    if dropped:
        print(f"用户侧：丢弃 {dropped} 条未来时间戳记录（时钟异常）", file=sys.stderr)
    return {"web": web, "opens": opens, "files": fse, "fg": fg}


def clamp_future(data, now_ms):
    """第四道防线：无论数据来自哪个源，未来时间戳一律剔除。

    时间轴的可视范围由 min/max 决定，所以只要有一条未来记录，整张图就被
    拉长、真实事件被压成一条线。采集层各自过滤后，这里再按最终产物兜一次，
    并重算被裁会话的 start/end。
    """
    n = 0
    for s in data["sessions"]:
        ev = [e for e in s["events"] if not e.get("t") or e["t"] <= now_ms]
        n += len(s["events"]) - len(ev)
        s["events"] = ev
        ts = [e["t"] for e in ev if e.get("t")]
        if ts:
            s["start"], s["end"] = min(ts), max(ts) + 1000
    for key in ("seenow", "frames"):
        ev = [e for e in data[key] if not e.get("t") or e["t"] <= now_ms]
        n += len(data[key]) - len(ev)
        data[key] = ev
    for key in ("web", "opens", "files", "fg"):
        ev = []
        for e in data["user"][key]:
            if e.get("t", 0) > now_ms:
                n += 1
                continue
            if e.get("t1", 0) > now_ms:
                e["t1"] = now_ms
            ev.append(e)
        data["user"][key] = ev
    return n


def load_coarse():
    """推测层：coarse_history.py 产出的 10 分钟桶（无前台采样时段的旁证）。

    只读它写好的精简文件，不在这里重跑采集——避免和独立运行时各算一遍。
    """
    f = OUT.parent / "coarse-buckets.json"
    if not f.exists():
        return []
    try:
        rows = json.loads(f.read_text("utf-8"))
        return [r for r in rows if isinstance(r, dict) and r.get("t")]
    except Exception as ex:
        print(f"coarse load failed: {ex}", file=sys.stderr)
        return []


def load_fg_guess():
    """推测前台：没有采样的时段里，用 Chrome 访问时刻反推「这段时间 Chrome 大概率在前台」。

    由 coarse_history.py 写盘（fg-guess.json），这里只做格式转换并打上 kind="fgx"，
    前端据此画成斜纹——视觉上必须和真实采样区分开。
    """
    f = OUT.parent / "fg-guess.json"
    if not f.exists():
        return []
    try:
        rows = json.loads(f.read_text("utf-8"))
    except Exception as ex:
        print(f"fg-guess load failed: {ex}", file=sys.stderr)
        return []
    out = []
    for r in rows:
        if not r.get("t"):
            continue
        n = r.get("n", 1)
        out.append({"t": r["t"], "t1": r.get("t1") or r["t"] + 2000, "kind": "fgx",
                    "name": "Google-chrome", "n": n,
                    "text": f"推测：Chrome 大概在前台（{n} 次访问）", "detail": "inferred"})
    return out


def main():
    now = int(time.time() * 1000)
    sessions = collect_sessions()
    data = {
        "generatedAt": now,
        "days": DAYS,
        "sessions": sessions,
        "seenow": collect_seenow(),
        "frames": collect_frames(),
        "user": collect_user(),
        "coarse": load_coarse(),
        "fgGuess": load_fg_guess(),
    }
    dropped = clamp_future(data, now)
    if dropped:
        print(f"全局兜底：再剔除 {dropped} 条未来时间戳", file=sys.stderr)

    # 断言：产物里不允许有任何时间戳晚于生成时刻
    ts = [e["t"] for s in data["sessions"] for e in s["events"] if e.get("t")]
    ts += [e["t"] for e in data["seenow"] if e.get("t")]
    ts += [e["t"] for e in data["frames"] if e.get("t")]
    for k in ("web", "opens", "files", "fg"):
        ts += [e["t"] for e in data["user"][k] if e.get("t")]
    assert not ts or max(ts) <= now, "仍有未来时间戳"

    OUT.write_text(json.dumps(data, ensure_ascii=False, separators=(",", ":")), "utf-8")
    n_ev = sum(len(s["events"]) for s in sessions)
    print(f"写出 {OUT}  ({OUT.stat().st_size/1024:.0f} KB)")
    u = data["user"]
    print(f"会话 {len(sessions)} 条，事件 {n_ev} 条，看屏发言 {len(data['seenow'])} 条，图片 {len(data['frames'])} 张")
    print(f"用户侧：前台 {len(u['fg'])} 段 / 浏览 {len(u['web'])} / 打开 {len(u['opens'])} / 文件 {len(u['files'])}")
    span = (f"{time.strftime('%H:%M:%S', time.localtime(min(ts)/1000))} → "
            f"{time.strftime('%H:%M:%S', time.localtime(max(ts)/1000))}") if ts else "-"
    print(f"时间范围 {span}；未来条目 0")


if __name__ == "__main__":
    main()
