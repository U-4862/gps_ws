#!/usr/bin/env python3
"""粗粒度活动补记：用「软件日志 + 会话事件流」把前台采样缺失的时段拼回来。

第一性原理
----------
前台窗口是**瞬时状态**（X11 的 _NET_ACTIVE_WINDOW 只有一个当前值），没人持续读
就没有历史。但它会在别的系统里留下**副作用**，每个副作用都自带时间戳：

  Chrome History   -> 某个时刻发生过一次网页访问（url + visit_time）
  KDE ResourceEvent-> 某个时刻打开过某个文件/应用（targettedResource + start）
  dsh session      -> 某个时刻 agent 跑过一次工具、收到过一条用户消息
  journalctl       -> 会话/登录/关机等系统级边界

这些都不能还原「前台是哪个窗口」，只能还原「那几分钟这台机器在被谁用、用在哪」。
所以本脚本的输出粒度是 10 分钟桶，标注每条记录的来源，绝不假装成前台记录。

用法
  python3 coarse_history.py                       # 默认 昨晚 19:00 → 现在
  python3 coarse_history.py --since 2026-11-03T19:00 --until 2026-11-04T14:40
"""

import argparse
import collections
import datetime as dt
import json
import os
import shutil
import sqlite3
import subprocess
import sys
import tempfile
import urllib.parse
import xml.etree.ElementTree as ET
from pathlib import Path

HOME = Path.home()
HERE = Path(__file__).parent
CHROME_EPOCH = 11644473600          # 1601-01-01 → 1970-01-01 秒差
BUCKET = 600                        # 10 分钟

SESSIONS = HOME / ".dsh/sessions/--home-w-projects-projects-gps_ws--"


# ---------- 各数据源 ----------

def src_chrome(lo, hi):
    hist = HOME / ".config/google-chrome/Default/History"
    if not hist.exists():
        return []
    tmp = Path(tempfile.mkdtemp()) / "H"
    shutil.copyfile(hist, tmp)       # 原库被浏览器锁着，必须快照
    out = []
    try:
        c = sqlite3.connect(tmp)
        c.row_factory = sqlite3.Row
        for r in c.execute(
            """select u.url as url, u.title as title, v.visit_time as vt
               from visits v join urls u on u.id = v.url
               where v.visit_time between ? and ? order by v.visit_time""",
            ((lo + CHROME_EPOCH) * 1e6, (hi + CHROME_EPOCH) * 1e6),
        ):
            t = r["vt"] / 1e6 - CHROME_EPOCH
            host = ""
            try:
                host = urllib.parse.urlparse(r["url"]).hostname or ""
            except Exception:
                pass
            out.append({"t": int(t), "src": "chrome", "what": r["title"] or r["url"], "host": host})
        c.close()
    except Exception as ex:
        print(f"chrome: {ex}", file=sys.stderr)
    finally:
        shutil.rmtree(tmp.parent, ignore_errors=True)
    return out


def src_kde(lo, hi):
    db = HOME / ".local/share/kactivitymanagerd/resources/database"
    if not db.exists():
        return []
    out = []
    try:
        k = sqlite3.connect(f"file:{db}?mode=ro", uri=True)
        k.row_factory = sqlite3.Row
        for r in k.execute(
            """select e.start as st, e.targettedResource as res, i.title as ti
               from ResourceEvent e
               left join ResourceInfo i on i.targettedResource = e.targettedResource
               where e.start between ? and ? order by e.start""",
            (lo, hi),
        ):
            name = (r["ti"] or r["res"] or "").replace("applications:", "").replace(".desktop", "")
            out.append({"t": int(r["st"]), "src": "kde", "what": name, "host": ""})
        k.close()
    except Exception as ex:
        print(f"kde: {ex}", file=sys.stderr)
    return out


def src_session(lo, hi):
    """dsh 会话事件流：tool/call 与 user/message 的时间戳。"""
    if not SESSIONS.exists():
        return []
    out = []
    for f in SESSIONS.glob("*/session.v4.jsonl.zstd"):
        try:
            raw = subprocess.run(["zstd", "-dc", str(f)], capture_output=True, timeout=60).stdout
        except Exception:
            continue
        for line in raw.splitlines():
            try:
                o = json.loads(line)
            except Exception:
                continue
            typ = o.get("type")
            if typ not in ("tool/call", "user/message"):
                continue
            t = (o.get("time") or 0) / 1000
            if not (lo <= t <= hi):
                continue
            name = (o.get("data") or {}).get("name") or ""
            text = ""
            if typ == "user/message":
                d = o.get("data") or {}
                c = d.get("content")
                if isinstance(c, list):
                    text = " ".join(str(x.get("text", "")) for x in c if isinstance(x, dict))
                elif isinstance(c, str):
                    text = c
                text = " ".join(text.split())[:60]
            out.append({"t": int(t), "src": "dsh", "what": text or name, "host": ""})
    return out


def src_system(lo, hi):
    """系统级边界：登录、关机、开机。"""
    try:
        raw = subprocess.run(
            ["journalctl", "--since", dt.datetime.fromtimestamp(lo).strftime("%Y-%m-%d %H:%M:%S"),
             "--until", dt.datetime.fromtimestamp(hi).strftime("%Y-%m-%d %H:%M:%S")],
            capture_output=True, text=True, timeout=60).stdout
    except Exception:
        return []
    keys = ("session opened for user w", "Shutting down", "system boot", "LockedHint",
            "PrepareForSleep", "suspend entry", "suspend exit")
    out = []
    for line in raw.splitlines():
        if not any(k in line for k in keys):
            continue
        head = line[:15]                     # "11月 04 14:27:47"
        try:
            parts = head.replace("月", "").replace("日", "").split()
            t = dt.datetime(2026, int(parts[0]), int(parts[1]),
                            *map(int, parts[2].split(":"))).timestamp()
        except Exception:
            continue
        msg = line.split(": ", 1)[-1][:70]
        out.append({"t": int(t), "src": "system", "what": msg, "host": ""})
    return out


# ---------- 聚合 ----------

def build_fg_guess(lo, hi):
    """没有前台采样的时段：用 Chrome 访问时刻反推「这段时间 Chrome 大概率在前台」。

    只做一件事——把访问时刻连成段：相邻访问间隔 < 90 秒视为同一段（同一窗口在连续翻页），
    单段最长按 60 秒算。真实采样已覆盖的时刻一律跳过。
    这是**推测**，前端用斜纹画，绝不与真实采样混在一起。
    """
    tj = HERE / "timeline.json"
    real = []
    if tj.exists():
        try:
            real = json.loads(tj.read_text("utf-8")).get("user", {}).get("fg", [])
        except Exception:
            pass

    def covered(t):
        for s in real:
            if s["t"] <= t <= (s.get("t1") or s["t"]):
                return True
        return False

    ev = sorted((r["t"] * 1000, r.get("host") or "") for r in src_chrome(lo, hi))  # 统一到毫秒
    segs = []
    for i, (t, host) in enumerate(ev):
        if covered(t):
            continue
        nxt = ev[i + 1][0] if i + 1 < len(ev) else t + 60_000
        e = t + max(2_000, min(60_000, nxt - t))
        if segs and t - segs[-1]["t1"] < 90_000:
            segs[-1]["t1"] = e
            segs[-1]["n"] += 1
            segs[-1]["host"] = host
        else:
            segs.append({"t": t, "t1": e, "host": host, "n": 1})
    return segs


def bucketize(rows, lo):
    buckets = collections.defaultdict(list)
    for r in rows:
        buckets[int((r["t"] - lo) // BUCKET)].append(r)
    return buckets


def verdict(items):
    kinds = {i["src"] for i in items}
    if "dsh" in kinds and "chrome" in kinds:
        return "人在电脑前：一边用 agent 干活一边浏览"
    if "dsh" in kinds:
        return "人在电脑前：与 agent 交互"
    if "chrome" in kinds and "kde" in kinds:
        return "浏览 + 打开文件/应用"
    if "chrome" in kinds:
        return "浏览网页"
    if "kde" in kinds:
        return "打开文件/应用"
    return "无痕迹"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="2026-11-03T19:00")
    ap.add_argument("--until", default=dt.datetime.now().strftime("%Y-%m-%dT%H:%M"))
    a = ap.parse_args()
    lo = dt.datetime.fromisoformat(a.since).timestamp()
    hi = dt.datetime.fromisoformat(a.until).timestamp()

    rows = src_chrome(lo, hi) + src_kde(lo, hi) + src_session(lo, hi)
    guess = build_fg_guess(lo, hi)
    out_fg = HERE / "fg-guess.json"
    out_fg.write_text(json.dumps(guess, ensure_ascii=False), "utf-8")
    covered_min = sum(g["t1"] - g["t"] for g in guess) / 60000
    print(f"写出 {out_fg}  ({out_fg.stat().st_size} B, {len(guess)} 段, 共 {covered_min:.0f} 分钟推测前台)")
    sysrows = src_system(lo, hi)
    buckets = bucketize(rows, lo)

    # ---- md ----
    fmt_t = lambda t: dt.datetime.fromtimestamp(t).strftime("%m-%d %H:%M")
    L = [f"# 粗粒度活动补记 · {a.since} → {a.until}", ""]
    L.append("> 来源：Chrome History / KDE Activity Manager / dsh 会话事件流 / journalctl。")
    L.append("> 粒度 10 分钟。**这不是前台窗口记录**——前台窗口是瞬时状态，没采到就永久丢失；")
    L.append("> 这里只有「那几分钟这台机器被用在哪」的副作用证据。")
    L.append("")

    if sysrows:
        L.append("## 系统边界")
        L.append("")
        L.append("| 时间 | 事件 |")
        L.append("|---|---|")
        for r in sorted(sysrows, key=lambda x: x["t"]):
            L.append(f"| {fmt_t(r['t'])} | {r['what']} |")
        L.append("")

    L.append("## 10 分钟桶")
    L.append("")
    L.append("| 时段 | 网页 | 打开 | agent | 判断 |")
    L.append("|---|---|---|---|---|")
    for b in sorted(buckets):
        items = buckets[b]
        t0 = lo + b * BUCKET
        hosts = collections.Counter(i["host"] for i in items if i["src"] == "chrome" and i["host"])
        web = "、".join(f"{h}×{n}" for h, n in hosts.most_common(3))
        opens = [i["what"] for i in items if i["src"] == "kde"]
        opens_s = "、".join(dict.fromkeys(opens))[:60]
        dsh = sum(1 for i in items if i["src"] == "dsh")
        L.append(f"| {dt.datetime.fromtimestamp(t0).strftime('%m-%d %H:%M')} | {web} | {opens_s} "
                 f"| {dsh or ''} | {verdict(items)} |")
    L.append("")

    # ---- 关机空档也要说 ----
    L.append("## 空档说明")
    L.append("")
    L.append("- 前台采样（`foreground.jsonl`）缺失的时段，本表只能给到「分钟级有过什么」，")
    L.append("  给不到「那一分钟屏幕上是哪个窗口」。")
    L.append("- 关机/重启期间系统不存在，任何来源都不会有记录——那是真空，不是漏采。")
    L.append("")

    out_md = HERE / f"history-coarse-{a.since[:10]}_{a.until[:10]}.md"
    out_md.write_text("\n".join(L), "utf-8")

    # ---- jsonl ----
    out_jl = out_md.with_suffix(".jsonl")
    with out_jl.open("w", encoding="utf-8") as f:
        for b in sorted(buckets):
            t0 = lo + b * BUCKET
            f.write(json.dumps({
                "bucket_start": int(t0),
                "time": dt.datetime.fromtimestamp(t0).strftime("%Y-%m-%d %H:%M"),
                "verdict": verdict(buckets[b]),
                "items": buckets[b],
            }, ensure_ascii=False) + "\n")

    # ---- 精简桶：给时间线当「推测层」用，不带逐条 items ----
    slim = []
    for b in sorted(buckets):
        items = buckets[b]
        t0 = lo + b * BUCKET
        hosts = collections.Counter(i["host"] for i in items if i["src"] == "chrome" and i["host"])
        opens = [i["what"] for i in items if i["src"] == "kde"]
        dsh = sum(1 for i in items if i["src"] == "dsh")
        slim.append({
            "t": int(t0 * 1000),
            "dur": BUCKET * 1000,
            "verdict": verdict(items),
            "web": [f"{h}×{n}" for h, n in hosts.most_common(3)],
            "opens": list(dict.fromkeys(opens))[:4],
            "dsh": dsh,
            "n": len(items),
            "strength": round(min(1.0, len(items) / 30), 2),
        })
    out_slim = HERE / "coarse-buckets.json"
    out_slim.write_text(json.dumps(slim, ensure_ascii=False), "utf-8")
    print(f"写出 {out_slim}  ({out_slim.stat().st_size} B, {len(slim)} 桶)")

    print(f"写出 {out_md}  ({out_md.stat().st_size} B, {len(buckets)} 个桶)")
    print(f"写出 {out_jl}  ({out_jl.stat().st_size} B)")
    print(f"证据条数：chrome {sum(1 for r in rows if r['src']=='chrome')} / "
          f"kde {sum(1 for r in rows if r['src']=='kde')} / "
          f"dsh {sum(1 for r in rows if r['src']=='dsh')} / system {len(sysrows)}")


if __name__ == "__main__":
    main()
