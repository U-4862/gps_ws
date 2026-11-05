#!/usr/bin/env python3
"""采集「用户侧活动」：浏览器访问 / KDE 资源使用 / 最近文件。

输出同目录 user.json：
  { generated, days, chrome:[{t,app,title,detail}], kde:[{t0,t1,app,detail}], files:[{t,app,title,detail}] }

数据全部来自本机，只读不写任何系统文件；Chrome/KDE 的 sqlite 都带锁，先复制再读。
"""
import os
import re
import sys
import json
import time
import shutil
import sqlite3
import tempfile
from datetime import datetime, timezone

HOME = os.path.expanduser("~")
HERE = os.path.dirname(os.path.abspath(__file__))
DAYS = float(os.environ.get("TIMELINE_DAYS", "3"))
NOW = time.time()
T0 = NOW - DAYS * 86400


# ---------- 1. Chrome 访问历史 ----------
def chrome():
    src = os.path.join(HOME, ".config/google-chrome/Default/History")
    if not os.path.exists(src):
        return []
    tmp = tempfile.mktemp(suffix=".db")
    shutil.copyfile(src, tmp)          # 正在被浏览器占用，只能复制快照
    out = []
    try:
        db = sqlite3.connect(tmp)
        q = ("select u.url, u.title, v.visit_time from visits v "
             "join urls u on u.id = v.url order by v.visit_time desc limit 40000")
        for url, title, vt in db.execute(q):
            t = vt / 1e6 - 11644473600        # Chrome epoch(1601) 微秒 → unix 秒
            if t < T0:
                break
            out.append({"t": int(t * 1000), "src": "chrome", "app": "chrome",
                        "title": (title or url)[:140], "detail": url[:300]})
    finally:
        os.unlink(tmp)
    return out


# ---------- 2. KDE Activity Manager：谁在什么时候碰了什么资源 ----------
def kde():
    base = os.path.join(HOME, ".local/share/kactivitymanagerd/resources/database")
    if not os.path.exists(base):
        return []
    d = tempfile.mkdtemp()
    dst = os.path.join(d, "db")
    for suf in ("", "-wal", "-shm"):
        if os.path.exists(base + suf):
            shutil.copyfile(base + suf, dst + suf)
    out = []
    try:
        db = sqlite3.connect(dst)
        q = ("select initiatingAgent, targettedResource, start, end "
             "from ResourceEvent order by start desc limit 20000")
        for agent, res, start, end in db.execute(q):
            if start < T0:
                break
            if end is None:          # ResourceEvent.end 允许 NULL → max(None, int) 会抛 TypeError
                end = start
            if res.startswith("applications:"):        # 点开某个应用
                name = res.split(":", 1)[1].replace(".desktop", "")
                out.append({"t0": int(start * 1000), "t1": int(max(end, start + 1) * 1000),
                            "src": "kde", "app": agent, "detail": name, "kind": "launch"})
            else:                                      # 打开某个文件/URL
                out.append({"t0": int(start * 1000), "t1": int(max(end, start + 1) * 1000),
                            "src": "kde", "app": agent, "detail": res[:300], "kind": "open"})
    finally:
        shutil.rmtree(d, ignore_errors=True)
    return out


# ---------- 3. 最近使用文件（XDG recently-used.xbel） ----------
BOOKMARK = re.compile(r'<bookmark href="([^"]+)"([^>]*)>')
def files():
    p = os.path.join(HOME, ".local/share/recently-used.xbel")
    if not os.path.exists(p):
        return []
    txt = open(p, encoding="utf-8", errors="replace").read()
    out = []
    for href, attrs in BOOKMARK.findall(txt):
        m = re.search(r'visited="([^"]+)"', attrs) or re.search(r'modified="([^"]+)"', attrs)
        if not m:
            continue
        try:
            dt = datetime.strptime(m.group(1)[:19], "%Y-%m-%dT%H:%M:%S").replace(tzinfo=timezone.utc)
        except ValueError:
            continue
        t = dt.timestamp()
        if t < T0:
            continue
        path = href[7:] if href.startswith("file://") else href
        path = path.rstrip("/") or path
        if path in ("", "/"):
            continue
        out.append({"t": int(t * 1000), "src": "xbel", "app": "file",
                    "title": os.path.basename(path)[:120], "detail": path[:300]})
    return sorted(out, key=lambda e: e["t"])


def sane(evts, keys):
    """丢掉时间戳落在未来的记录。

    第一条原理：未来时间不存在，出现就只能是时钟跳变或数据源写坏
    （kactivitymanagerd 的 ResourceEvent 里出现过 22:59 / 23:00 的条目）。
    一条这样的记录就能把整条时间轴拉长几小时，所以在采集层就砍掉。
    """
    out, dropped = [], 0
    for e in evts:
        if any(e.get(k, 0) > (NOW + 60) * 1000 for k in keys):
            dropped += 1
            continue
        out.append(e)
    if dropped:
        print(f"丢弃未来时间戳 {dropped} 条（{','.join(keys)}）", file=sys.stderr)
    return out


def main():
    data = {"generated": int(NOW * 1000), "days": DAYS,
            "chrome": sane(chrome(), ("t",)),
            "kde": sane(kde(), ("t0", "t1")),
            "files": sane(files(), ("t",))}
    path = os.path.join(HERE, "user.json")
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False)
    print(f"chrome={len(data['chrome'])} kde={len(data['kde'])} files={len(data['files'])} "
          f"→ {path} ({os.path.getsize(path)/1024:.0f} KB)")


if __name__ == "__main__":
    main()
