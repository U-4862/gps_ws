#!/usr/bin/env python3
"""时间线流水线：采集 → 构建 → 服务重扫 → 事件记录 → 截图。

一条命令跑完，每一步写进 run.log；某一步失败会中断并标红，
所以「回复中断」不会再让人搞不清数据到底更新了没有。

用法：
  python3 run.py                     # 默认：采集+构建+重扫+近30分钟记录+截图
  python3 run.py --since 16:55       # 指定记录起点
  python3 run.py --no-record         # 不生成事件记录
  python3 run.py --no-shot           # 不截图（无 Chrome 环境时）
  python3 run.py --serve             # 服务没起时自动拉起 serve.py
"""

import argparse
import datetime as dt
import json
import re
import subprocess
import sys
import time
import urllib.error
import urllib.request
from pathlib import Path

HERE = Path(__file__).parent
LOG = HERE / "run.log"
PY = sys.executable
URL = "http://127.0.0.1:3099/"


def log(msg: str) -> None:
    line = f"[{dt.datetime.now():%Y-%m-%d %H:%M:%S}] {msg}"
    print(line, flush=True)
    with LOG.open("a", encoding="utf-8") as f:
        f.write(line + "\n")


def tail(text: str, n: int = 3) -> str:
    lines = [l for l in (text or "").strip().splitlines() if l.strip()]
    return " / ".join(lines[-n:]) if lines else ""


def step(name: str, fn):
    """执行一步，计时并入日志。返回 (ok, 摘要)。"""
    log(f"▶ {name}")
    t0 = time.time()
    try:
        ok, msg = fn()
    except Exception as ex:
        ok, msg = False, f"{type(ex).__name__}: {ex}"
    dur = time.time() - t0
    log(f"{'✔' if ok else '✘'} {name} ({dur:.1f}s) {msg}")
    if not ok:
        raise SystemExit(f"流水线中断于「{name}」：{msg}")
    return msg


def run_py(script: str, *args: str):
    r = subprocess.run([PY, str(HERE / script), *args],
                       capture_output=True, text=True, cwd=str(HERE), timeout=300)
    out = (r.stdout or "") + (r.stderr or "")
    return r.returncode == 0, tail(out)


def collect():
    return run_py("collect_user.py")


def build():
    ok, msg = run_py("build.py")
    if ok:
        data = json.loads((HERE / "timeline.json").read_text("utf-8"))
        ts = [e["t"] for s in data["sessions"] for e in s["events"]]
        ts += [e["t"] for e in data["frames"]] + [e["t"] for e in data["seenow"]]
        future = sum(1 for t in ts if t > time.time() * 1000 + 60_000)
        if future:
            return False, f"仍有 {future} 条未来时间戳"
        msg = f"{len(ts)} 个时间戳 / future=0 / " + msg
    return ok, msg


def rescan(autostart: bool = False):
    def post():
        req = urllib.request.Request(URL + "rescan", method="POST")
        with urllib.request.urlopen(req, timeout=240) as r:
            return json.loads(r.read())

    try:
        d = post()
    except (urllib.error.URLError, ConnectionError):
        if not autostart:
            return False, f"服务未在 {URL} 运行（加 --serve 可自动拉起）"
        log("服务未运行，正在启动 serve.py …")
        subprocess.Popen([PY, str(HERE / "serve.py")],
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, cwd=str(HERE))
        for _ in range(40):
            time.sleep(0.25)
            try:
                d = post()
                break
            except Exception:
                continue
        else:
            return False, "拉起 serve.py 后仍连不上"
    return bool(d.get("ok")), tail(d.get("log", ""), 2)


def record(since: str):
    return run_py("record_events.py", "--since", since)


def shot(out: Path):
    try:
        r = subprocess.run(
            ["google-chrome", "--headless=new", "--disable-gpu", "--no-sandbox",
             "--hide-scrollbars", "--window-size=1680,1050", "--virtual-time-budget=9000",
             f"--screenshot={out}", URL],
            capture_output=True, text=True, timeout=120)
    except (FileNotFoundError, subprocess.TimeoutExpired) as ex:
        return False, f"截图失败：{ex}"
    ok = out.exists() and out.stat().st_size > 5000
    return ok, f"{out.name} {out.stat().st_size//1024} KB" if ok else "截图文件为空"


def parse_since(s: str) -> str:
    """'30m' / '16:55' / '2026-11-03T16:55' → record_events.py 需要的 ISO 串。"""
    if m := re.fullmatch(r"(\d+)m", s):
        t = dt.datetime.now() - dt.timedelta(minutes=int(m.group(1)))
    elif m := re.fullmatch(r"(\d{1,2}):(\d{2})(?::(\d{2}))?", s):
        now = dt.datetime.now()
        t = now.replace(hour=int(m.group(1)), minute=int(m.group(2)),
                        second=int(m.group(3) or 0), microsecond=0)
    else:
        t = dt.datetime.fromisoformat(s)
    return t.strftime("%Y-%m-%dT%H:%M:%S")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--since", default="30m", help="记录起点：30m / 16:55 / ISO")
    ap.add_argument("--no-record", action="store_true")
    ap.add_argument("--no-shot", action="store_true")
    ap.add_argument("--serve", action="store_true", help="服务没起时自动拉起")
    args = ap.parse_args()

    started = time.time()
    log("=" * 60)

    collect()
    build()
    rescan(autostart=args.serve)
    if not args.no_record:
        record(parse_since(args.since))
    if not args.no_shot:
        step("截图", lambda: shot(HERE / "_preview.png"))

    dur = time.time() - started
    log(f"完成，用时 {dur:.1f}s")
    print(f"\n全部完成：{dur:.1f}s｜数据 {HERE/'timeline.json'}｜日志 {LOG}")


if __name__ == "__main__":
    main()
