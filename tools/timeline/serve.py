#!/usr/bin/env python3
"""时间线本地服务：静态文件 + /objects/ 转发到 dsh 附件仓。

为什么需要它：抓屏帧存在 ~/.dsh/attachments/v1/objects/<前两位>/<sha>，
文件名没有扩展名，浏览器不会按图片猜类型，所以由这里补 Content-Type。
"""

import json
import mimetypes
import os
import subprocess
import sys
from http.server import SimpleHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

ROOT = Path(__file__).parent
OBJECTS = Path.home() / ".dsh" / "attachments" / "v1" / "objects"
PORT = int(os.environ.get("PORT", "3099"))


class H(SimpleHTTPRequestHandler):
    def __init__(self, *a, **kw):
        super().__init__(*a, directory=str(ROOT), **kw)

    # 默认不开缓存：timeline.json / index.html 每次刷新都必须重新取。
    # 之前这里是 SimpleHTTPRequestHandler 的默认行为（只有 Last-Modified），
    # 浏览器会把旧的 timeline.json 留在内存里，看起来像「数据没更新」。
    _allow_cache = False

    def end_headers(self):
        if not self._allow_cache:
            self.send_header("Cache-Control", "no-store, no-cache, must-revalidate")
            self.send_header("Pragma", "no-cache")
        self._allow_cache = False
        super().end_headers()

    def do_GET(self):
        if self.path.startswith("/objects/"):
            rel = self.path[len("/objects/"):].split("?")[0]
            p = (OBJECTS / rel).resolve()
            try:
                p.relative_to(OBJECTS.resolve())
            except ValueError:
                self.send_error(403)
                return
            if not p.is_file():
                self.send_error(404)
                return
            head = p.open("rb").read(12)
            ctype = ("image/png" if head.startswith(b"\x89PNG") else
                     "image/jpeg" if head[:3] == b"\xff\xd8\xff" else
                     "image/webp" if head[:4] == b"RIFF" else
                     "application/octet-stream")
            body = p.read_bytes()
            self.send_response(200)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self._allow_cache = True            # 抓屏帧内容不会变，可以缓存
            self.send_header("Cache-Control", "max-age=600")
            self.end_headers()
            self.wfile.write(body)
            return
        super().do_GET()

    def do_POST(self):
        """POST /rescan —— 重跑 build.py，把新会话/新抓屏帧并进 timeline.json。"""
        if self.path.split("?")[0] != "/rescan":
            self.send_error(404)
            return
        try:
            r = subprocess.run(
                [sys.executable, str(ROOT / "build.py")],
                capture_output=True, text=True, timeout=120, cwd=str(ROOT),
            )
            body = (r.stdout + r.stderr).strip()[:2000]
            ok = r.returncode == 0
        except Exception as ex:
            body, ok = f"rescan failed: {ex}", False
        payload = json.dumps({"ok": ok, "log": body}, ensure_ascii=False).encode()
        self.send_response(200 if ok else 500)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(payload)))
        self.end_headers()
        self.wfile.write(payload)

    def log_message(self, fmt, *args):
        sys.stderr.write("%s - %s\n" % (self.address_string(), fmt % args))


if __name__ == "__main__":
    srv = ThreadingHTTPServer(("127.0.0.1", PORT), H)
    print(f"活动时间线: http://127.0.0.1:{PORT}/  (附件: {OBJECTS})")
    srv.serve_forever()
