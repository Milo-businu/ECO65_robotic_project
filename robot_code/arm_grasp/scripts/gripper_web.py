#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""MS42DC 夹爪网页控制 —— 发布 /gripper/cmd、订阅 /gripper/state

和 gripper_node.py 配合使用：
  终端 A: python3 scripts/gripper_node.py --config config/real_params.yaml
  终端 B: python3 scripts/gripper_web.py --config config/real_params.yaml
浏览器打开 http://127.0.0.1:8777

注意：real_params.yaml 的 profile.mode 默认是 dry-run；要动真夹爪先改成 real。
"""
from __future__ import annotations

import argparse
import json
import os
from http import HTTPStatus
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
import sys
import threading
import time

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from common import is_dry_run, load_config  # noqa: E402

import rclpy  # noqa: E402
from rclpy.node import Node  # noqa: E402
from std_msgs.msg import Float64, Float64MultiArray, String  # noqa: E402

PAGE = r"""<!doctype html>
<html lang="zh-CN">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width, initial-scale=1">
<title>MS42DC 夹爪控制</title>
<style>
  :root { color-scheme: light; font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", sans-serif; }
  * { box-sizing: border-box; }
  body { margin: 0; background: #f4f6f8; color: #17202a; }
  header { height: 58px; display: flex; align-items: center; justify-content: space-between; padding: 0 22px; background: #fff; border-bottom: 1px solid #d9dee3; }
  h1 { margin: 0; font-size: 18px; font-weight: 650; }
  .status { display: flex; align-items: center; gap: 8px; font-size: 13px; color: #53616f; }
  .dot { width: 9px; height: 9px; border-radius: 50%; background: #b42318; }
  .dot.online { background: #16803c; }
  main { width: min(1080px, calc(100% - 32px)); margin: 22px auto; display: grid; grid-template-columns: 360px 1fr; gap: 18px; }
  .panel { background: #fff; border: 1px solid #d9dee3; border-radius: 8px; padding: 18px; }
  h2 { font-size: 15px; margin: 0 0 14px; }
  label { display: block; font-size: 13px; color: #465462; margin: 14px 0 6px; }
  .buttons { display: grid; grid-template-columns: 1fr 1fr; gap: 8px; }
  button { min-height: 40px; border: 1px solid #aab4be; border-radius: 6px; background: #fff; font-size: 14px; cursor: pointer; }
  button.primary { background: #1769aa; color: #fff; border-color: #1769aa; }
  button:disabled { opacity: 0.45; cursor: not-allowed; }
  input[type="number"] { width: 100%; height: 40px; border: 1px solid #aab4be; border-radius: 6px; padding: 0 10px; font-size: 15px; }
  .inline { display: grid; grid-template-columns: 1fr 96px; gap: 8px; }
  .confirm { display: flex; gap: 8px; align-items: flex-start; margin: 16px 0 10px; font-size: 13px; color: #344250; }
  .message { min-height: 20px; margin-top: 10px; font-size: 13px; color: #53616f; }
  .message.error { color: #b42318; }
  .state { display: grid; grid-template-columns: repeat(4, 1fr); gap: 10px; margin-top: 14px; }
  .state div { background: #f7f9fb; border: 1px solid #e4e8ec; border-radius: 6px; padding: 10px; }
  .state strong { display: block; font-size: 18px; margin-bottom: 3px; }
  .state span { font-size: 12px; color: #53616f; }
  .warn { margin-top: 12px; padding: 9px 11px; border-radius: 6px; background: #fff4e5; color: #8a5200; font-size: 13px; }
  table { border-collapse: collapse; width: 100%; font-size: 12px; }
  th, td { padding: 8px 6px; text-align: left; border-bottom: 1px solid #e4e8ec; white-space: nowrap; }
  th { color: #53616f; }
  .empty { color: #74818d; padding: 20px 6px; text-align: center; }
  @media (max-width: 760px) { main { grid-template-columns: 1fr; } }
</style>
</head>
<body>
<header>
  <h1>WHEELTEC MS42DC 夹爪</h1>
  <div class="status"><span id="dot" class="dot"></span><span id="connection">连接中...</span></div>
</header>
<main>
  <section class="panel">
    <h2>控制</h2>
    <div class="buttons">
      <button id="home" type="button">Home / 张开到底</button>
      <button id="query" type="button">查询状态</button>
      <button id="open" class="primary" type="button">Open</button>
      <button id="close" class="primary" type="button">Close</button>
    </div>
    <label for="grip">夹持物体宽度（mm）</label>
    <div class="inline">
      <input id="grip" type="number" min="5" max="120" value="40">
      <button id="gripBtn" type="button">Grip</button>
    </div>
    <label for="width">目标开口（mm）</label>
    <div class="inline">
      <input id="width" type="number" min="1" max="120" value="60">
      <button id="widthBtn" type="button">Move</button>
    </div>
    <label class="confirm"><input id="clear" type="checkbox">夹爪运动区域内没有手、线缆或硬物</label>
    <div id="message" class="message"></div>
    <div id="warn" class="warn" style="display:none"></div>
  </section>
  <section class="panel">
    <h2>状态</h2>
    <div class="state">
      <div><strong id="sWidth">-</strong><span>开口 mm</span></div>
      <div><strong id="sAngle">-</strong><span>角度 °</span></div>
      <div><strong id="sReached">-</strong><span>到位</span></div>
      <div><strong id="sStall">-</strong><span>堵转</span></div>
    </div>
    <h2 style="margin-top:22px">命令日志</h2>
    <table><thead><tr><th>时间</th><th>命令</th><th>结果</th></tr></thead>
      <tbody id="logs"><tr><td class="empty" colspan="3">尚无命令</td></tr></tbody></table>
  </section>
</main>
<script>
const $ = id => document.getElementById(id);
let busy = false;
const logs = [];
function addLog(cmd, result) {
  logs.unshift({t: new Date().toLocaleTimeString(), cmd, result});
  if (logs.length > 20) logs.pop();
  $("logs").innerHTML = logs.map(e => `<tr><td>${e.t}</td><td><code>${e.cmd}</code></td><td>${e.result}</td></tr>`).join("");
}
function setBusy(v) {
  busy = v;
  for (const id of ["home","query","open","close","gripBtn","widthBtn"]) $(id).disabled = v;
}
$("clear").onchange = () => {};
async function post(payload, label) {
  if (busy) return;
  setBusy(true);
  $("message").className = "message";
  $("message").textContent = "发送中...";
  try {
    const r = await fetch("/api/cmd", {method:"POST", headers:{"Content-Type":"application/json"}, body:JSON.stringify(payload)});
    const d = await r.json();
    if (!d.ok) throw new Error(d.error || "请求失败");
    $("message").textContent = "命令已发送";
    addLog(label, "OK");
    $("clear").checked = false;
  } catch (e) {
    $("message").className = "message error";
    $("message").textContent = e.message;
    addLog(label, "失败: " + e.message);
  } finally {
    setBusy(false);
  }
}
const motion = (action, extra={}) => {
  if (!$("clear").checked) { $("message").className = "message error"; $("message").textContent = "请先确认运动区域已清空"; return; }
  post({action, area_clear: true, ...extra}, action);
};
$("home").onclick = () => motion("home");
$("open").onclick = () => motion("open");
$("close").onclick = () => motion("close");
$("gripBtn").onclick = () => motion("grip", {object_mm: Number($("grip").value)});
$("widthBtn").onclick = () => motion("width", {width_mm: Number($("width").value)});
$("query").onclick = () => post({action: "query"}, "query");
async function refresh() {
  try {
    const r = await fetch("/api/status", {cache:"no-store"});
    const d = await r.json();
    $("dot").classList.toggle("online", d.connected);
    $("connection").textContent = d.connected ? `已连接 (${d.mode})` : `等待 /gripper/state (${d.mode})`;
    if (d.state) {
      $("sWidth").textContent = d.state.width_mm < 0 ? "未知" : d.state.width_mm.toFixed(1);
      $("sAngle").textContent = d.state.angle_deg.toFixed(1);
      $("sReached").textContent = d.state.reached ? "是" : "否";
      $("sStall").textContent = d.state.stall ? "是" : "否";
    }
    const warnings = [];
    if (d.mode !== "real") warnings.push("当前是 dry-run，命令不会动真夹爪；把 real_params.yaml 的 profile.mode 改成 real 后重启。");
    if (!d.calibrated) warnings.push("开口映射未标定（calibration_done: false），mm 数值只是粗估。");
    $("warn").style.display = warnings.length ? "block" : "none";
    $("warn").textContent = warnings.join(" ");
  } catch (_) {
    $("dot").classList.remove("online");
    $("connection").textContent = "后端未连接";
  }
}
refresh();
setInterval(refresh, 500);
</script>
</body>
</html>
"""


class GripperWebNode(Node):
    def __init__(self, cfg):
        super().__init__('gripper_web')
        g = cfg['gripper']
        self.cfg = cfg
        self.cmd_pub = self.create_publisher(String, g['cmd_topic'], 10)
        self.width_pub = self.create_publisher(Float64, g['width_topic'], 10)
        self.state = None
        self.updated = 0.0
        self.lock = threading.Lock()
        self.create_subscription(Float64MultiArray, g['state_topic'],
                                 self.on_state, 10)
        self.get_logger().info(
            'gripper web: %s -> %s, state %s' %
            (g['cmd_topic'], g['state_topic'], g['state_topic']))

    def on_state(self, msg):
        with self.lock:
            self.state = list(msg.data)
            self.updated = time.monotonic()

    def snapshot(self):
        with self.lock:
            state, updated = self.state, self.updated
        g = self.cfg['gripper']
        connected = bool(state) and (time.monotonic() - updated) < 2.0
        payload = {
            'ok': True,
            'mode': str(self.cfg['profile'].get('mode', 'dry-run')),
            'calibrated': bool(g.get('calibration_done', False)),
            'connected': connected,
            'limits': {
                'min_width_mm': float(g['min_width_mm']),
                'max_width_mm': float(g['max_width_mm']),
                'open_width_mm': float(g['open_width_mm']),
                'interference_mm': float(g['interference_mm']),
            },
            'topics': {
                'cmd': g['cmd_topic'],
                'width': g['width_topic'],
                'state': g['state_topic'],
            },
            'state': None,
        }
        if state and len(state) >= 4:
            payload['state'] = {
                'width_mm': float(state[0]),
                'angle_deg': float(state[1]),
                'reached': bool(state[2]),
                'stall': bool(state[3]),
            }
        return payload

    def command(self, payload):
        action = str(payload.get('action', '')).strip().lower()
        if action in ('home', 'open', 'close', 'grip', 'width'):
            if payload.get('area_clear') is not True:
                raise ValueError('请先确认运动区域已清空')
        if action in ('open', 'close', 'home', 'query', 'stop'):
            msg = String()
            msg.data = action
            self.cmd_pub.publish(msg)
            return action
        if action == 'grip':
            try:
                obj = float(payload['object_mm'])
            except (KeyError, TypeError, ValueError):
                raise ValueError('object_mm 必须是数字')
            if not 5.0 <= obj <= 120.0:
                raise ValueError('object_mm 必须在 5~120 mm')
            msg = String()
            msg.data = 'grip:%.1f' % obj
            self.cmd_pub.publish(msg)
            return msg.data
        if action == 'width':
            try:
                width = float(payload['width_mm'])
            except (KeyError, TypeError, ValueError):
                raise ValueError('width_mm 必须是数字')
            g = self.cfg['gripper']
            if not float(g['min_width_mm']) <= width <= float(g['max_width_mm']):
                raise ValueError('width_mm 必须在 %.1f~%.1f mm' %
                                 (float(g['min_width_mm']), float(g['max_width_mm'])))
            msg = Float64()
            msg.data = width
            self.width_pub.publish(msg)
            return 'width:%.1f' % width
        raise ValueError('未知 action: %r' % action)


def make_handler(node, page):
    class Handler(BaseHTTPRequestHandler):
        def _json(self, status, payload):
            body = json.dumps(payload, ensure_ascii=False).encode('utf-8')
            self.send_response(status)
            self.send_header('Content-Type', 'application/json; charset=utf-8')
            self.send_header('Content-Length', str(len(body)))
            self.send_header('Cache-Control', 'no-store')
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path in ('/', '/index.html'):
                body = page.encode('utf-8')
                self.send_response(HTTPStatus.OK)
                self.send_header('Content-Type', 'text/html; charset=utf-8')
                self.send_header('Content-Length', str(len(body)))
                self.send_header('Cache-Control', 'no-store')
                self.end_headers()
                self.wfile.write(body)
                return
            if self.path == '/api/status':
                self._json(HTTPStatus.OK, node.snapshot())
                return
            self.send_error(HTTPStatus.NOT_FOUND)

        def do_POST(self):
            if self.path != '/api/cmd':
                self.send_error(HTTPStatus.NOT_FOUND)
                return
            try:
                size = int(self.headers.get('Content-Length', '0'))
                if size <= 0 or size > 4096:
                    raise ValueError('invalid request size')
                payload = json.loads(self.rfile.read(size))
                if not isinstance(payload, dict):
                    raise ValueError('request body must be an object')
                sent = node.command(payload)
                self._json(HTTPStatus.OK, {'ok': True, 'sent': sent})
            except (ValueError, TypeError, json.JSONDecodeError) as exc:
                self._json(HTTPStatus.BAD_REQUEST, {'ok': False, 'error': str(exc)})

        def log_message(self, fmt, *args):
            return

    return Handler


def main():
    parser = argparse.ArgumentParser(description='MS42DC gripper web control')
    parser.add_argument('--config', default=None, help='real_params.yaml')
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--listen-port', type=int, default=8777)
    args, _ = parser.parse_known_args()

    cfg = load_config(args.config)
    rclpy.init()
    node = GripperWebNode(cfg)
    threading.Thread(target=rclpy.spin, args=(node,), daemon=True).start()
    server = ThreadingHTTPServer((args.host, args.listen_port),
                                 make_handler(node, PAGE))
    print('http://%s:%d  mode=%s  %s' %
          (args.host, args.listen_port,
           'dry-run' if is_dry_run(cfg) else 'REAL',
           'calibration_done=%s' % cfg['gripper'].get('calibration_done')),
          flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
