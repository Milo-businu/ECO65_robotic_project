"""http_client_test.py —— 用系统 Python 3.8 调用 Py3.11 的 VLA 服务
只依赖标准库（urllib），证明跨版本边界真的成立。
"""
import json
import sys
import urllib.request

print("客户端 Python 版本:", sys.version.split()[0])

req = json.dumps({
    "prompt": "pick up the green cube",
    "joint_position": [0.0] * 7,
    "gripper_position": [0.0],
}).encode()

r = urllib.request.urlopen(
    urllib.request.Request("http://127.0.0.1:8000/predict",
                           data=req, headers={"Content-Type": "application/json"}),
    timeout=300,
)
resp = json.loads(r.read())
print("服务返回动作形状:", resp["shape"])
print("首个动作(前3维):", resp["actions"][0][:3])
print("=== 跨版本硬边界验证通过：Py3.8 成功调用 Py3.11 的 VLA ===")
