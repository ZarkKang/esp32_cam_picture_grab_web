from flask import Flask, render_template, jsonify, request
import time
import os
from datetime import datetime
import socket
import subprocess
import re
from typing import List, Dict

app = Flask(__name__)

# ============ 设备IP配置 ============
ESP32_CAM_IP = "192.168.50.50"           # ESP32-CAM 的 IP
ESP32_S3_IP = "192.168.50.60"            # ESP32-S3 的固定 IP

# ESP32-CAM 相关 URL
ESP32_STREAM_URL = f"http://{ESP32_CAM_IP}"
ESP32_CAPTURE_URL = f"http://{ESP32_CAM_IP}/capture"
ESP32_LOCATION_URL = f"http://{ESP32_CAM_IP}/location"

# ESP32-S3 取物指令 URL
ESP32_S3_FETCH_URL = f"http://{ESP32_S3_IP}/fetch"

PHOTO_DIR = "static/photos"
if not os.path.exists(PHOTO_DIR):
    os.makedirs(PHOTO_DIR)

# ============ 获取本机IP ============
def get_host_ip() -> str:
    try:
        s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        s.connect(("8.8.8.8", 80))
        ip = s.getsockname()[0]
    finally:
        s.close()
    return ip

# ============ 扫描局域网设备（可选，仅用于展示） ============
def scan_lan_devices() -> List[Dict]:
    devices = []
    host_ip = get_host_ip()
    ip_prefix = '.'.join(host_ip.split('.')[:3]) + '.'
    try:
        if os.name == 'nt':
            result = subprocess.check_output(['arp', '-a'], encoding='gbk', errors='ignore')
        else:
            result = subprocess.check_output(['arp', '-n'], encoding='utf-8', errors='ignore')

        pattern = re.compile(r'(' + ip_prefix + r'\d{1,3})\s+([0-9a-fA-F:-]{17})')
        matches = pattern.findall(result)

        for ip, mac in matches:
            mac = mac.lower().replace('-', ':')
            is_esp32 = ip == ESP32_CAM_IP
            try:
                name = socket.gethostbyaddr(ip)[0]
            except:
                name = "ESP32-CAM" if is_esp32 else "未知设备"
            devices.append({"ip": ip, "name": name, "mac": mac, "is_esp32": is_esp32})
    except:
        pass
    return devices

# ============ 右上角悬浮IP信息接口 ============
@app.route('/device_ips')
def device_ips():
    try:
        host_ip = get_host_ip()
        return jsonify({
            "status": "ok",
            "host_ip": host_ip,
            "esp32_ip": ESP32_CAM_IP
        })
    except:
        return jsonify({
            "status": "err",
            "host_ip": "获取失败",
            "esp32_ip": ESP32_CAM_IP
        })

# ============ 首页 ============
@app.route('/')
def index():
    return render_template('index.html')

# ============ 拍照 ============
@app.route('/capture')
def take_photo():
    try:
        import requests
        resp = requests.get(ESP32_CAPTURE_URL, timeout=10)
        filename = datetime.now().strftime("%Y%m%d_%H%M%S.jpg")
        file_path = os.path.join(PHOTO_DIR, filename)
        with open(file_path, 'wb') as f:
            f.write(resp.content)
        return jsonify({"status": "ok", "msg": "保存成功", "filename": filename})
    except Exception as e:
        return jsonify({"status": "err", "msg": str(e)})

# ============ 照片列表 ============
@app.route('/photo_list')
def photo_list():
    try:
        files = [f for f in os.listdir(PHOTO_DIR) if f.endswith('.jpg')]
        files.sort(reverse=True)
        return jsonify({"status": "ok", "photos": files})
    except:
        return jsonify({"status": "err", "photos": []})

# ============ 删除照片 ============
@app.route('/delete_photo/<name>', methods=['DELETE'])
def delete_photo(name):
    try:
        path = os.path.join(PHOTO_DIR, name)
        if os.path.exists(path):
            os.remove(path)
        return jsonify({"status": "ok"})
    except:
        return jsonify({"status": "err"})

# ============ 取物 ============
@app.route('/fetch_item', methods=['POST'])
def fetch_item():
    try:
        import requests
        data = request.get_json()
        loc = data.get('location', '1')
        # 向 ESP32-S3 发送 HTTP 取物指令
        resp = requests.get(ESP32_S3_FETCH_URL, params={'slot': loc}, timeout=5)
        if resp.status_code == 200:
            return jsonify({"status": "ok", "msg": "指令已发送"})
        else:
            return jsonify({"status": "err", "msg": f"S3响应错误: {resp.status_code}"})
    except Exception as e:
        return jsonify({"status": "err", "msg": str(e)})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)