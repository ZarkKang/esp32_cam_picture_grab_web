from flask import Flask, render_template, send_file, jsonify, request
import time
import os
from datetime import datetime
import socket
import subprocess
import re
from typing import List, Dict

app = Flask(__name__)

# ============ 改成你自己ESP32-CAM的地址 ============
ESP32_STREAM_URL = "http://192.168.50.50"
ESP32_CAPTURE_URL = "http://192.168.50.50/capture"
ESP32_LOCATION_URL = "http://192.168.50.50/location"
ESP32_FETCH_URL = "http://192.168.50.50/fetch"
ESP32_TARGET_IP = "192.168.50.50"

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

# ============ 扫描局域网设备 ============
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
            is_esp32 = ip == ESP32_TARGET_IP
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
            "esp32_ip": ESP32_TARGET_IP
        })
    except:
        return jsonify({
            "status": "err",
            "host_ip": "获取失败",
            "esp32_ip": ESP32_TARGET_IP
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
        cmd = data.get('command', 'fetch')
        loc = data.get('location', 'auto')
        res = requests.get(ESP32_FETCH_URL, params={'cmd': cmd, 'loc': loc}, timeout=5)
        return jsonify({"status": "ok", "msg": res.text})
    except:
        return jsonify({"status": "err", "msg": "连接失败"})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=5000, debug=False)