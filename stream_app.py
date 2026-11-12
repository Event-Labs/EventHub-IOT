#!/usr/bin/env python3
"""
EventHub QR Check-in System: Multi-Threaded USB High-Speed Video Streamer & Backend Proxy
Connects to ESP32-S3 over CH343 USB-UART @ 2,000,000 baud, decodes JPEG frames,
provides an MJPEG HTTP stream, and proxies Staff API calls to EventHub Backend.
"""

import sys
import os
import time
import json
import base64
import struct
import threading
import argparse
import subprocess
import re
import socket
from http.server import ThreadingHTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse
import serial
import requests

# Enable line-buffered standard output
if hasattr(sys.stdout, "reconfigure"):
    sys.stdout.reconfigure(line_buffering=True)

# Default Settings
DEFAULT_PORT = "COM6"
DEFAULT_BAUD = 2000000
HTTP_PORT = 5000
SYNC_MAGIC = b"\xaa\x55\xaa\x55"
END_MAGIC = b"\x55\xaa\x55\xaa"

# Shared Global State
latest_frame = None
latest_frame_lock = threading.Lock()
frame_counter = 0
current_fps = 0.0
is_serial_connected = False
active_port = DEFAULT_PORT
active_baud = DEFAULT_BAUD
serial_connection = None
serial_lock = threading.Lock()
api_http_session = requests.Session()


def send_session_request(method, url, payload, headers, timeout=12):
    """Send an HTTP API request through the persistent Keep-Alive session."""
    if method == "GET":
        return api_http_session.request(method, url, headers=headers, timeout=timeout)
    return api_http_session.request(method, url, json=payload, headers=headers, timeout=timeout)

# Shared Network Diagnostics State
network_state = {
    "wifi_connected": False,
    "ssid": "Chưa kết nối Wi-Fi",
    "signal": 0,
    "signal_level": 0,
    "signal_text": "Chưa kết nối Wi-Fi",
    "description": "",
    "connection_type": "none",
    "latency_ms": -1,
    "latency_quality": "offline",
    "server_reachable": False,
    "backend_host": "api.eventhub.id.vn",
    "last_updated": 0,
}
network_lock = threading.Lock()

# ESP32-S3 Hardware State (Telemetry over UART / Wi-Fi)
ESP32_CACHE_FILE = os.path.join(os.path.dirname(os.path.abspath(__file__)), ".esp32_last_ip.json")


def _load_cached_esp_ip():
    try:
        with open(ESP32_CACHE_FILE, "r", encoding="utf-8") as f:
            ip = str(json.load(f).get("ip", "")).strip()
            if ip and ip != "0.0.0.0":
                return ip
    except Exception:
        pass
    return "0.0.0.0"


def _save_cached_esp_ip(ip):
    if not ip or ip == "0.0.0.0":
        return
    try:
        with open(ESP32_CACHE_FILE, "w", encoding="utf-8") as f:
            json.dump({"ip": ip}, f)
    except Exception:
        pass


_cached_esp_ip = _load_cached_esp_ip()

esp32_device_state = {
    "wifi_connected": False,
    "ip": _cached_esp_ip,
    "rssi": 0,
    "ssid": "",
    "signal_pct": 0,
    "signal_level": 0,
    "signal_text": "Chưa kết nối",
    "signal_bars": "░░░░",
    "latency_ms": -1,
    "latency_text": "--",
    "latency_quality": "offline",
    "status_text": "Chưa kết nối Wi-Fi",
    "auto_confirm": None,
    "has_token": False,
    "wifi_profiles_count": 0,
    "last_scan": None,
    "last_seen": 0,
}
esp32_state_lock = threading.Lock()

# Request routing mode: "pc" (PC sends requests) or "esp32" (ESP32 Wi-Fi sends requests directly)
request_route_mode = "pc"
request_route_lock = threading.Lock()

# Wi-Fi Connection Handshake State & Event
wifi_connect_event = threading.Event()
wifi_connect_result = {
    "status": "idle",
    "ssid": "",
    "ip": "0.0.0.0",
    "rssi": 0,
    "reason_code": 0,
    "reason_str": "",
    "message": "",
}
wifi_connect_lock = threading.Lock()
pending_target_ssid = ""

# Wi-Fi Scan Caching & ESP32 Hardware Scan State
wifi_scan_cache = {
    "timestamp": 0.0,
    "networks": [],
}
wifi_scan_lock = threading.Lock()
esp_scanned_networks = []
esp_wifi_scan_lock = threading.Lock()
esp_scan_event = threading.Event()
esp_auto_event = threading.Event()
esp_token_event = threading.Event()
esp_wifi_profile_event = threading.Event()

# ESP32 Direct UART HTTPS Request/Response State & Events
esp_verify_event = threading.Event()
esp_verify_result = {"status_code": 500, "content": b""}
esp_verify_lock = threading.Lock()

esp_checkin_event = threading.Event()
esp_checkin_result = {"status_code": 500, "content": b""}
esp_checkin_lock = threading.Lock()


def send_esp32_uart_or_http(uart_cmd, ack_event, http_path, http_payload, wait_timeout=0.8):
    """Send a control command over USB UART first, falling back to ESP32 HTTP when unplugged."""
    if ack_event is not None:
        ack_event.clear()

    sent_via = None
    with serial_lock:
        if serial_connection and serial_connection.is_open:
            try:
                serial_connection.write(f"\n{uart_cmd}\n".encode("utf-8"))
                serial_connection.flush()
                sent_via = "uart"
            except Exception:
                pass

    if sent_via == "uart":
        acked = True if ack_event is None else ack_event.wait(timeout=wait_timeout)
        return True, sent_via, acked, None

    with esp32_state_lock:
        esp_ip = esp32_device_state.get("ip", "0.0.0.0")

    if esp_ip and esp_ip != "0.0.0.0":
        try:
            r = requests.post(f"http://{esp_ip}{http_path}", json=http_payload, timeout=3.0)
            if 200 <= r.status_code < 300:
                return True, "http", True, r.json() if r.content else {}
        except Exception:
            pass

    return False, None, False, None


def _extract_short_ticket_code(raw_val):
    s = str(raw_val or "").strip()
    if s.startswith("{") and s.endswith("}"):
        try:
            obj = json.loads(s)
            s = str(obj.get("ticket_code") or obj.get("ticketCode") or obj.get("code") or obj.get("qrCode") or s).strip()
        except Exception:
            pass
    return s.replace("\r", "").replace("\n", "").replace("|", "-")[:32]


def notify_esp32_oled_and_beep(oled_status=None, qr_code="", beep_ms=0):
    cmds = []
    if oled_status:
        st_clean = str(oled_status).strip().upper()
        code_clean = _extract_short_ticket_code(qr_code)
        cmds.append(f"OLED:{st_clean}|{code_clean}")
    if beep_ms and int(beep_ms) > 0:
        cmds.append(f"BEEP:{int(beep_ms)}")
    if not cmds:
        return
    payload = ("\n" + "\n".join(cmds) + "\n").encode("utf-8", errors="ignore")
    with serial_lock:
        if serial_connection and serial_connection.is_open:
            try:
                serial_connection.write(payload)
                serial_connection.flush()
            except Exception:
                pass


def classify_api_ticket_oled_feedback(url, qr_code, status_code, resp_json):
    is_checkin = ("check-in" in url)
    is_verify = ("verify-qr" in url)
    if not is_checkin and not is_verify:
        return None, 0, ""

    body = resp_json if isinstance(resp_json, dict) else {}
    data_obj = body.get("data") if isinstance(body.get("data"), dict) else {}
    code = (
        data_obj.get("ticket_code")
        or data_obj.get("ticketCode")
        or data_obj.get("code")
        or qr_code
    )
    status_str = str(data_obj.get("status", "") or body.get("status", "")).upper()
    err_code = str(body.get("errorCode", "")).upper()
    msg_low = str(body.get("message", "")).lower()

    is_used = (
        status_str in ("USED", "CHECKED_IN")
        or bool(data_obj.get("checked_in_at") or data_obj.get("checkedInAt") or data_obj.get("isCheckedIn"))
        or "ALREADY" in err_code
        or "USED" in err_code
        or any(k in msg_low for k in ("already", "đã sử dụng", "đã dùng", "đã check", "checked in", "checked-in"))
    )
    is_ok = (status_code in (200, 201)) and bool(body.get("success")) and not is_used

    if is_used:
        return "USED", 240, code
    if is_ok:
        return ("OK", 90, code) if is_checkin else ("VERIFIED", 60, code)
    if status_code in (401, 403):
        return "ERROR", 240, code
    return "NOTFOUND", 240, code


def rssi_to_quality(rssi):
    if rssi == 0 or rssi <= -100:
        return 0, 0, "Chưa có tín hiệu", "░░░░"
    pct = max(0, min(100, int(2 * (rssi + 100))))
    if rssi >= -55:
        return pct, 4, f"{pct}% (Sóng rất mạnh • {rssi} dBm)", "▂▄▆█"
    elif rssi >= -67:
        return pct, 3, f"{pct}% (Sóng tốt • {rssi} dBm)", "▂▄▆░"
    elif rssi >= -78:
        return pct, 2, f"{pct}% (Sóng trung bình • {rssi} dBm)", "▂▄░░"
    elif rssi >= -88:
        return pct, 1, f"{pct}% (Sóng yếu • {rssi} dBm)", "▂░░░"
    else:
        return pct, 1, f"{pct}% (Sóng rất yếu • {rssi} dBm)", "░░░░"


def check_esp32_latency(ip, port=80, timeout=1.2):
    if not ip or ip == "0.0.0.0":
        return -1
    t0 = time.time()
    try:
        s = socket.create_connection((ip, port), timeout=timeout)
        s.close()
        return int((time.time() - t0) * 1000)
    except Exception:
        return -1


def parse_text_telemetry(raw_bytes):
    global esp32_device_state
    try:
        text = raw_bytes.decode("utf-8", errors="ignore")
        for m in re.finditer(r"ESPSTAT:WIFI=(\d+),IP=([0-9.]+),RSSI=(-?\d+),SSID=([^\r\n]*)", text):
            w_conn = (m.group(1) == "1")
            ip_val = m.group(2)
            rssi_val = int(m.group(3))
            ssid_raw = m.group(4).strip()

            auto_val = None
            has_tok_val = None
            profiles_val = None
            prof_m = re.search(r",PROFILES=(\d+)$", ssid_raw)
            if prof_m:
                profiles_val = int(prof_m.group(1))
                ssid_raw = ssid_raw[:prof_m.start()]
            tok_m = re.search(r",TOKEN=([01])$", ssid_raw)
            if tok_m:
                has_tok_val = (tok_m.group(1) == "1")
                ssid_raw = ssid_raw[:tok_m.start()]
            auto_m = re.search(r",AUTO=([01])$", ssid_raw)
            if auto_m:
                auto_val = (auto_m.group(1) == "1")
                ssid_raw = ssid_raw[:auto_m.start()]
            ssid_val = ssid_raw.strip()

            pct, level, sig_txt, bars = rssi_to_quality(rssi_val if w_conn else -100)
            with esp32_state_lock:
                esp32_device_state["wifi_connected"] = w_conn
                esp32_device_state["ip"] = ip_val
                esp32_device_state["rssi"] = rssi_val
                esp32_device_state["ssid"] = ssid_val
                esp32_device_state["signal_pct"] = pct
                esp32_device_state["signal_level"] = level
                esp32_device_state["signal_text"] = sig_txt
                esp32_device_state["signal_bars"] = bars
                esp32_device_state["status_text"] = "Đã tự kết nối Wi-Fi" if w_conn else "Chưa kết nối Wi-Fi"
                if auto_val is not None:
                    esp32_device_state["auto_confirm"] = auto_val
                if has_tok_val is not None:
                    esp32_device_state["has_token"] = has_tok_val
                if profiles_val is not None:
                    esp32_device_state["wifi_profiles_count"] = profiles_val
                esp32_device_state["last_seen"] = time.time()

            if w_conn and ip_val != "0.0.0.0":
                _save_cached_esp_ip(ip_val)
                with wifi_connect_lock:
                    if wifi_connect_result.get("status") == "connecting":
                        wifi_connect_result["status"] = "success"
                        wifi_connect_result["ip"] = ip_val
                        wifi_connect_result["rssi"] = rssi_val
                        wifi_connect_result["ssid"] = ssid_val
                        wifi_connect_result["message"] = f"Kết nối Wi-Fi '{ssid_val}' thành công! IP: {ip_val}"
                        wifi_connect_event.set()

        for m in re.finditer(r"AUTOCFG:([01])", text):
            auto_enabled = (m.group(1) == "1")
            with esp32_state_lock:
                esp32_device_state["auto_confirm"] = auto_enabled
            esp_auto_event.set()

        for m in re.finditer(r"TOKENSET:([01])", text):
            has_tok = (m.group(1) == "1")
            with esp32_state_lock:
                esp32_device_state["has_token"] = has_tok
            esp_token_event.set()

        for m in re.finditer(r"WIFISAVED:(\d+)", text):
            cnt = int(m.group(1))
            with esp32_state_lock:
                esp32_device_state["wifi_profiles_count"] = cnt
            esp_wifi_profile_event.set()

        for m in re.finditer(r"WIFIRESP:(CONNECTING|CONFIG_APPLIED)", text):
            print(f"[ESP32 TELEMETRY] Bo mạch ESP32 đã nhận lệnh và bắt đầu kết nối Wi-Fi...")
            with wifi_connect_lock:
                wifi_connect_result["esp_acknowledged"] = True

        for m in re.finditer(r"WIFIRESULT:SUCCESS\|([0-9.]+)\|(-?\d+)\|([^\r\n]*)", text):
            ip_val = m.group(1)
            rssi_val = int(m.group(2))
            ssid_val = m.group(3).strip()
            print(f"[ESP32 TELEMETRY] Wi-Fi KẾT NỐI THÀNH CÔNG! IP: {ip_val}, SSID: {ssid_val}, RSSI: {rssi_val}")
            with wifi_connect_lock:
                wifi_connect_result["status"] = "success"
                wifi_connect_result["ip"] = ip_val
                wifi_connect_result["rssi"] = rssi_val
                wifi_connect_result["ssid"] = ssid_val
                wifi_connect_result["reason_str"] = "SUCCESS"
                wifi_connect_result["message"] = f"Kết nối Wi-Fi '{ssid_val}' thành công! IP: {ip_val}"
            wifi_connect_event.set()

        for m in re.finditer(r"WIFIRESULT:FAILED\|(\d+)\|([^\r\n]*)", text):
            reason_code = int(m.group(1))
            reason_str = m.group(2).strip()
            print(f"[ESP32 TELEMETRY] Wi-Fi KẾT NỐI THẤT BẠI! Mã {reason_code}: {reason_str}")
            with wifi_connect_lock:
                wifi_connect_result["status"] = "failed"
                wifi_connect_result["reason_code"] = reason_code
                wifi_connect_result["reason_str"] = reason_str
                if reason_str == "WRONG_PASSWORD":
                    wifi_connect_result["message"] = f"Sai mật khẩu Wi-Fi của mạng '{pending_target_ssid}'! Vui lòng kiểm tra lại mật khẩu."
                elif reason_str == "NO_AP_FOUND":
                    wifi_connect_result["message"] = f"Không tìm thấy mạng Wi-Fi '{pending_target_ssid}'! Vui lòng kiểm tra lại tên Wi-Fi."
                elif reason_str == "ASSOC_FAIL":
                    wifi_connect_result["message"] = f"Mạng Wi-Fi '{pending_target_ssid}' từ chối kết nối!"
                else:
                    wifi_connect_result["message"] = f"Không thể kết nối tới Wi-Fi '{pending_target_ssid}' (Lỗi {reason_code}: {reason_str})."
            wifi_connect_event.set()

        for m in re.finditer(r"WIFISCAN:([^\r\n]*)", text):
            scan_content = m.group(1).strip()
            print(f"[ESP32 TELEMETRY] Nhận danh sách Wi-Fi từ phần cứng ESP32: {scan_content[:80]}")
            if scan_content and not scan_content.startswith("ERROR"):
                with esp_wifi_scan_lock:
                    esp_scanned_networks.clear()
                    for item in scan_content.split(","):
                        if "|" in item:
                            parts = item.split("|")
                            s_ssid = parts[0].strip()
                            try:
                                s_rssi = int(parts[1])
                            except Exception:
                                s_rssi = -70
                            if s_ssid:
                                sig_pct = min(100, max(0, int((s_rssi + 100) * 2)))
                                esp_scanned_networks.append({
                                    "ssid": s_ssid,
                                    "signal": sig_pct,
                                    "rssi": s_rssi,
                                    "band": "2.4 GHz",
                                    "is_24g": True,
                                    "auth": "WPA2-Personal",
                                    "bars": "▂▄▆█" if sig_pct >= 80 else ("▂▄▆░" if sig_pct >= 60 else "▂▄░░"),
                                    "source": "esp32",
                                    "badge": "Ăng-ten ESP32 (2.4G)"
                                })
            esp_scan_event.set()

        for m in re.finditer(r"VERIFYRESP:([^\r\n]*)", text):
            v_content = m.group(1).strip()
            print(f"[ESP32 TELEMETRY] Nhận kết quả đối soát VERIFY từ ESP32 ({len(v_content)} bytes)")
            with esp_verify_lock:
                esp_verify_result["content"] = v_content.encode("utf-8")
                esp_verify_result["status_code"] = 200
            esp_verify_event.set()

        for m in re.finditer(r"CHECKINRESP:([^\r\n]*)", text):
            c_content = m.group(1).strip()
            print(f"[ESP32 TELEMETRY] Nhận kết quả CHECKIN từ ESP32 ({len(c_content)} bytes)")
            with esp_checkin_lock:
                esp_checkin_result["content"] = c_content.encode("utf-8")
                esp_checkin_result["status_code"] = 200
            esp_checkin_event.set()

        for m in re.finditer(r"WIFISTAT:(CONNECTED|DISCONNECTED)\|([0-9.]+)\|(-?\d+)\|([^\r\n]*)", text):
            w_conn = (m.group(1) == "CONNECTED")
            ip_val = m.group(2)
            rssi_val = int(m.group(3))
            ssid_val = m.group(4).strip()
            pct, level, sig_txt, bars = rssi_to_quality(rssi_val if w_conn else -100)
            with esp32_state_lock:
                esp32_device_state["wifi_connected"] = w_conn
                esp32_device_state["ip"] = ip_val
                esp32_device_state["rssi"] = rssi_val
                esp32_device_state["ssid"] = ssid_val
                esp32_device_state["signal_pct"] = pct
                esp32_device_state["signal_level"] = level
                esp32_device_state["signal_text"] = sig_txt
                esp32_device_state["signal_bars"] = bars
                esp32_device_state["status_text"] = "Đã tự kết nối Wi-Fi" if w_conn else "Chưa kết nối Wi-Fi"
                esp32_device_state["last_seen"] = time.time()
    except Exception:
        pass


def get_wifi_diagnostics():
    info = {
        "wifi_connected": False,
        "ssid": "",
        "signal": 0,
        "signal_level": 0,
        "signal_text": "Chưa kết nối Wi-Fi",
        "description": "",
    }
    try:
        creation_flags = 0x08000000 if os.name == "nt" else 0  # CREATE_NO_WINDOW
        res = subprocess.run(
            ["netsh", "wlan", "show", "interfaces"],
            capture_output=True,
            text=True,
            timeout=2.0,
            creationflags=creation_flags,
        )
        for line in res.stdout.splitlines():
            line = line.strip()
            if line.startswith("State") or "Trạng thái" in line:
                val = line.split(":", 1)[-1].strip().lower()
                if ("connected" in val or "kết nối" in val) and "dis" not in val and "ngắt" not in val:
                    info["wifi_connected"] = True
            elif line.startswith("SSID") and not line.startswith("BSSID"):
                info["ssid"] = line.split(":", 1)[-1].strip()
            elif line.startswith("Signal") or "Tín hiệu" in line:
                m = re.search(r"(\d+)%", line)
                if m:
                    sig = int(m.group(1))
                    info["signal"] = sig
                    if sig >= 80:
                        info["signal_level"] = 4
                        info["signal_text"] = f"{sig}% (Sóng rất mạnh)"
                    elif sig >= 60:
                        info["signal_level"] = 3
                        info["signal_text"] = f"{sig}% (Sóng tốt)"
                    elif sig >= 35:
                        info["signal_level"] = 2
                        info["signal_text"] = f"{sig}% (Sóng trung bình)"
                    else:
                        info["signal_level"] = 1
                        info["signal_text"] = f"{sig}% (Sóng yếu)"
            elif line.startswith("Description") or "Mô tả" in line:
                info["description"] = line.split(":", 1)[-1].strip()
    except Exception:
        pass

    if not info["wifi_connected"]:
        info["ssid"] = "Chưa kết nối Wi-Fi"
        info["signal_text"] = "Chưa kết nối Wi-Fi"
        info["signal_level"] = 0

    return info


def scan_nearby_wifi(force_refresh=False):
    global wifi_scan_cache
    now = time.time()
    with wifi_scan_lock:
        if not force_refresh and (now - wifi_scan_cache["timestamp"] < 5.0) and wifi_scan_cache["networks"]:
            return list(wifi_scan_cache["networks"])

    networks = []
    seen_ssids = {}
    try:
        creation_flags = 0x08000000 if os.name == "nt" else 0
        try:
            out = subprocess.check_output(
                ["netsh", "wlan", "show", "networks", "mode=bssid"],
                encoding="utf-8",
                errors="ignore",
                timeout=2.5,
                creationflags=creation_flags
            )
        except Exception:
            out = subprocess.check_output(
                ["netsh", "wlan", "show", "networks"],
                encoding="utf-8",
                errors="ignore",
                timeout=2.0,
                creationflags=creation_flags
            )

        current_net = None

        def commit_net(net):
            if not net or not net.get("ssid"):
                return
            s = net["ssid"]
            if net["signal"] < 0:
                net["signal"] = 70
                net["rssi"] = -65
            if net.get("is_24g") is None:
                # Nếu không có thông tin Band rõ ràng, kiểm tra tên SSID
                if "5g" in s.lower():
                    net["is_24g"] = False
                    net["band"] = "5 GHz"
                else:
                    net["is_24g"] = True
                    net["band"] = net.get("band") or "2.4 GHz"
            if s not in seen_ssids:
                seen_ssids[s] = net
                networks.append(net)
            else:
                existing = seen_ssids[s]
                if net.get("is_24g"):
                    existing["is_24g"] = True
                    existing["band"] = "2.4 GHz"
                if net["signal"] > existing["signal"]:
                    existing["signal"] = net["signal"]
                    existing["rssi"] = net["rssi"]
                    existing["bars"] = net["bars"]

        for line in out.splitlines():
            line = line.strip()
            m_ssid = re.match(r"^SSID\s+\d+\s*:\s*(.*)$", line, re.IGNORECASE)
            if m_ssid:
                commit_net(current_net)
                ssid_val = m_ssid.group(1).strip()
                if not ssid_val:
                    current_net = None
                    continue
                current_net = {
                    "ssid": ssid_val,
                    "signal": -1,
                    "band": "",
                    "is_24g": None,
                    "auth": "WPA2-Personal",
                    "bars": "▂▄▆░",
                    "rssi": -65,
                    "source": "pc",
                }
                continue

            if not current_net:
                continue

            m_auth = re.match(r"^(?:Authentication|Xác thực)\s*:\s*(.*)$", line, re.IGNORECASE)
            if m_auth:
                current_net["auth"] = m_auth.group(1).strip()
                continue

            m_sig = re.match(r"^(?:Signal|Tín hiệu)\s*:\s*(\d+)%", line, re.IGNORECASE)
            if m_sig:
                sig = int(m_sig.group(1))
                if sig > current_net["signal"]:
                    current_net["signal"] = sig
                    current_net["rssi"] = int((sig / 2) - 100)
                    if sig >= 80:
                        current_net["bars"] = "▂▄▆█"
                    elif sig >= 60:
                        current_net["bars"] = "▂▄▆░"
                    elif sig >= 40:
                        current_net["bars"] = "▂▄░░"
                    else:
                        current_net["bars"] = "▂░░░"
                continue

            m_band = re.match(r"^(?:Band|Băng tần)\s*:\s*(.*)$", line, re.IGNORECASE)
            if m_band:
                raw_band = m_band.group(1).strip()
                if "2.4" in raw_band or "2,4" in raw_band:
                    current_net["band"] = "2.4 GHz"
                    current_net["is_24g"] = True
                elif "5" in raw_band:
                    if not current_net.get("is_24g"):
                        current_net["band"] = "5 GHz"
                        current_net["is_24g"] = False
                continue

            m_radio = re.match(r"^(?:Radio type|Loại vô tuyến)\s*:\s*(.*)$", line, re.IGNORECASE)
            if m_radio and current_net.get("is_24g") is None:
                radio = m_radio.group(1).strip().lower()
                if any(x in radio for x in ["802.11a", "802.11ac"]):
                    current_net["band"] = "5 GHz"
                    current_net["is_24g"] = False
                elif any(x in radio for x in ["802.11b", "802.11g", "802.11n"]):
                    current_net["band"] = "2.4 GHz"
                    current_net["is_24g"] = True

        commit_net(current_net)

    except Exception as e:
        print(f"[-] Lỗi scan Wi-Fi PC: {e}")

    # 1. Gộp thêm danh sách Wi-Fi 2.4GHz phát hiện bởi anten phần cứng ESP32
    with esp_wifi_scan_lock:
        for enet in esp_scanned_networks:
            essid = enet.get("ssid")
            if not essid:
                continue
            if essid not in seen_ssids:
                item = dict(enet)
                item["band"] = "2.4 GHz"
                item["is_24g"] = True
                item["source"] = "esp32"
                item["badge"] = "Ăng-ten ESP32 (2.4G)"
                seen_ssids[essid] = item
                networks.append(item)
            else:
                existing = seen_ssids[essid]
                existing["is_24g"] = True
                existing["band"] = "2.4 GHz"
                existing["badge"] = "Ăng-ten ESP32 + PC"
                if enet.get("signal", 0) > existing.get("signal", 0):
                    existing["signal"] = enet["signal"]
                    existing["rssi"] = enet["rssi"]
                    existing["bars"] = enet["bars"]

    # 2. Đảm bảo mạng Wi-Fi hiện tại của máy tính có mặt nếu chưa thấy trong list
    with network_lock:
        pc_ssid = network_state.get("ssid")
        pc_conn = network_state.get("wifi_connected")
        pc_sig = network_state.get("signal", 85)
    if pc_conn and pc_ssid and pc_ssid not in ("Chưa kết nối Wi-Fi", "Không xác định") and pc_ssid not in seen_ssids:
        is_5g = ("5g" in pc_ssid.lower())
        item = {
            "ssid": pc_ssid,
            "signal": pc_sig or 85,
            "band": "5 GHz" if is_5g else "2.4 GHz",
            "is_24g": not is_5g,
            "auth": "WPA2-Personal",
            "bars": "▂▄▆█",
            "rssi": -55,
            "source": "pc_active",
        }
        seen_ssids[pc_ssid] = item
        networks.append(item)

    # Ưu tiên xếp mạng 2.4 GHz lên trên cùng, sau đó xếp theo độ mạnh sóng
    networks.sort(key=lambda x: (1 if x.get("is_24g") else 0, x.get("signal", 0)), reverse=True)
    with wifi_scan_lock:
        wifi_scan_cache["timestamp"] = time.time()
        wifi_scan_cache["networks"] = list(networks)

    return networks


def check_server_latency(host="api.eventhub.id.vn", port=443, timeout=3.0):
    t0 = time.time()
    try:
        s = socket.create_connection((host, port), timeout=timeout)
        s.close()
        latency = int((time.time() - t0) * 1000)
        if latency < 100:
            quality = "excellent"
        elif latency < 300:
            quality = "good"
        elif latency < 800:
            quality = "fair"
        else:
            quality = "slow"
        return latency, True, quality
    except Exception:
        return -1, False, "offline"


def network_monitor_thread():
    global network_state, esp32_device_state

    while True:
        try:
            wifi_info = get_wifi_diagnostics()
            lat_ms, reachable, quality = check_server_latency()

            conn_type = "offline"
            if wifi_info["wifi_connected"]:
                conn_type = "wifi"
            elif reachable:
                conn_type = "ethernet"

            with network_lock:
                network_state = {
                    "wifi_connected": wifi_info["wifi_connected"],
                    "ssid": wifi_info["ssid"],
                    "signal": wifi_info["signal"],
                    "signal_level": wifi_info["signal_level"],
                    "signal_text": wifi_info["signal_text"],
                    "description": wifi_info["description"],
                    "connection_type": conn_type,
                    "latency_ms": lat_ms,
                    "latency_quality": quality,
                    "server_reachable": reachable,
                    "backend_host": "api.eventhub.id.vn",
                    "last_updated": time.time(),
                }

            # Duy trì PC USB Heartbeat để ESP32 biết có máy tính đang điều khiển
            if is_serial_connected:
                with serial_lock:
                    if serial_connection and serial_connection.is_open:
                        try:
                            serial_connection.write(b"\nPCHB:1\n")
                        except Exception:
                            pass

            # Đo độ trễ mạng Wi-Fi tới bo mạch ESP32 & đồng bộ khi chạy độc lập qua Wi-Fi
            esp_ip = "0.0.0.0"
            esp_conn = False
            with esp32_state_lock:
                esp_ip = esp32_device_state.get("ip", "0.0.0.0")
                esp_conn = esp32_device_state.get("wifi_connected", False)

            if esp_ip and esp_ip != "0.0.0.0" and (esp_conn or not is_serial_connected):
                esp_lat = check_esp32_latency(esp_ip)
                if esp_lat > 0:
                    if esp_lat < 50:
                        q = "excellent"
                        txt = f"{esp_lat} ms (Rất nhanh ⚡)"
                    elif esp_lat < 150:
                        q = "good"
                        txt = f"{esp_lat} ms (Tốt)"
                    elif esp_lat < 300:
                        q = "fair"
                        txt = f"{esp_lat} ms (Chấp nhận được)"
                    else:
                        q = "slow"
                        txt = f"{esp_lat} ms (Chậm)"
                else:
                    q = "offline"
                    txt = "Timeout"

                with esp32_state_lock:
                    esp32_device_state["latency_ms"] = esp_lat
                    esp32_device_state["latency_quality"] = q
                    esp32_device_state["latency_text"] = txt

                # Nếu ESP32 chạy độc lập (không cắm USB), fetch /status qua HTTP để đồng bộ cấu hình & trạng thái
                if not is_serial_connected and esp_lat > 0:
                    try:
                        r = requests.get(f"http://{esp_ip}/status", timeout=1.5)
                        if r.status_code == 200:
                            d = r.json()
                            rssi_val = d.get("rssi", -100)
                            w_conn = d.get("wifi_connected", True)
                            ssid_val = d.get("ssid", "")
                            pct, level, sig_txt, bars = rssi_to_quality(rssi_val)
                            with esp32_state_lock:
                                esp32_device_state["wifi_connected"] = w_conn
                                esp32_device_state["ip"] = d.get("ip", esp_ip) or esp_ip
                                esp32_device_state["ssid"] = ssid_val
                                esp32_device_state["rssi"] = rssi_val
                                esp32_device_state["signal_pct"] = pct
                                esp32_device_state["signal_level"] = level
                                esp32_device_state["signal_text"] = sig_txt
                                esp32_device_state["signal_bars"] = bars
                                esp32_device_state["status_text"] = "Đã tự kết nối Wi-Fi" if w_conn else "Chưa kết nối Wi-Fi"
                                if "auto_confirm" in d:
                                    esp32_device_state["auto_confirm"] = bool(d.get("auto_confirm"))
                                if "has_token" in d:
                                    esp32_device_state["has_token"] = bool(d.get("has_token"))
                                if "wifi_profiles_count" in d:
                                    esp32_device_state["wifi_profiles_count"] = int(d.get("wifi_profiles_count") or 0)
                                if "last_scan" in d:
                                    esp32_device_state["last_scan"] = d.get("last_scan")
                                esp32_device_state["last_seen"] = time.time()
                    except Exception:
                        pass
            else:
                with esp32_state_lock:
                    esp32_device_state["latency_ms"] = -1
                    esp32_device_state["latency_quality"] = "offline"
                    esp32_device_state["latency_text"] = "--"
        except Exception:
            pass

        time.sleep(2.5)

WEB_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "web")


def serial_receiver_thread(port_name, baud_rate):
    global latest_frame, frame_counter, current_fps, is_serial_connected, serial_connection

    print(f"[*] Opening serial port {port_name} @ {baud_rate} baud...")
    while True:
        try:
            target_port = port_name
            try:
                import serial.tools.list_ports as lp
                all_ports = lp.comports()
                port_names = [p.device for p in all_ports]
                if target_port not in port_names:
                    candidates = [p.device for p in all_ports if "bluetooth" not in p.description.lower() and "bth" not in p.hwid.lower()]
                    if candidates:
                        target_port = candidates[0]
            except Exception:
                pass

            ser = serial.Serial(target_port, baud_rate, timeout=0.05)
            with serial_lock:
                serial_connection = ser
            is_serial_connected = True
            print(f"[+] Serial port {target_port} successfully connected! Listening for frames...")
            try:
                time.sleep(0.2)
                ser.write(b"AE:-2\nQUAL:15\nPCHB:1\n")
            except Exception:
                pass

            buffer = bytearray()
            fps_start_time = time.time()
            fps_frame_count = 0

            while True:
                to_read = max(1, min(16384, ser.in_waiting)) if ser.in_waiting > 0 else 1
                chunk = ser.read(to_read)
                if not chunk:
                    continue

                buffer.extend(chunk)

                # Look for complete text lines or frames in buffer
                while len(buffer) > 0:
                    idx = buffer.find(SYNC_MAGIC)
                    if idx == -1:
                        last_nl = buffer.rfind(b"\n")
                        if last_nl != -1:
                            parse_text_telemetry(buffer[: last_nl + 1])
                            del buffer[: last_nl + 1]
                        elif len(buffer) > 4096:
                            parse_text_telemetry(buffer[:-3])
                            buffer = buffer[-3:]
                        break

                    if idx > 0:
                        parse_text_telemetry(buffer[:idx])
                        del buffer[:idx]

                    if len(buffer) < 12:
                        break

                    payload_len = struct.unpack("<I", buffer[4:8])[0]
                    if payload_len > 500000 or payload_len < 100:
                        del buffer[:4]
                        continue

                    total_packet_len = 4 + 4 + 4 + payload_len + 4
                    if len(buffer) < total_packet_len:
                        break

                    end_marker = buffer[12 + payload_len : total_packet_len]
                    if end_marker == END_MAGIC:
                        jpeg_data = bytes(buffer[12 : 12 + payload_len])
                        if len(jpeg_data) >= 4 and jpeg_data[:2] == b"\xff\xd8" and jpeg_data[-2:] == b"\xff\xd9":
                            with latest_frame_lock:
                                latest_frame = jpeg_data
                                frame_counter += 1

                            fps_frame_count += 1
                            now = time.time()
                            if now - fps_start_time >= 1.0:
                                current_fps = round(fps_frame_count / (now - fps_start_time), 1)
                                fps_frame_count = 0
                                fps_start_time = now

                        del buffer[:total_packet_len]
                    else:
                        del buffer[:4]

        except serial.SerialException as e:
            with serial_lock:
                serial_connection = None
            is_serial_connected = False
            current_fps = 0.0
            print(f"[-] Serial error: {e}. Reconnecting in 2 seconds...")
            time.sleep(2)
        except Exception as e:
            with serial_lock:
                serial_connection = None
            is_serial_connected = False
            print(f"[-] Unexpected error in serial thread: {e}")
            time.sleep(2)


def normalize_api_base(raw_url):
    url = str(raw_url or "").strip().rstrip("/")
    if not url:
        return "https://api.eventhub.id.vn/api"
    # Cắt bỏ phần route con sau /api nếu có (ví dụ /staff/tickets...)
    if "/api/" in url:
        url = url.split("/api/")[0] + "/api"
    # Sửa lỗi nhập nhầm tên miền frontend eventhub.id.vn -> tự sửa thành subdomain backend api.eventhub.id.vn
    if "eventhub.id.vn" in url and "api.eventhub.id.vn" not in url:
        url = url.replace("eventhub.id.vn", "api.eventhub.id.vn")
    # Đảm bảo có giao thức http:// hoặc https://
    if not url.startswith("http://") and not url.startswith("https://"):
        url = f"https://{url}"
    # Đảm bảo có /api
    if not url.endswith("/api") and "localhost" not in url and not url.endswith(":3000"):
        url = f"{url}/api"
    elif url.endswith(":3000"):
        url = f"{url}/api"
    return url


class CheckinHTTPHandler(BaseHTTPRequestHandler):
    def handle(self):
        try:
            super().handle()
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError, OSError):
            pass
        except Exception as e:
            if any(err_code in str(e) for err_code in ("10053", "10054", "10058")):
                pass
            else:
                raise

    def log_message(self, format, *args):
        if "/stream" in args[0] or "/status" in args[0]:
            return
        super().log_message(format, *args)

    def do_OPTIONS(self):
        self.send_response(200)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Allow-Methods", "GET, POST, PATCH, OPTIONS")
        self.send_header("Access-Control-Allow-Headers", "Content-Type, Authorization")
        self.end_headers()

    def do_GET(self):
        clean_path = urlparse(self.path).path
        if clean_path in ("/", "/index.html"):
            self.serve_file(os.path.join(WEB_DIR, "index.html"), "text/html; charset=utf-8")
        elif clean_path == "/zxing.min.js":
            self.serve_file(os.path.join(WEB_DIR, "zxing.min.js"), "application/javascript")
        elif clean_path == "/favicon.ico":
            self.send_response(204)
            self.end_headers()
        elif clean_path in ("/status", "/api/station-status"):
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            with network_lock:
                net_info = dict(network_state)
            with esp32_state_lock:
                esp_info = dict(esp32_device_state)
            with request_route_lock:
                r_mode = request_route_mode
            status_data = {
                "fps": current_fps,
                "frame_count": frame_counter,
                "port": active_port,
                "baud": active_baud,
                "serial_connected": is_serial_connected,
                "network": net_info,
                "esp32_device": esp_info,
                "route_mode": r_mode,
            }
            self.wfile.write(json.dumps(status_data).encode("utf-8"))
        elif clean_path == "/api/network-status":
            self.send_response(200)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            with network_lock:
                net_info = dict(network_state)
            self.wfile.write(json.dumps(net_info).encode("utf-8"))
        elif clean_path.startswith("/api/wifi-scan"):
            force = ("refresh=1" in self.path or "force=1" in self.path)
            with esp_wifi_scan_lock:
                has_esp_nets = len(esp_scanned_networks) > 0

            if force or not has_esp_nets:
                esp_scan_event.clear()
                sent_scan = False
                with serial_lock:
                    if serial_connection and serial_connection.is_open:
                        try:
                            serial_connection.write(b"\nSCANWIFI\n")
                            serial_connection.flush()
                            sent_scan = True
                        except Exception:
                            pass
                if sent_scan:
                    esp_scan_event.wait(timeout=4.5)

            nets = scan_nearby_wifi(force_refresh=True)
            self.send_json_response(200, {
                "success": True,
                "count": len(nets),
                "networks": nets
            })
        elif clean_path == "/capture":
            with latest_frame_lock:
                frame = latest_frame

            if frame:
                self.send_response(200)
                self.send_header("Content-Type", "image/jpeg")
                self.send_header("Content-Length", str(len(frame)))
                self.send_header("Access-Control-Allow-Origin", "*")
                self.end_headers()
                try:
                    self.wfile.write(frame)
                except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError, OSError):
                    pass
            else:
                self.send_response(503)
                self.end_headers()
                self.wfile.write(b"No frame available yet")
        elif clean_path.startswith("/stream"):
            self.send_response(200)
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()

            last_sent_count = -1
            try:
                while True:
                    with latest_frame_lock:
                        frame = latest_frame
                        f_count = frame_counter

                    if frame and f_count != last_sent_count:
                        last_sent_count = f_count
                        header = (
                            b"--frame\r\n"
                            b"Content-Type: image/jpeg\r\n"
                            + f"Content-Length: {len(frame)}\r\n\r\n".encode("ascii")
                        )
                        self.wfile.write(header)
                        self.wfile.write(frame)
                        self.wfile.write(b"\r\n")
                    else:
                        time.sleep(0.01)
            except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError, OSError):
                pass
            except Exception:
                pass
        else:
            self.send_response(404)
            self.end_headers()

    def do_PATCH(self):
        self.do_POST()

    def do_POST(self):
        global request_route_mode
        content_length = int(self.headers.get("Content-Length", 0))
        post_data = self.rfile.read(content_length)

        try:
            body = json.loads(post_data.decode("utf-8")) if post_data else {}
        except Exception:
            body = {}

        auth_header = self.headers.get("Authorization", "")
        api_base = normalize_api_base(body.get("apiBase", "https://api.eventhub.id.vn/api"))
        clean_path = urlparse(self.path).path

        if clean_path == "/api/proxy/login":
            target_url = f"{api_base}/auth/login"
            payload = {
                "email": body.get("email"),
                "password": body.get("password"),
            }
            self.forward_api_request("POST", target_url, payload, auth_header)

        elif clean_path == "/api/proxy/verify-login-otp":
            target_url = f"{api_base}/auth/verify-login-otp"
            payload = {
                "challengeId": body.get("challengeId"),
                "otp": body.get("otp"),
            }
            self.forward_api_request("POST", target_url, payload, auth_header)

        elif clean_path == "/api/config-route":
            mode = body.get("mode", "pc").lower()
            if mode not in ("pc", "esp32"):
                mode = "pc"
            with request_route_lock:
                request_route_mode = mode
            print(f"[ROUTE CONFIG] Chế độ đối soát vé được chuyển sang: '{mode.upper()}'")
            self.send_json_response(200, {
                "success": True,
                "route_mode": mode,
                "message": f"Đã chuyển luồng đối soát vé sang: {'Mạng Máy Tính (PC)' if mode == 'pc' else 'Wi-Fi ESP32-S3 (Trực tiếp)'}"
            })

        elif clean_path == "/api/esp32/auto-confirm":
            raw_val = body.get("enabled", body.get("auto_confirm", False))
            if isinstance(raw_val, str):
                enabled = raw_val.strip().lower() in ("1", "true", "yes", "on")
            else:
                enabled = bool(raw_val)

            ok, via, acked, http_data = send_esp32_uart_or_http(
                f"SETAUTO:{1 if enabled else 0}",
                esp_auto_event,
                "/api/auto-confirm",
                {"enabled": enabled},
                wait_timeout=0.8,
            )
            if ok:
                with esp32_state_lock:
                    esp32_device_state["auto_confirm"] = enabled
                print(f"[AUTO CONFIRM CFG] Đã lưu auto_confirm={enabled} vào ESP32 NVS qua {via.upper()} (ack={acked})")
                self.send_json_response(200, {
                    "success": True,
                    "auto_confirm": enabled,
                    "via": via,
                    "acked": acked,
                    "message": f"Đã lưu cấu hình Auto Check-in ({'BẬT' if enabled else 'TẮT'}) vào ESP32!"
                })
            else:
                self.send_json_response(503, {
                    "success": False,
                    "auto_confirm": enabled,
                    "message": "Chưa kết nối tới ESP32 (qua USB hoặc Wi-Fi) để lưu cấu hình vào NVS."
                })

        elif clean_path == "/api/esp32/staff-token":
            token_val = str(body.get("token", "")).strip()
            if not token_val and auth_header:
                token_val = auth_header.replace("Bearer ", "").replace("bearer ", "").strip()

            ok, via, acked, _ = send_esp32_uart_or_http(
                f"SETTOKEN:{token_val}",
                esp_token_event,
                "/api/token",
                {"token": token_val},
                wait_timeout=0.8,
            )
            if ok:
                with esp32_state_lock:
                    esp32_device_state["has_token"] = bool(token_val)
                self.send_json_response(200, {
                    "success": True,
                    "has_token": bool(token_val),
                    "via": via,
                    "acked": acked,
                })
            else:
                self.send_json_response(503, {
                    "success": False,
                    "has_token": bool(token_val),
                    "message": "Chưa kết nối tới ESP32 để đồng bộ Staff Token."
                })

        elif clean_path == "/api/esp32/wifi-profiles":
            raw_profiles = body.get("profiles") or []
            if not isinstance(raw_profiles, list):
                self.send_json_response(400, {"success": False, "message": "Danh sách profiles không hợp lệ."})
                return

            synced = 0
            last_via = None
            attempted = 0
            for item in raw_profiles[:4]:
                if not isinstance(item, dict):
                    continue
                s = str(item.get("ssid", "")).replace("\r", "").replace("\n", "").strip()
                p = str(item.get("password", item.get("pass", ""))).replace("\r", "").replace("\n", "")
                if not s or "|" in s:
                    continue
                attempted += 1
                ok, via, _, _ = send_esp32_uart_or_http(
                    f"SAVEWIFI:{s}|{p}",
                    esp_wifi_profile_event,
                    "/api/wifi-profiles",
                    {"ssid": s, "pass": p},
                    wait_timeout=0.35,
                )
                if ok:
                    synced += 1
                    last_via = via

            with esp32_state_lock:
                prof_cnt = esp32_device_state.get("wifi_profiles_count", 0)
            if synced > 0 or attempted == 0:
                self.send_json_response(200, {
                    "success": True,
                    "synced": synced,
                    "via": last_via,
                    "wifi_profiles_count": prof_cnt,
                })
            else:
                self.send_json_response(503, {
                    "success": False,
                    "synced": 0,
                    "message": "Chưa kết nối tới ESP32 để đồng bộ hồ sơ Wi-Fi."
                })

        elif clean_path == "/api/esp32/wifi-config":
            ssid = str(body.get("ssid", "")).strip()
            password = str(body.get("password", "")).strip()
            if not ssid:
                self.send_json_response(400, {"success": False, "message": "Tên mạng Wi-Fi (SSID) không được để trống!"})
                return

            print(f"\n[ESP32 WIFI CONFIG] Bắt đầu kết nối Wi-Fi cho ESP32: SSID='{ssid}'")
            with wifi_connect_lock:
                global pending_target_ssid
                pending_target_ssid = ssid
                wifi_connect_result.clear()
                wifi_connect_result.update({
                    "status": "connecting",
                    "ssid": ssid,
                    "ip": "0.0.0.0",
                    "reason_code": 0,
                    "reason_str": "",
                    "message": "Đang kết nối...",
                })
                wifi_connect_event.clear()

            sent = False
            with serial_lock:
                if serial_connection and serial_connection.is_open:
                    cmd = f"\nSETWIFI:{ssid}|{password}\n"
                    serial_connection.write(cmd.encode("utf-8"))
                    serial_connection.flush()
                    sent = True

            if not sent:
                with esp32_state_lock:
                    esp_ip = esp32_device_state.get("ip", "0.0.0.0")
                if esp_ip and esp_ip != "0.0.0.0":
                    try:
                        requests.post(f"http://{esp_ip}/api/wifi", json={"ssid": ssid, "pass": password}, timeout=4)
                        sent = True
                    except Exception:
                        pass

            if not sent:
                self.send_json_response(503, {
                    "success": False,
                    "message": "Không tìm thấy kết nối USB (Serial COM6) hoặc bo mạch ESP32 để gửi cấu hình Wi-Fi."
                })
                return

            # Chờ ESP32 thực hiện kết nối và phản hồi trong tối đa 15.0 giây
            wifi_connect_event.wait(timeout=15.0)

            with wifi_connect_lock:
                res_status = wifi_connect_result.get("status", "connecting")
                res_ip = wifi_connect_result.get("ip", "0.0.0.0")
                res_rssi = wifi_connect_result.get("rssi", 0)
                res_msg = wifi_connect_result.get("message", "")
                res_err = wifi_connect_result.get("reason_str", "")
                esp_ack = wifi_connect_result.get("esp_acknowledged", False)

            # Fallback check từ esp32_device_state nếu sự kiện chưa kịp trigger
            if res_status != "success" and res_status != "failed":
                with esp32_state_lock:
                    if esp32_device_state.get("wifi_connected") and esp32_device_state.get("ip") != "0.0.0.0":
                        res_status = "success"
                        res_ip = esp32_device_state.get("ip")
                        res_rssi = esp32_device_state.get("rssi", 0)
                        res_msg = f"Kết nối Wi-Fi '{ssid}' thành công! IP: {res_ip}"

            if res_status == "success":
                self.send_json_response(200, {
                    "success": True,
                    "ip": res_ip,
                    "rssi": res_rssi,
                    "ssid": ssid,
                    "message": res_msg or f"Kết nối Wi-Fi '{ssid}' thành công! ESP32 đã nhận địa chỉ IP: {res_ip}"
                })
            elif res_status == "failed":
                self.send_json_response(400, {
                    "success": False,
                    "errorCode": res_err or "CONNECT_FAILED",
                    "message": res_msg or f"Kết nối tới Wi-Fi '{ssid}' thất bại. Vui lòng kiểm tra lại mật khẩu hoặc sóng Wi-Fi."
                })
            else:
                if not esp_ack:
                    self.send_json_response(408, {
                        "success": False,
                        "errorCode": "FIRMWARE_UNRESPONSIVE",
                        "message": f"Bo mạch ESP32 chưa nhận được lệnh Wi-Fi (Cần nạp firmware mới bằng flash_firmware.bat hoặc khởi động lại mạch)."
                    })
                else:
                    self.send_json_response(408, {
                        "success": False,
                        "errorCode": "DHCP_TIMEOUT",
                        "message": f"ESP32 đã phát lệnh kết nối tới mạng '{ssid}' nhưng quá thời gian cấp IP (DHCP Timeout > 15s). Vui lòng thử lại hoặc kiểm tra xem router có đang chặn thiết bị lạ hay không."
                    })

        elif clean_path == "/api/proxy/verify-qr":
            qr_text = str(body.get("qrCode", "")).strip()
            print(f"\n=======================================================")
            print(f"[QR DETECTED] Read Code: '{qr_text}' (Length: {len(qr_text)} chars)")
            if not qr_text:
                print(f"   [!] Cảnh báo: Mã QR rỗng! Không gửi đối soát.")
                print(f"=======================================================")
                self.send_json_response(400, {
                    "success": False,
                    "errorCode": "EMPTY_QR",
                    "message": "Mã QR rỗng. Vui lòng căn chỉnh mã QR vào giữa khung hình camera."
                })
                return

            if not auth_header:
                print(f"   [Staff Auth]: Cảnh báo - Chưa có Token Staff! Trả về 401 yêu cầu đăng nhập hoặc dùng Test Offline.")
                print(f"=======================================================")
                self.send_json_response(401, {
                    "success": False,
                    "errorCode": "AUTH_REQUIRED",
                    "message": "Chưa đăng nhập tài khoản Nhân Sự (Staff). Vui lòng đăng nhập ở cột bên phải hoặc bật Chế Độ Test Offline để xem vé ngay.",
                    "scannedCode": qr_text
                })
                return

            with request_route_lock:
                curr_route = request_route_mode

            print(f"   [Staff Auth]: Token Attached (Luồng gửi: {curr_route.upper()})...")

            # Giải mã và hiển thị thông tin tài khoản đăng nhập từ JWT Token
            try:
                parts = auth_header.replace("Bearer ", "").strip().split(".")
                if len(parts) >= 2:
                    pad = len(parts[1]) % 4
                    p_str = parts[1] + ("=" * (4 - pad) if pad > 0 else "")
                    payload_raw = base64.urlsafe_b64decode(p_str.encode("utf-8")).decode("utf-8")
                    token_info = json.loads(payload_raw)
                    user_sub = str(token_info.get("sub", ""))
                    user_roles = [str(r).upper() for r in token_info.get("roles", [])]
                    print(f"   [Staff Account]: Sub: {user_sub[:8]}... | Roles: {user_roles}")
                    if "STAFF" not in user_roles:
                        print(f"   [!] CẢNH BÁO: Tài khoản này KHÔNG CÓ vai trò 'STAFF' (Hiện tại: {user_roles})!")
                        print(f"       -> Backend EventHub sẽ trả về 403 AUTH_FORBIDDEN nếu không có quyền STAFF.")
            except Exception:
                pass

            # Route: ESP32 Wi-Fi (Gửi trực tiếp từ ESP32-S3 qua Wi-Fi 2.4GHz)
            if curr_route == "esp32":
                self.forward_via_esp32("verify", qr_text, auth_header)
                return

            # Route: PC Network (Mặc định)
            print(f"   [PC Route]: Đang gửi đối soát từ máy tính tới EventHub Backend: {api_base}...")
            print(f"=======================================================")
            target_url = f"{api_base}/staff/tickets/verify-qr"
            payload = {
                "qrCode": qr_text,
            }
            self.forward_api_request("POST", target_url, payload, auth_header)

        elif clean_path == "/api/proxy/check-in":
            qr_text = str(body.get("qrCode", ""))
            print(f"\n[CHECK-IN REQUEST] Checking in ticket: '{qr_text}'")

            with request_route_lock:
                curr_route = request_route_mode

            if curr_route == "esp32":
                if not qr_text:
                    self.send_json_response(400, {"success": False, "message": "Missing qrCode for ESP32 check-in"})
                    return
                self.forward_via_esp32("check-in", qr_text, auth_header)
                return

            if body.get("qrCode"):
                target_url = f"{api_base}/staff/tickets/check-in/qr"
                payload = {"qrCode": body.get("qrCode")}
                self.forward_api_request("POST", target_url, payload, auth_header)
            elif body.get("ticketId"):
                target_url = f"{api_base}/staff/tickets/{body.get('ticketId')}/check-in"
                self.forward_api_request("PATCH", target_url, {}, auth_header)
            else:
                self.send_json_response(400, {"success": False, "message": "Missing qrCode or ticketId"})

        elif clean_path == "/api/beep":
            duration = int(body.get("duration", 90))
            oled_status = body.get("oled_status")
            qr_code = body.get("qr_code", "")
            notify_esp32_oled_and_beep(oled_status=oled_status, qr_code=qr_code, beep_ms=duration)
            self.send_json_response(200, {"success": True})

        elif clean_path == "/api/proxy/profile":
            target_url = f"{api_base}/users/profile"
            self.forward_api_request("GET", target_url, None, auth_header)

        elif clean_path == "/api/cam-settings":
            with serial_lock:
                if serial_connection and serial_connection.is_open:
                    cmd_parts = []
                    for k, v in body.items():
                        if k == "ae":
                            cmd_parts.append(f"AE:{int(v)}")
                        elif k == "contrast":
                            cmd_parts.append(f"CONTRAST:{int(v)}")
                        elif k == "sharpness":
                            cmd_parts.append(f"SHARP:{int(v)}")
                        elif k == "brightness":
                            cmd_parts.append(f"BRIGHT:{int(v)}")
                        elif k == "quality":
                            cmd_parts.append(f"QUAL:{int(v)}")
                        elif k == "af" and v:
                            cmd_parts.append("AF:TRIG")
                        elif k == "res":
                            cmd_parts.append(f"RES:{v}")
                    if cmd_parts:
                        full_cmd = "\n".join(cmd_parts) + "\n"
                        serial_connection.write(full_cmd.encode("ascii"))
                    self.send_json_response(200, {"success": True, "sent": cmd_parts})
                else:
                    self.send_json_response(503, {"success": False, "message": "Serial port not connected"})

        else:
            self.send_response(404)
            self.end_headers()

    def _send_proxied_json_response(self, status_code, raw_bytes, route_name):
        """Send forwarded API JSON while tagging the network route that fulfilled it."""
        self.send_response(status_code)
        self.send_header("Content-Type", "application/json")
        self.send_header("X-Checkin-Route", route_name)
        self.send_header("Access-Control-Allow-Origin", "*")
        self.send_header("Access-Control-Expose-Headers", "X-Checkin-Route")
        self.end_headers()
        self.wfile.write(raw_bytes)

    def forward_via_esp32(self, action, qr_text, auth_header):
        with esp32_state_lock:
            esp_conn = esp32_device_state.get("wifi_connected", False)
            esp_ip = esp32_device_state.get("ip", "0.0.0.0")

        if not esp_conn or esp_ip == "0.0.0.0":
            print("   [!] Bo mạch ESP32 chưa kết nối Wi-Fi 2.4GHz!")
            self.send_json_response(400, {
                "success": False,
                "errorCode": "ESP32_WIFI_DISCONNECTED",
                "message": "Bo mạch ESP32 chưa kết nối Wi-Fi 2.4GHz! Vui lòng kết nối Wi-Fi 2.4GHz cho ESP32 ở khung cài đặt bên dưới, hoặc chuyển sang Mạng Máy Tính (PC)."
            })
            return

        token_raw = auth_header.replace("Bearer ", "").strip()
        clean_code = qr_text
        if qr_text.startswith("{") and qr_text.endswith("}"):
            try:
                parsed_qr = json.loads(qr_text)
                clean_code = parsed_qr.get("ticket_code") or parsed_qr.get("code") or parsed_qr.get("ticketCode") or clean_code
            except Exception:
                pass

        is_verify = (action == "verify")
        uart_prefix = "VERIFY" if is_verify else "CHECKIN"
        http_endpoint = "/api/verify" if is_verify else "/api/check-in"
        ev = esp_verify_event if is_verify else esp_checkin_event
        lk = esp_verify_lock if is_verify else esp_checkin_lock
        res_dict = esp_verify_result if is_verify else esp_checkin_result

        # Ưu tiên gửi lệnh qua USB UART để ESP32 dùng sóng Wi-Fi 2.4GHz của chính nó gửi HTTPS
        with serial_lock:
            ser_open = (serial_connection is not None and serial_connection.is_open)

        if ser_open:
            print(f"   [ESP32 Route]: Gửi lệnh {uart_prefix} qua USB Serial -> ESP32 dùng Wi-Fi 2.4GHz gửi HTTPS...")
            with lk:
                res_dict["content"] = b""
                res_dict["status_code"] = 500
            ev.clear()

            cmd = f"\n{uart_prefix}:{clean_code}|{token_raw}\n"
            with serial_lock:
                if serial_connection and serial_connection.is_open:
                    serial_connection.write(cmd.encode("utf-8"))
                    serial_connection.flush()

            if ev.wait(timeout=12.0):
                with lk:
                    raw_resp = res_dict.get("content", b"")
                if raw_resp:
                    print(f"   [ESP32 Response via UART]: {len(raw_resp)} bytes")
                    status_to_send = 200
                    try:
                        parsed = json.loads(raw_resp.decode("utf-8", errors="ignore"))
                        if isinstance(parsed, dict):
                            if parsed.get("statusCode") and isinstance(parsed.get("statusCode"), int):
                                status_to_send = parsed["statusCode"]
                            elif parsed.get("success") is False:
                                msg_lower = str(parsed.get("message", "")).lower()
                                err_code = str(parsed.get("errorCode", "")).upper()
                                if "401" in err_code or "unauthorized" in msg_lower:
                                    status_to_send = 401
                                elif "403" in err_code or "forbidden" in msg_lower:
                                    status_to_send = 403
                                elif "loi ket noi https" in msg_lower:
                                    status_to_send = 502
                    except Exception:
                        pass
                    self._send_proxied_json_response(status_to_send, raw_resp, "esp32")
                    return

        # Fallback sang HTTP Local nếu chạy độc lập (không cắm dây USB)
        if esp_ip and esp_ip != "0.0.0.0":
            print(f"   [ESP32 Route]: Chuyển tiếp HTTP tới http://{esp_ip}{http_endpoint}...")
            try:
                esp_resp = requests.post(
                    f"http://{esp_ip}{http_endpoint}",
                    json={"qrCode": clean_code, "token": token_raw},
                    timeout=10
                )
                print(f"   [ESP32 Response via HTTP]: HTTP {esp_resp.status_code} ({len(esp_resp.content)} bytes)")
                self._send_proxied_json_response(esp_resp.status_code, esp_resp.content, "esp32")
                return
            except Exception as e:
                print(f"   [-] Lỗi gửi qua ESP32 HTTP ({e})")

        self.send_json_response(504, {
            "success": False,
            "errorCode": "ESP32_TIMEOUT",
            "message": "ESP32 không phản hồi khi gửi yêu cầu qua Wi-Fi 2.4GHz (Timeout > 12s). Vui lòng kiểm tra kết nối Wi-Fi của ESP32 hoặc chuyển sang Mạng Máy Tính (PC)."
        })

    def forward_api_request(self, method, url, data, auth_header):
        headers = {
            "Content-Type": "application/json",
            "User-Agent": "Mozilla/5.0 (Windows NT 10.0; Win64; x64) AppleWebKit/537.36 (KHTML, like Gecko) Chrome/122.0.0.0 Safari/537.36 EventHubStation/1.0",
            "Accept": "application/json, text/plain, */*",
        }
        if auth_header:
            headers["Authorization"] = auth_header

        # Tự động phòng vệ nếu URL còn trỏ nhầm sang frontend eventhub.id.vn
        if "eventhub.id.vn" in url and "api.eventhub.id.vn" not in url:
            url = url.replace("eventhub.id.vn", "api.eventhub.id.vn")
        if "/api/" not in url and not url.endswith("/api") and "api.eventhub.id.vn" in url:
            url = url.replace("api.eventhub.id.vn", "api.eventhub.id.vn/api")

        try:
            r = send_session_request(method, url, data, headers, timeout=12)

            # Tự động khắc phục nếu nhận 405 Method Not Allowed từ host frontend
            if r.status_code == 405 and "api.eventhub.id.vn/api" not in url:
                retry_url = url.replace("eventhub.id.vn", "api.eventhub.id.vn")
                if "/api/" not in retry_url and not retry_url.endswith("/api"):
                    retry_url = retry_url.replace("api.eventhub.id.vn", "api.eventhub.id.vn/api")
                print(f"   [!] Server trả về 405! Đang tự động retry sang API chuẩn: {retry_url}...")
                try:
                    r = send_session_request(method, retry_url, data, headers, timeout=12)
                    url = retry_url
                except Exception as retry_err:
                    print(f"   [-] Retry thất bại: {retry_err}")

            print(f"   [API Response]: HTTP {r.status_code} ({len(r.content)} bytes)")
            resp_json = None
            try:
                resp_json = r.json()
                print(f"   [API Body]: {json.dumps(resp_json, ensure_ascii=False)[:300]}")
            except Exception:
                print(f"   [API Text]: {r.text[:200]}")

            qr_hint = (data.get("qrCode") if isinstance(data, dict) else "") or ""
            oled_st, beep_ms, code_hint = classify_api_ticket_oled_feedback(url, qr_hint, r.status_code, resp_json)

            if r.status_code == 401:
                print("   [!] 401 Unauthorized: Cần đăng nhập tài khoản nhân sự (Staff) trên giao diện Web!")
            elif r.status_code == 403:
                print("   [!] 403 Forbidden: Tài khoản hiện tại không có quyền STAFF cho sự kiện này (AUTH_FORBIDDEN)!")
                print("       -> Vui lòng bấm [Đăng Xuất] trên Web và đăng nhập lại bằng tài khoản Staff được phân công.")
            elif r.status_code == 405:
                print(f"   [!] 405 Method Not Allowed từ {url}!")
                self.send_json_response(405, {
                    "success": False,
                    "statusCode": 405,
                    "errorCode": "METHOD_NOT_ALLOWED",
                    "message": f"Máy chủ phản hồi 405 Method Not Allowed khi gửi tới '{url}'. Đang tự động điều chỉnh về https://api.eventhub.id.vn/api.",
                    "url": url
                })
                return

            if oled_st:
                notify_esp32_oled_and_beep(oled_status=oled_st, qr_code=code_hint, beep_ms=beep_ms)

            self._send_proxied_json_response(r.status_code, r.content, "pc")

        except requests.exceptions.Timeout:
            with network_lock:
                lat = network_state.get("latency_ms", -1)
                wifi_txt = network_state.get("signal_text", "Chưa kết nối")
            self.send_json_response(504, {
                "success": False,
                "statusCode": 504,
                "message": f"Kết nối tới máy chủ EventHub quá thời gian (Timeout >12s). Mạng hoặc Wi-Fi chập chờn ({wifi_txt}, độ trễ: {lat}ms). Vui lòng kiểm tra lại mạng hoặc dùng Chế Độ Test Offline."
            })
        except requests.exceptions.ConnectionError as e:
            self.send_json_response(502, {
                "success": False,
                "statusCode": 502,
                "message": f"Mất kết nối mạng tới {url}. Vui lòng kiểm tra lại kết nối Wi-Fi."
            })
        except Exception as e:
            self.send_json_response(500, {
                "success": False,
                "statusCode": 500,
                "message": f"Lỗi Proxy: {str(e)}"
            })

    def send_json_response(self, code, obj):
        try:
            self.send_response(code)
            self.send_header("Content-Type", "application/json")
            self.send_header("Access-Control-Allow-Origin", "*")
            self.end_headers()
            self.wfile.write(json.dumps(obj).encode("utf-8"))
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError, OSError):
            pass

    def serve_file(self, filepath, content_type):
        if not os.path.exists(filepath):
            try:
                self.send_response(404)
                self.end_headers()
            except Exception:
                pass
            return

        with open(filepath, "rb") as f:
            data = f.read()

        try:
            self.send_response(200)
            self.send_header("Content-Type", content_type)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-cache, no-store, must-revalidate")
            self.send_header("Pragma", "no-cache")
            self.send_header("Expires", "0")
            self.end_headers()
            self.wfile.write(data)
        except (ConnectionResetError, BrokenPipeError, ConnectionAbortedError, OSError):
            pass


def main():
    global active_port, active_baud

    parser = argparse.ArgumentParser(description="EventHub ESP32-S3 USB Stream & QR Check-in Server")
    parser.add_argument("--port", default=DEFAULT_PORT, help=f"Serial port (default: {DEFAULT_PORT})")
    parser.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"Baud rate (default: {DEFAULT_BAUD})")
    parser.add_argument("--http-port", type=int, default=HTTP_PORT, help=f"HTTP Server port (default: {HTTP_PORT})")
    args = parser.parse_args()

    active_port = args.port
    active_baud = args.baud

    # Start Serial Reader Thread
    ser_thread = threading.Thread(target=serial_receiver_thread, args=(active_port, active_baud), daemon=True)
    ser_thread.start()

    # Start Real-Time Network & Wi-Fi Monitor Thread
    net_thread = threading.Thread(target=network_monitor_thread, daemon=True)
    net_thread.start()

    # Start Multi-Threaded HTTP Server
    server_address = ("", args.http_port)
    httpd = ThreadingHTTPServer(server_address, CheckinHTTPHandler)
    httpd.daemon_threads = True

    print("=" * 60)
    print(f" EventHub USB Check-in Station running at: http://localhost:{args.http_port}")
    print(f" Reading camera stream from: {active_port} @ {active_baud} baud")
    print(f" Live Stream URL: http://localhost:{args.http_port}/stream")
    print("=" * 60)

    try:
        httpd.serve_forever()
    except KeyboardInterrupt:
        print("\n[!] Shutting down server...")
        httpd.server_close()


if __name__ == "__main__":
    main()
