#!/usr/bin/env python3
"""
Минимальный веб-сервер для страницы с регулируемой линией.
Отдаёт index.html и эмулирует API-ответы.
"""

import http.server
import socketserver
import json
import os
import time
import re
import sys
from urllib.parse import urlparse

PORT = 8000
DIRECTORY = os.path.dirname(os.path.abspath(__file__))
POSITION_FILE = os.path.join(DIRECTORY, 'position.json')

# Порт микроконтроллера и время ожидания
SERIAL_PORT = 'COM3'
BAUDRATE = 115200
TIMEOUT = 2 

# Константы движения
SPEED = 30.0
ACCEL = 50.0
DECEL = 50.0
CALIB_SPEED = 10.0
MAX_STROKE_MM = 50

# Глобальные переменнные
ser = None # Хранит объект последовательного порта. Инициализация None означает, что порт ещё не открыт или не доступен.
emulation = False
current_x = 0.0
current_y = 0.0
calibrated = False

# Попытка импорта pyserial
# если ошибка - работаем в режиме эмуляции (без подключения к оборудованию)
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("⚠️ pyserial не установлен. Установите: pip install pyserial")
    print("   Сервер будет работать в режиме ЭМУЛЯЦИИ.")

# ============================== РАБОТА С ПОРТОМ  ==============================
# Попытка открыть последовательный порт
def open_serial():
    global ser, emulation
    if not SERIAL_AVAILABLE:
        emulation = True
        return
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
        ser.flushInput()
        ser.flushOutput()
        print(f"✅ Порт {SERIAL_PORT} открыт на скорости {BAUDRATE} бод")
        emulation = False
    except Exception as e:
        print(f"Не удалось открыть порт {SERIAL_PORT}: {e}")
        print(" Сервер переходит в режим ЭМУЛЯЦИИ.")
        emulation = True
        ser = None

# Закрытие порта
def close_serial():
    global ser
    if ser and not emulation:
        ser.close()
        print("Порт закрыт.")

# Получить статус подключения
def get_connection_status():
    if emulation:
        return "emulation"
    elif ser is not None and ser.is_open:
        return "connected"
    else:
        return "disconnected"







# ============================== ОБРАБОТКА HTTP ==============================
class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def log_message(self, format, *args):
        print(f"[{self.log_date_time_string()}] {format % args}")

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/ping':
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
            return
        super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        if parsed.path == '/calibrate':
            # Эмуляция калибровки
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"success": True}).encode('utf-8'))
            return
        elif parsed.path == '/move':
            # Эмуляция движения
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length).decode('utf-8')
            try:
                data = json.loads(body)
            except:
                self.send_error(400, "Invalid JSON")
                return
            axis = data.get('axis')
            amount = data.get('amount')
            if axis not in ('x', 'y') or amount not in (1, -1):
                self.send_error(400, "Invalid parameters")
                return
            # Возвращаем успех и фиктивную позицию (0,0)
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"success": True, "position": {"x": 0, "y": 0}}).encode('utf-8'))
            return
        else:
            self.send_error(404, "Not found")

# ============================== ЗАПУСК WEB-СЕРВЕРА ==============================
def start_serial():
    global emulation
    print ("Пытаемся подключить Teensy")
    open_serial()
    if emulation:
        print ("Работаем в эмуляции без реального оборудования")
    else:
        print ("Teensy подключена успешно (Порт: {SERIAL_PORT})")

def main():
    global emulation
    start_serial()

    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print("=" * 50)
        print(f"  Сервер запущен на http://localhost:{PORT}")
        print("  (Эмуляция — реальное оборудование не задействовано)")
        print("  Для остановки нажмите Ctrl+C")
        print("=" * 50)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nСервер остановлен.")

if __name__ == "__main__":
    main()