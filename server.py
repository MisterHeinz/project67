#!/usr/bin/env python3
"""
ДЛЯ СЕРЕГИ - РЕЖИМ ЭМУЛЯЦИИ НУЖЕН ДЛЯ ПРОВЕРКИ РАБОТЫ БЕЗ ПОДКЛЮЧЕНИЯ КОНТРОЛЛЕРА
"""

import http.server
import socketserver
import json
import os
import time
import re
from urllib.parse import urlparse

PORT = 8000
DIRECTORY = os.path.dirname(os.path.abspath(__file__))

# Порт микроконтроллера и время ожидания
SERIAL_PORT = 'COM3'
BAUDRATE = 115200
TIMEOUT = 2

# Константы движения
SPEED = 30.0
ACCEL = 50.0
DECEL = 50.0
MAX_STROKE_MM = 50

# Глобальные переменные
ser = None
emulation = False
current_x = 0.0
current_y = 0.0

# Попытка импорта pyserial
try:
    import serial
    SERIAL_AVAILABLE = True
except ImportError:
    SERIAL_AVAILABLE = False
    print("pyserial не установлен. Установите: pip install pyserial")
    print("   Сервер будет работать в режиме ЭМУЛЯЦИИ.")

# ============================== РАБОТА С ПОРТОМ ==============================
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

def close_serial():
    global ser
    if ser and not emulation:
        ser.close()
        print("Порт закрыт.")

# ============================== РАБОТА С КОНТРОЛЛЕРОМ ==============================
def send_command(cmd):
    if emulation or ser is None:
        print(f"[EMU] -> {cmd}")
        time.sleep(0.05)
        # Возвращаем фиктивный ответ, но координаты будут обновлены в do_move отдельно
        return True, "RMRESULT_SUCCESSOX0.00000OY0.00000OZ0.00000OA0.00000"
    try:
        ser.write((cmd + '\n').encode('utf-8'))
        ser.flush()
        print(f"[SER] -> {cmd}")
        response = ser.readline().decode('utf-8').strip()
        print(f"[SER] <- {response}")
        return True, response
    except Exception as e:
        print(f"Ошибка при обмене: {e}")
        return False, None

def parse_response(response):
    if not response:
        return None, None
    ox_match = re.search(r'OX([\d.]+)', response)
    oy_match = re.search(r'OY([\d.]+)', response)
    if ox_match and oy_match:
        return float(ox_match.group(1)), float(oy_match.group(1))
    return None, None

def do_move(axis, amount_cm):
    global current_x, current_y
    amount_mm = amount_cm * 10

    # Проверка границ
    if axis == 'x':
        new_pos = current_x + amount_mm
        if new_pos < 0 or new_pos > MAX_STROKE_MM:
            return {"success": False, "error": f"Выход за границы (0..{MAX_STROKE_MM} мм)"}
        ox = amount_mm
        oy = 0.0
    elif axis == 'y':
        new_pos = current_y + amount_mm
        if new_pos < 0 or new_pos > MAX_STROKE_MM:
            return {"success": False, "error": f"Выход за границы (0..{MAX_STROKE_MM} мм)"}
        ox = 0.0
        oy = amount_mm
    else:
        return {"success": False, "error": "Неизвестная ось"}

    cmd = f"RMOX{ox:.1f}OY{oy:.1f}OZ0.0OA0.0SP{SPEED:.1f}AC{ACCEL:.1f}DC{DECEL:.1f}"
    ok, response = send_command(cmd)
    if not ok:
        return {"success": False, "error": "Ошибка отправки команды"}

    # В эмуляции обновляем координаты расчётными значениями
    if emulation:
        if axis == 'x':
            current_x = new_pos
        else:
            current_y = new_pos
    else:
        x, y = parse_response(response)
        if x is not None and y is not None:
            current_x = x
            current_y = y
        else:
            # Если не удалось распарсить, используем расчётное значение
            if axis == 'x':
                current_x = new_pos
            else:
                current_y = new_pos
            print("Не удалось распарсить ответ, использую расчётную позицию")

    return {"success": True, "position": {"x": current_x, "y": current_y}}

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
        if parsed.path == '/move':
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
            result = do_move(axis, amount)
            status = 200 if result.get('success') else 400
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
            return
        else:
            self.send_error(404, "Not found")

# ============================== ЗАПУСК ==============================
def start_serial():
    global emulation
    print("Пытаемся подключить Teensy")
    open_serial()
    if emulation:
        print("Работаем в эмуляции без реального оборудования")
    else:
        print(f"Teensy подключена успешно (Порт: {SERIAL_PORT})")

def main():
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