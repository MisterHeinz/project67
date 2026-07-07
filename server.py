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
MIN_STROKE_MM = -50
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
    print("ERROR: pyserial не установлен. Установите: pip install pyserial")
    print("Сервер будет работать в режиме ЭМУЛЯЦИИ.")

# ============================== РАБОТА С ПОРТОМ ==============================
def open_serial():
    global ser, emulation
    if not SERIAL_AVAILABLE:
        emulation = True
        return
    try:
        ser = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)
        ser.reset_input_buffer()
        ser.reset_output_buffer()
        print(f" Порт {SERIAL_PORT} открыт на скорости {BAUDRATE} бод")
        emulation = False
    except Exception as e:
        print(f"ERROR: Не удалось открыть порт {SERIAL_PORT}: {e}")
        print("Сервер переходит в режим ЭМУЛЯЦИИ.")
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
        # В эмуляции возвращаем фиктивный ответ с текущими координатами
        response = f"RMRESULT_SUCCESSOX{current_x:.5f}OY{current_y:.5f}OZ0.00000OA0.00000"
        print(f"[EMU] <- {response}")
        return True, response
    
    try:
        ser.write((cmd + '\n').encode('utf-8'))
        ser.flush()
        print(f"[SER] -> {cmd}")
        response = ser.readline().decode('utf-8').strip()
        print(f"[SER] <- {response}")
        return True, response
    except Exception as e:
        print(f"ERROR: Ошибка при обмене: {e}")
        return False, None

def parse_response(response):
    if not response:
        return None, None
    ox_match = re.search(r'OX([\d.]+)', response)
    oy_match = re.search(r'OY([\d.]+)', response)
    if ox_match and oy_match:
        return float(ox_match.group(1)), float(oy_match.group(1))
    return None, None

# ============================== ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ДЛЯ ОТНОСИТЕЛЬНОГО ДВИЖЕНИЯ ==============================
def calc_relative_move(axis, current, amount_mm):
    """
    Вычисляет фактическое смещение с учётом границ MIN_STROKE_MM..MAX_STROKE_MM.
    Возвращает словарь с actual_mm, ox, oy, new_pos, либо None и сообщение об ошибке.
    """
    if axis == 'x':
        ox, oy = amount_mm, 0.0
    elif axis == 'y':
        ox, oy = 0.0, amount_mm
    else:
        return None, {"success": False, "error": "Неизвестная ось"}

    max_possible = MAX_STROKE_MM - current
    min_possible = MIN_STROKE_MM - current

    if amount_mm > 0:
        actual_mm = min(amount_mm, max_possible)
    else:
        actual_mm = max(amount_mm, min_possible)  # amount_mm отрицательный

    if actual_mm == 0:
        # уже на границе, движения нет
        return None, {"success": True, "position": {"x": current_x, "y": current_y}, "note": "already at limit"}

    new_pos = current + actual_mm
    if new_pos < MIN_STROKE_MM or new_pos > MAX_STROKE_MM:
        return None, {"success": False, "error": f"Выход за границы ({MIN_STROKE_MM}..{MAX_STROKE_MM} мм)"}

    return {
        'actual_mm': actual_mm,
        'ox': ox if axis == 'x' else 0.0,
        'oy': oy if axis == 'y' else 0.0,
        'new_pos': new_pos
    }, None

# ============================== ОТНОСИТЕЛЬНОЕ ДВИЖЕНИЕ ==============================
def do_move(axis, amount_cm):
    global current_x, current_y
    amount_mm = amount_cm * 10  # 1 см = 10 мм
    print(f"[SYS] do_move: axis={axis}, amount_cm={amount_cm}, amount_mm={amount_mm}")

    # Выбираем текущую позицию для оси
    current = current_x if axis == 'x' else current_y

    params, err = calc_relative_move(axis, current, amount_mm)
    if err is not None:
        print(f"[SYS] Ошибка в calc_relative_move: {err}")
        return err
    if params is None:
        # если движения нет (уже на границе)
        print("[SYS] Движение не требуется (уже на границе)")
        return {"success": True, "position": {"x": current_x, "y": current_y}, "note": "already at limit"}

    actual_mm = params['actual_mm']
    ox = params['ox']
    oy = params['oy']
    new_pos = params['new_pos']
    print(f"[SYS] Вычисленное фактическое смещение: actual_mm={actual_mm}, ox={ox}, oy={oy}, new_pos={new_pos}")

    cmd = f"RMOX{ox:.1f}OY{oy:.1f}OZ0.0OA0.0SP{SPEED:.1f}AC{ACCEL:.1f}DC{DECEL:.1f}"
    ok, response = send_command(cmd)
    if not ok:
        return {"success": False, "error": "Ошибка отправки команды"}

    # Обновляем координаты
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
            # fallback
            if axis == 'x':
                current_x = new_pos
            else:
                current_y = new_pos
            print("ERROR: Не удалось распарсить ответ, использую расчётную позицию")
    print(f"[SYS] Текущие координаты после движения: X={current_x:.2f}, Y={current_y:.2f}")
    return {"success": True, "position": {"x": current_x, "y": current_y}}

# ============================== АБСОЛЮТНОЕ ПЕРЕМЕЩЕНИЕ ==============================
def do_move_absolute(axis, target_mm):
    global current_x, current_y
    print(f"[SYS] do_move_absolute: axis={axis}, target_mm={target_mm}")

    if target_mm < MIN_STROKE_MM or target_mm > MAX_STROKE_MM:
        print(f"[SYS] Цель вне границ ({MIN_STROKE_MM}..{MAX_STROKE_MM} мм)")
        return {"success": False, "error": f"Цель вне границ ({MIN_STROKE_MM}..{MAX_STROKE_MM} мм)"}

    if axis == 'x':
        ox = target_mm
        oy = current_y
    else:  # axis == 'y'
        ox = current_x
        oy = target_mm
    print(f"[SYS] Формируем команду AM: OX={ox}, OY={oy}")

    cmd = f"AMOX{ox:.1f}OY{oy:.1f}OZ0.0OA0.0SP{SPEED:.1f}AC{ACCEL:.1f}DC{DECEL:.1f}"
    ok, response = send_command(cmd)
    if not ok:
        return {"success": False, "error": "Ошибка отправки команды AM"}

    # Обновляем координаты
    if emulation:
        if axis == 'x':
            current_x = ox
        else:
            current_y = oy
    else:
        x, y = parse_response(response)
        if x is not None and y is not None:
            current_x = x
            current_y = y
        else:
            if axis == 'x':
                current_x = ox
            else:
                current_y = oy
            print("ERROR: Не удалось распарсить ответ, использую заданные координаты")
    print(f"[SYS] Текущие координаты после AM: X={current_x:.2f}, Y={current_y:.2f}")
    return {"success": True, "position": {"x": current_x, "y": current_y}}

# ============================== ВОЗВРАЩЕНИЕ В 0 ==============================
def do_go_home():
    global current_x, current_y
    print("[SYS] do_go_home вызван")
    if current_x == 0.0 and current_y == 0.0:
        return {"success": True, "position": {"x": 0.0, "y": 0.0}}

    # Вычисляем дельту для возврата в 0.
    ox_needed = -current_x
    oy_needed = -current_y
    print(f"[SYS] Необходимые смещения: OX={ox_needed}, OY={oy_needed}")

    cmd = f"RMOX{ox_needed:.1f}OY{oy_needed:.1f}OZ0.0OA0.0SP{SPEED:.1f}AC{ACCEL:.1f}DC{DECEL:.1f}"
    ok, response = send_command(cmd)
    if not ok:
        return {"success": False, "error": "Ошибка отправки команды возврата"}

    # Принудительно выставляем в 0, так как мы вернулись домой
    current_x = 0.0
    current_y = 0.0
    return {"success": True, "position": {"x": current_x, "y": current_y}}

# ============================== ОБРАБОТКА HTTP ==============================
class Handler(http.server.SimpleHTTPRequestHandler):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, directory=DIRECTORY, **kwargs)

    def log_message(self, format, *args):
        print(f"[HTTP] {format % args}")

    def do_GET(self):
        parsed = urlparse(self.path)
        if parsed.path == '/ping':
            print("[HTTP] GET /ping")
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"status": "ok"}).encode('utf-8'))
            return
        super().do_GET()

    def do_POST(self):
        parsed = urlparse(self.path)
        print(f"[HTTP] POST {parsed.path}")

        if parsed.path == '/calibrate':
            global current_x, current_y
            current_x = 0.0
            current_y = 0.0
            print("[SYS] Произведена калибровка: текущая позиция принята за 0.0")
            self.send_response(200)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps({"success": True}).encode('utf-8'))
            return

        elif parsed.path == '/go_home':
            result = do_go_home()
            status = 200 if result.get('success') else 400
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
            return

        elif parsed.path == '/move':
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length).decode('utf-8')
            try:
                data = json.loads(body)
            except:
                self.send_error(400, "Invalid JSON")
                return
            axis = data.get('axis')
            amount = data.get('amount')
            print(f"[HTTP] /move: axis={axis}, amount={amount}")
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

        elif parsed.path == '/move_abs':
            content_length = int(self.headers.get('Content-Length', 0))
            body = self.rfile.read(content_length).decode('utf-8')
            try:
                data = json.loads(body)
            except:
                self.send_error(400, "Invalid JSON")
                return
            axis = data.get('axis')
            target_mm = data.get('target_mm')
            print(f"[HTTP] /move_abs: axis={axis}, target_mm={target_mm}")
            if axis not in ('x', 'y') or target_mm is None:
                self.send_error(400, "Invalid parameters")
                return
            result = do_move_absolute(axis, float(target_mm))
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
        print("ERROR: не удалось подключить Teensy")
        print("Работаем в эмуляции без реального оборудования")
    else:
        print(f"Teensy подключена успешно (Порт: {SERIAL_PORT})")

def main():
    start_serial()

    with socketserver.TCPServer(("", PORT), Handler) as httpd:
        print("=" * 50)
        print(f" Сервер запущен на http://localhost:{PORT}")
        print("  (Эмуляция — реальное оборудование не задействовано)")
        print("  Для остановки нажмите Ctrl+C")
        print("=" * 50)
        try:
            httpd.serve_forever()
        except KeyboardInterrupt:
            print("\nСервер остановлен.")
            close_serial()

if __name__ == "__main__":
    main()