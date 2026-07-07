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
TIMEOUT = 3  # таймаут для readline

# Константы движения
SPEED = 20.0
ACCEL = 20.0
DECEL = 20.0
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
    """
    Отправляет команду и читает ответ, возвращает (success, response).
    В цикле ожидает ответ, пока не получит непустую строку или не истечёт таймаут.
    """
    if emulation or ser is None:
        print(f"[EMU] -> {cmd}")
        time.sleep(0.05)
        # Эмулируем ответ в зависимости от команды
        if cmd.startswith('HR'):
            response = f"HROK OX{current_x:.5f} OY{current_y:.5f}"
        elif cmd.startswith('HW'):
            # Извлекаем параметры OX и OY из команды
            ox_match = re.search(r'OX([\d.]+)', cmd)
            oy_match = re.search(r'OY([\d.]+)', cmd)
            if ox_match and oy_match:
                ox = float(ox_match.group(1))
                oy = float(oy_match.group(1))
            else:
                ox = current_x
                oy = current_y
            response = f"HWOK OX{ox:.5f} OY{oy:.5f}"
        elif cmd.startswith('MH'):
            # В эмуляции считаем, что дом = 0
            response = f"MHOK OX0.00000 OY0.00000"
        elif cmd.startswith('RM'):
            # RM — относительное движение, извлекаем OX и OY из команды
            ox_match = re.search(r'OX([\d.]+)', cmd)
            oy_match = re.search(r'OY([\d.]+)', cmd)
            if ox_match:
                ox = float(ox_match.group(1))
                current_x += ox
            if oy_match:
                oy = float(oy_match.group(1))
                current_y += oy
            # Ограничиваем координаты
            current_x = max(MIN_STROKE_MM, min(MAX_STROKE_MM, current_x))
            current_y = max(MIN_STROKE_MM, min(MAX_STROKE_MM, current_y))
            response = f"RMRESULT_SUCCESSOX{current_x:.5f}OY{current_y:.5f}OZ0.00000OA0.00000"
        else:
            response = "OK"
        print(f"[EMU] <- {response}")
        return True, response

    # Реальный режим
    try:
        ser.reset_input_buffer()
        ser.write((cmd + '\n').encode('utf-8'))
        ser.flush()
        print(f"[SER] -> {cmd}")

        # Цикл ожидания ответа с таймаутом
        start_time = time.time()
        response = ""
        while time.time() - start_time < 5.0:  # максимум 5 секунд ждём
            if ser.in_waiting > 0:
                response = ser.readline().decode('utf-8').strip()
                if response:
                    break
            else:
                time.sleep(0.05)  # небольшая пауза, чтобы не грузить процессор

        if response:
            print(f"[SER] <- {response}")
        else:
            print("[SER] Таймаут: ответ не получен")
            # Принудительно сбрасываем буфер и возвращаем ошибку
            ser.reset_input_buffer()
            return False, None

        return True, response
    except Exception as e:
        print(f"ERROR: Ошибка при обмене: {e}")
        try:
            ser.reset_input_buffer()
        except:
            pass
        return False, None

def parse_coordinates(response):
    """Извлекает OX и OY из ответа (HROK, HWOK, MHOK, RMRESULT_SUCCESS)."""
    if not response:
        return None, None
    ox_match = re.search(r'OX([\d.]+)', response)
    oy_match = re.search(r'OY([\d.]+)', response)
    if ox_match and oy_match:
        return float(ox_match.group(1)), float(oy_match.group(1))
    return None, None

# ============================== КОМАНДЫ УПРАВЛЕНИЯ ==============================
def do_read_home():
    """
    Читает текущие координаты домашнего положения (команда HR).
    Обновляет current_x и current_y из ответа контроллера.
    """
    global current_x, current_y
    print("[SYS] Чтение домашних координат: отправка HR...")
    cmd = "HR"
    ok, response = send_command(cmd)
    if not ok:
        print("[SYS] Ошибка отправки HR, координаты остаются по умолчанию")
        return {"success": False, "error": "Ошибка отправки HR"}

    x, y = parse_coordinates(response)
    if x is not None and y is not None:
        current_x = x
        current_y = y
        print(f"[SYS] Домашние координаты прочитаны: X={current_x:.2f}, Y={current_y:.2f}")
        return {"success": True, "position": {"x": current_x, "y": current_y}}
    else:
        print("[SYS] Не удалось распарсить ответ HR, координаты остаются по умолчанию")
        return {"success": False, "error": "Ошибка парсинга HR"}

def do_calibrate():
    """
    Калибровка: отправляет HW с текущими координатами, чтобы записать их как домашнее положение.
    Координаты сервера НЕ МЕНЯЕМ.
    """
    global current_x, current_y
    print(f"[SYS] Калибровка: отправка HW с координатами OX{current_x:.1f} OY{current_y:.1f}...")
    cmd = f"HW OX{current_x:.1f} OY{current_y:.1f}"
    ok, response = send_command(cmd)
    if not ok:
        return {"success": False, "error": "Ошибка отправки HW"}

    # Координаты сервера остаются прежними, т.к. физическое положение не изменилось
    print("[SYS] Калибровка выполнена. Координаты сервера не изменились.")
    return {"success": True, "position": {"x": current_x, "y": current_y}}

def do_go_home():
    """
    Перемещение в домашнее положение: отправляет MH.
    После выполнения обновляет координаты сервера из ответа MHOK.
    """
    global current_x, current_y
    print("[SYS] Перемещение в домашнее положение: отправка MH...")
    cmd = f"MHSP{SPEED:.1f}AC{ACCEL:.1f}DC{DECEL:.1f}"
    ok, response = send_command(cmd)
    if not ok:
        return {"success": False, "error": "Ошибка отправки MH"}

    x, y = parse_coordinates(response)
    if x is not None and y is not None:
        current_x = x
        current_y = y
        print(f"[SYS] Перемещение домой завершено. Координаты сервера: X={current_x:.2f}, Y={current_y:.2f}")
    else:
        print("[SYS] Не удалось распарсить ответ MH, координаты сервера не изменены.")
    return {"success": True, "position": {"x": current_x, "y": current_y}}

def do_move_absolute(axis, target_mm):
    """
    Абсолютное перемещение через относительное движение.
    Вычисляет дельту (target - current) и отправляет RM.
    """
    global current_x, current_y
    print(f"[SYS] do_move_absolute: axis={axis}, target_mm={target_mm}")

    if target_mm < MIN_STROKE_MM or target_mm > MAX_STROKE_MM:
        print(f"[SYS] Цель вне границ ({MIN_STROKE_MM}..{MAX_STROKE_MM} мм)")
        return {"success": False, "error": f"Цель вне границ ({MIN_STROKE_MM}..{MAX_STROKE_MM} мм)"}

    if axis == 'x':
        delta = target_mm - current_x
        new_pos = current_x + delta
        if new_pos < MIN_STROKE_MM or new_pos > MAX_STROKE_MM:
            return {"success": False, "error": f"Выход за границы ({MIN_STROKE_MM}..{MAX_STROKE_MM} мм)"}
        ox = delta
        oy = 0.0
    else:
        delta = target_mm - current_y
        new_pos = current_y + delta
        if new_pos < MIN_STROKE_MM or new_pos > MAX_STROKE_MM:
            return {"success": False, "error": f"Выход за границы ({MIN_STROKE_MM}..{MAX_STROKE_MM} мм)"}
        ox = 0.0
        oy = delta

    print(f"[SYS] Относительное смещение: delta={delta}, ox={ox}, oy={oy}")

    if abs(delta) < 0.01:
        return {"success": True, "position": {"x": current_x, "y": current_y}, "note": "already at position"}

    cmd = f"RMOX{ox:.1f}OY{oy:.1f}OZ0.0OA0.0SP{SPEED:.1f}AC{ACCEL:.1f}DC{DECEL:.1f}"
    ok, response = send_command(cmd)
    if not ok:
        return {"success": False, "error": "Ошибка отправки RM"}

    x, y = parse_coordinates(response)
    if x is not None and y is not None:
        current_x = x
        current_y = y
        print(f"[SYS] Текущие координаты после RM: X={current_x:.2f}, Y={current_y:.2f}")
    else:
        print("[SYS] Не удалось распарсить ответ RM, координаты сервера не изменены.")
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
            result = do_calibrate()
            status = 200 if result.get('success') else 400
            self.send_response(status)
            self.send_header('Content-Type', 'application/json')
            self.end_headers()
            self.wfile.write(json.dumps(result).encode('utf-8'))
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
            amount_mm = amount * 10
            current = current_x if axis == 'x' else current_y
            target = current + amount_mm
            result = do_move_absolute(axis, target)
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
    # Читаем текущие координаты домашнего положения (HR)
    print("[SYS] Чтение домашних координат (HR)...")
    result = do_read_home()
    if result.get('success'):
        print("[SYS] Домашние координаты успешно получены")
    else:
        print("[SYS] Не удалось прочитать домашние координаты, используем 0,0")

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