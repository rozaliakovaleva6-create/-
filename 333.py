import serial
import math
import time
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# --- НАСТРОЙКА ПОРТА ---
PORT = 'COM9'          # Убедитесь, что COM-порт верный
BAUD_RATE = 115200
GYRO_UNITS = "deg"     # "deg" (часто) или "rad" (если шлёшь rad/s)
MAX_DT_SEC = 0.1       # ограничение всплесков dt (сек)
ORIENTATION_MODE = "gyro"  # "gyro" = ориентация ТОЛЬКО по гироскопу
ANGLE_UNITS = "deg"    # если Arduino шлёт углы (roll,pitch,yaw) — обычно deg

try:
    ser = serial.Serial(PORT, BAUD_RATE, timeout=0.1)
    print(f"✅ Подключено к {PORT}")
except Exception as e:
    print(f"❌ Ошибка Serial: {e}")
    raise

# --- ГЕОМЕТРИЯ КУБА ---
points = np.array([
    [-1, -1, -1], [1, -1, -1], [1, 1, -1], [-1, 1, -1],
    [-1, -1,  1], [1, -1,  1], [1, 1,  1], [-1, 1,  1]
])
edges = [[0,1], [1,2], [2,3], [3,0], [4,5], [5,6], [6,7], [7,4], [0,4], [1,5], [2,6], [3,7]]

# Переменные вращения (yaw/pitch/roll) в РАДИАНАХ
cur_yaw = 0.0
cur_roll = 0.0
cur_pitch = 0.0

# Смещения гироскопа (в рад/с)
gyro_offset_x = 0.0
gyro_offset_y = 0.0
gyro_offset_z = 0.0

# Офсеты для углов (когда Arduino шлёт 3 числа roll,pitch,yaw)
angle_offset_roll = 0.0
angle_offset_pitch = 0.0
angle_offset_yaw = 0.0
angle_offset_ready = False
last_angles_rad = None  # (roll, pitch, yaw) в rad, последние принятые углы (до офсета)

# Защита от "скачков" по Serial (битая строка может выглядеть как сброс калибровки)
MAX_ANGLE_RATE_RAD_S = 25.0  # примерно 1430 deg/s

last_time = time.time()
_last_error_print_time = 0.0
_last_debug_print_time = 0.0

# --- ГРАФИКА ---
fig = plt.figure(figsize=(8, 6))
ax = fig.add_subplot(111, projection='3d')
ax.set_xlim([-2, 2]); ax.set_ylim([-2, 2]); ax.set_zlim([-2, 2])
lines = [ax.plot([], [], [], color='blue', lw=2)[0] for _ in range(len(edges))]
top_mark, = ax.plot([], [], [], 'ro', markersize=10)

def _gyro_to_rad_per_sec(v: float) -> float:
    return math.radians(v) if GYRO_UNITS == "deg" else v

def _angle_to_rad(v: float) -> float:
    return math.radians(v) if ANGLE_UNITS == "deg" else v

def _parse_csv_floats(line: str):
    """
    Поддерживаем 2 формата:
    - 6 чисел: ax,ay,az,gx,gy,gz
    - 3 числа: roll,pitch,yaw (углы), если Arduino считает их сама

    Также пропускаем строки с префиксами типа:
    - "CALIBRATING_GYRO: ..."
    - "GYRO_BIAS_DPS:x,y,z"
    """
    raw = line.strip()
    if not raw:
        return None

    # ЯВНО игнорируем сервисные строки Arduino, чтобы их числа не приняли за углы
    upper = raw.upper()
    if upper.startswith("CALIBRATING_GYRO") or upper.startswith("CALIBRATING"):
        return None
    if upper.startswith("GYRO_BIAS_DPS"):
        return None

    s = raw
    if not s:
        return None

    # если есть префикс "XXX:...", попробуем взять часть после последнего ":"
    if ":" in s:
        s = s.split(":")[-1].strip()

    parts = s.split(',')
    if len(parts) not in (3, 6):
        return None
    try:
        return [float(p) for p in parts]
    except ValueError:
        return None

def calibrate_sensor(seconds: float = 3.0):
    """Калибровка смещения гироскопа: не трогай плату seconds секунд."""
    global gyro_offset_x, gyro_offset_y, gyro_offset_z
    print(f"\n⚠️  КАЛИБРОВКА! НЕ ТРОГАЙ ПЛАТУ {seconds:.0f} СЕК...")

    samples = 0
    gx_sum = 0.0
    gy_sum = 0.0
    gz_sum = 0.0

    start = time.time()
    ser.reset_input_buffer()

    while time.time() - start < seconds:
        if ser.in_waiting <= 0:
            continue
        try:
            line = ser.readline().decode('utf-8', errors='ignore')
            vals = _parse_csv_floats(line)
            if vals is None or len(vals) != 6:
                continue
            _, _, _, gx_raw, gy_raw, gz_raw = vals
            gx_sum += _gyro_to_rad_per_sec(gx_raw)
            gy_sum += _gyro_to_rad_per_sec(gy_raw)
            gz_sum += _gyro_to_rad_per_sec(gz_raw)
            samples += 1
        except Exception:
            pass

    if samples <= 0:
        print("❌ Калибровка не удалась: нет данных")
        return

    gyro_offset_x = gx_sum / samples
    gyro_offset_y = gy_sum / samples
    gyro_offset_z = gz_sum / samples
    print(f"✅ Готово! offsets(rad/s): x={gyro_offset_x:.4f} y={gyro_offset_y:.4f} z={gyro_offset_z:.4f}")

def zero_angles_to_current():
    """Сделать текущую ориентацию нулём (для режима 3 чисел)."""
    global angle_offset_roll, angle_offset_pitch, angle_offset_yaw, angle_offset_ready
    if last_angles_rad is None:
        return
    angle_offset_roll, angle_offset_pitch, angle_offset_yaw = last_angles_rad
    angle_offset_ready = True
    print("✅ Ноль установлен (углы).")

def get_rotation_matrix(yaw, pitch, roll):
    # Полная матрица поворота (Yaw * Pitch * Roll)
    Rz = np.array([[math.cos(yaw), -math.sin(yaw), 0],
                   [math.sin(yaw),  math.cos(yaw), 0],
                   [0, 0, 1]])
    Ry = np.array([[math.cos(pitch), 0, math.sin(pitch)],
                   [0, 1, 0],
                   [-math.sin(pitch), 0, math.cos(pitch)]])
    Rx = np.array([[1, 0, 0],
                   [0, math.cos(roll), -math.sin(roll)],
                   [0, math.sin(roll),  math.cos(roll)]])
    return np.dot(Rz, np.dot(Ry, Rx))

def update(frame):
    global cur_yaw, cur_roll, cur_pitch, last_time, _last_error_print_time, _last_debug_print_time
    global angle_offset_roll, angle_offset_pitch, angle_offset_yaw, angle_offset_ready, last_angles_rad

    now = time.time()
    dt = now - last_time
    if dt < 0:
        dt = 0.0
    elif dt > MAX_DT_SEC:
        dt = MAX_DT_SEC
    last_time = now

    if ser.in_waiting > 0:
        try:
            # Берём самую свежую строку из буфера
            data_block = ser.read(ser.in_waiting).decode('utf-8', errors='ignore').split('\n')
            # Ищем последнюю НЕпустую строку (data_block[-1] часто пустая из-за '\n')
            line = ""
            for candidate in reversed(data_block):
                c = candidate.strip()
                if c:
                    line = c
                    break
            vals = _parse_csv_floats(line)
            if vals is not None:
                if len(vals) == 3:
                    # Arduino шлёт готовые углы: roll,pitch,yaw (обычно в градусах)
                    roll_a, pitch_a, yaw_a = vals
                    roll_r = _angle_to_rad(roll_a)
                    pitch_r = _angle_to_rad(pitch_a)
                    yaw_r = _angle_to_rad(yaw_a)

                    # Фильтр "глюков": если внезапно пришёл огромный скачок — игнорируем пакет
                    if last_angles_rad is not None and dt > 0:
                        max_step = MAX_ANGLE_RATE_RAD_S * dt * 2.0  # небольшой запас
                        if (
                            abs(roll_r - last_angles_rad[0]) > max_step
                            or abs(pitch_r - last_angles_rad[1]) > max_step
                            or abs(yaw_r - last_angles_rad[2]) > max_step
                        ):
                            # просто пропускаем этот пакет
                            return lines + [top_mark]

                    last_angles_rad = (roll_r, pitch_r, yaw_r)

                    # При первом валидном пакете делаем "ноль" автоматически
                    if not angle_offset_ready:
                        angle_offset_roll, angle_offset_pitch, angle_offset_yaw = last_angles_rad
                        angle_offset_ready = True
                        print("✅ Ноль установлен (первый пакет углов).")

                    cur_roll = roll_r - angle_offset_roll
                    cur_pitch = pitch_r - angle_offset_pitch
                    cur_yaw = yaw_r - angle_offset_yaw

                    if now - _last_debug_print_time > 0.5:
                        _last_debug_print_time = now
                        print(f"ANGLES({ANGLE_UNITS}): roll={roll_a:>8.3f} pitch={pitch_a:>8.3f} yaw={yaw_a:>8.3f}")

                elif len(vals) == 6:
                    ax_v, ay_v, az_v, gx_raw, gy_raw, gz_raw = vals

                    # Приводим гироскоп к рад/с и вычитаем смещения
                    gx = _gyro_to_rad_per_sec(gx_raw) - gyro_offset_x
                    gy = _gyro_to_rad_per_sec(gy_raw) - gyro_offset_y
                    gz = _gyro_to_rad_per_sec(gz_raw) - gyro_offset_z

                    # ОРИЕНТАЦИЯ ТОЛЬКО ПО ГИРОСКОПУ (акселерометр не влияет на картинку)
                    cur_roll += gx * dt
                    cur_pitch += gy * dt
                    cur_yaw += gz * dt

                    # Редкий debug, чтобы видеть, что данные приходят
                    if now - _last_debug_print_time > 0.5:
                        _last_debug_print_time = now
                        print(
                            f"ACC: x={ax_v:>7.3f} y={ay_v:>7.3f} z={az_v:>7.3f} | "
                            f"GYRO(rad/s): x={gx:>7.3f} y={gy:>7.3f} z={gz:>7.3f}"
                        )

        except Exception as e:
            if now - _last_error_print_time > 1.0:
                _last_error_print_time = now
                print(f"❌ Ошибка чтения/парсинга Serial: {e}")

    # Отрисовка
    R = get_rotation_matrix(cur_yaw, cur_pitch, cur_roll)
    rotated = np.dot(points, R.T)
    for i, edge in enumerate(edges):
        p1, p2 = rotated[edge[0]], rotated[edge[1]]
        lines[i].set_data([p1[0], p2[0]], [p1[1], p2[1]])
        lines[i].set_3d_properties([p1[2], p2[2]])
    
    top_c = (rotated[4]+rotated[5]+rotated[6]+rotated[7])/4
    top_mark.set_data([top_c[0]], [top_c[1]])
    top_mark.set_3d_properties([top_c[2]])

    return lines + [top_mark]

def on_key(event):
    global cur_yaw, cur_roll, cur_pitch, last_time
    if event.key == 'r':
        # Сброс ориентации в ноль
        cur_yaw = cur_roll = cur_pitch = 0.0
        last_time = time.time()
        zero_angles_to_current()
        print("Сброс позиции!")
    elif event.key == 'c':
        # Перекалибровка гироскопа (актуально для режима 6 чисел)
        calibrate_sensor(3.0)
        cur_yaw = cur_roll = cur_pitch = 0.0
        last_time = time.time()
        print("Перекалибровка гироскопа завершена.")

fig.canvas.mpl_connect('key_press_event', on_key)

# Калибровка гироскопа (если по Serial идёт 6 чисел — поможет убрать дрейф скорости).
# Если Arduino шлёт 3 числа (готовые углы), калибровка скорости тут не нужна — ноль ставится автоматически.
calibrate_sensor(3.0)

ani = FuncAnimation(fig, update, interval=10, cache_frame_data=False)
try:
    plt.show()
finally:
    ser.close()