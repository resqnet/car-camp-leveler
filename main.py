from machine import Pin, I2C, PWM, reset
from mpu6050 import MPU6050
from ssd1306 import SSD1306_I2C
from time import sleep, ticks_ms
from math import atan2, degrees

# 定数の定義
I2C_MPU_ID = 0
I2C_OLED_ID = 1
SCL_MPU = 9
SDA_MPU = 8
SCL_OLED = 15
SDA_OLED = 14
BUZZER_PIN = 5

BUTTON_SCREEN_PIN = 27       # 画面切替用ボタン (GP27)
SOUND_STOP_BUTTON_PIN = 21   # 音停止用ボタン (GP21)

OLED_WIDTH = 128
OLED_HEIGHT = 64

ALPHA = 0.95           # コンプリメントフィルタ係数
FLAT_THRESHOLD = 5     # 平坦状態の判定閾値 (度)
CALIBRATION_TIME = 3   # キャリブレーション前の待機時間 (秒)
NUM_SAMPLES = 100      # キャリブレーション用サンプル数
DEBOUNCE_DELAY = 0.3   # ボタンチャタリング対策用スリープ時間

STARTUP_BEEP_DELAY = 2000  # 起動後2秒未満はFLATならブザーOFF（ミリ秒）

# 追加：MPU6050の温度取得用関数
def get_temperature(mpu):
    hi = mpu.i2c.readfrom_mem(mpu.addr, 0x41, 1)
    lo = mpu.i2c.readfrom_mem(mpu.addr, 0x42, 1)
    raw_temp = (hi[0] << 8) | lo[0]
    if raw_temp & 0x8000:  # 負の値の場合
        raw_temp = -((raw_temp ^ 0xFFFF) + 1)
    temp_c = (raw_temp / 340.0) + 36.53
    return temp_c

def init_i2c():
    """MPU-6050 用と OLED 用の I2C バスを初期化"""
    i2c_mpu = I2C(I2C_MPU_ID, scl=Pin(SCL_MPU), sda=Pin(SDA_MPU))
    i2c_oled = I2C(I2C_OLED_ID, scl=Pin(SCL_OLED), sda=Pin(SDA_OLED))
    return i2c_mpu, i2c_oled

def init_devices(i2c_mpu, i2c_oled):
    """各デバイス（MPU, OLED, ブザー, ボタン）を初期化"""
    mpu = MPU6050(i2c_mpu)
    oled = SSD1306_I2C(OLED_WIDTH, OLED_HEIGHT, i2c_oled)
    buzzer = PWM(Pin(BUZZER_PIN))
    buzzer.deinit()  # 起動時はブザーOFF
    screen_button = Pin(BUTTON_SCREEN_PIN, Pin.IN, Pin.PULL_UP)
    sound_stop_button = Pin(SOUND_STOP_BUTTON_PIN, Pin.IN, Pin.PULL_UP)
    return mpu, oled, buzzer, screen_button, sound_stop_button

def show_startup_screen(oled):
    """起動画面の表示"""
    oled.fill(0)
    oled.text("Car-Camp-Leveler", 0, 20)
    oled.text("Loading...", 30, 40)
    oled.show()
    sleep(2)

def calibrate_gyro(mpu):
    """ジャイロのキャリブレーション処理"""
    print("キャリブレーション中... 3秒間動かさないでください")
    sleep(CALIBRATION_TIME)
    gyro_offset_x = 0
    gyro_offset_y = 0
    for _ in range(NUM_SAMPLES):
        gyro = mpu.get_gyro_data()
        gyro_offset_x += gyro['x']
        gyro_offset_y += gyro['y']
        sleep(0.01)
    gyro_offset_x /= NUM_SAMPLES
    gyro_offset_y /= NUM_SAMPLES
    print("キャリブレーション完了")
    return gyro_offset_x, gyro_offset_y

def draw_circle(oled, x0, y0, radius):
    """中点円アルゴリズムでOLED上に円を描画"""
    x = radius
    y = 0
    err = 0
    while x >= y:
        oled.pixel(x0 + x, y0 + y, 1)
        oled.pixel(x0 + y, y0 + x, 1)
        oled.pixel(x0 - y, y0 + x, 1)
        oled.pixel(x0 - x, y0 + y, 1)
        oled.pixel(x0 - x, y0 - y, 1)
        oled.pixel(x0 - y, y0 - x, 1)
        oled.pixel(x0 + y, y0 - x, 1)
        oled.pixel(x0 + x, y0 - y, 1)
        y += 1
        err += 1 + 2 * y
        if 2 * (err - x) + 1 > 0:
            x -= 1
            err += 1 - 2 * x

def update_display_text(oled, angle_x, angle_y, is_flat, accel):
    """画面１：テキスト表示モードの更新（追加情報あり）"""
    oled.text("Text Mode", 0, 0)
    oled.text("X: {} deg".format(int(angle_x)), 0, 10)
    oled.text("Y: {} deg".format(int(angle_y)), 0, 20)
    oled.text("Acc: {:>4.1f} {:>4.1f} {:>4.1f}".format(accel['x'], accel['y'], accel['z']), 0, 30)
    if is_flat:
        oled.text("FLAT", 0, OLED_HEIGHT - 8)
    else:
        oled.text("Tilted", 0, OLED_HEIGHT - 8)

def update_display_bubble(oled, angle_x, angle_y, is_flat):
    """画面２：バブルレベル表示モードの更新"""
    oled.hline(0, OLED_HEIGHT // 2, OLED_WIDTH, 1)
    oled.vline(OLED_WIDTH // 2, 0, OLED_HEIGHT, 1)
    bubble_x = int((OLED_WIDTH // 2) - angle_x * 2)
    bubble_y = int((OLED_HEIGHT // 2) + angle_y * 2)
    draw_circle(oled, bubble_x, bubble_y, 2)
    if is_flat:
        oled.text("FLAT", 0, OLED_HEIGHT - 8)
    else:
        oled.text("Tilted", 0, OLED_HEIGHT - 8)

def update_display_temp(oled, temp, min_temp, max_temp):
    """画面３：温度表示モードの更新
       ・現在温度、最小／最大温度
       ・下部に温度バーグラフ（20～50℃の範囲）
    """
    oled.text("Temperature", 0, 0)
    oled.text("{:.1f} C".format(temp), 0, 10)
    oled.text("Min: {:.1f}".format(min_temp), 0, 30)
    oled.text("Max: {:.1f}".format(max_temp), 0, 40)
    bar_width = int((temp - 20) / 30 * OLED_WIDTH)
    if bar_width < 0:
        bar_width = 0
    if bar_width > OLED_WIDTH:
        bar_width = OLED_WIDTH
    oled.fill_rect(0, OLED_HEIGHT - 10, bar_width, 8, 1)

def main():
    i2c_mpu, i2c_oled = init_i2c()
    mpu, oled, buzzer, screen_button, sound_stop_button = init_devices(i2c_mpu, i2c_oled)
    show_startup_screen(oled)
    print("Car-Camp-Leveler 起動中...")
    gyro_offset_x, gyro_offset_y = calibrate_gyro(mpu)

    # 初期状態の設定
    angle_x = 0
    angle_y = 0
    last_time = ticks_ms()
    startup_time = last_time
    # デフォルト画面はバブル表示（画面モード：2）
    screen_mode = 2

    # 温度画面用：最小／最大温度の初期化
    min_temp = 1000.0
    max_temp = -1000.0

    # ボタン状態の保存（エッジ検出用）
    last_screen_button = screen_button.value()
    last_sound_button = sound_stop_button.value()
    restart_start_time = None  # 同時押し再起動用タイマー

    sound_stop = False  # 音停止状態フラグ

    while True:
        current_time = ticks_ms()
        dt = (current_time - last_time) / 1000.0
        last_time = current_time

        # 現在のボタン状態を取得
        current_screen_button = screen_button.value()
        current_sound_button = sound_stop_button.value()

        # 画面モード切替：GP27 のエッジ検出で、1→2→3→1…と切替
        if last_screen_button == 1 and current_screen_button == 0:
            screen_mode = (screen_mode % 3) + 1
            sleep(DEBOUNCE_DELAY)
        last_screen_button = current_screen_button

        # 音停止ボタン：GP21 のエッジ検出で sound_stop フラグをセット
        if last_sound_button == 1 and current_sound_button == 0:
            sound_stop = True
            sleep(DEBOUNCE_DELAY)
        last_sound_button = current_sound_button

        # 同時押しによる再起動の判定：両方のボタンが押されていればタイマー開始
        if current_screen_button == 0 and current_sound_button == 0:
            if restart_start_time is None:
                restart_start_time = current_time
            elif current_time - restart_start_time >= 2000:
                oled.fill(0)
                oled.text("Restarting...", 0, OLED_HEIGHT // 2)
                oled.show()
                sleep(1)
                reset()
        else:
            restart_start_time = None

        # センサー値の取得と角度計算
        accel = mpu.get_accel_data()
        accel_angle_x = degrees(atan2(accel['y'], accel['z']))
        accel_angle_y = degrees(atan2(-accel['x'], accel['z']))
        gyro = mpu.get_gyro_data()
        gyro['x'] -= gyro_offset_x
        gyro['y'] -= gyro_offset_y
        angle_x += gyro['x'] * dt
        angle_y += gyro['y'] * dt
        angle_x = ALPHA * angle_x + (1 - ALPHA) * accel_angle_x
        angle_y = ALPHA * angle_y + (1 - ALPHA) * accel_angle_y

        display_x = abs(int(angle_x))
        display_y = abs(int(angle_y))
        is_flat = (display_x < FLAT_THRESHOLD and display_y < FLAT_THRESHOLD)

        oled.fill(0)
        if screen_mode == 1:
            update_display_text(oled, angle_x, angle_y, is_flat, accel)
        elif screen_mode == 2:
            update_display_bubble(oled, angle_x, angle_y, is_flat)
        elif screen_mode == 3:
            temp = get_temperature(mpu)
            if temp < min_temp:
                min_temp = temp
            if temp > max_temp:
                max_temp = temp
            update_display_temp(oled, temp, min_temp, max_temp)
            buzzer.duty_u16(0)

        # ブザー制御（温度画面以外）
        if screen_mode in (1, 2):
            # 起動直後（STARTUP_BEEP_DELAY未満）はFLATでも音を鳴らさない
            if is_flat and not sound_stop and (current_time - startup_time >= STARTUP_BEEP_DELAY):
                buzzer.freq(5000)
                buzzer.duty_u16(65535)
            else:
                buzzer.duty_u16(0)
            if not is_flat:
                sound_stop = False

        oled.show()
        sleep(0.1)

if __name__ == "__main__":
    main()
