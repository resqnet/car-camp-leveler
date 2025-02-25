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

def update_display_text(oled, angle_x, angle_y, is_flat):
    """テキスト表示モードの画面更新"""
    oled.text("X-Angle:", 0, 20)
    oled.text(f"{int(angle_x)} deg", 80, 20)
    oled.text("Y-Angle:", 0, 35)
    oled.text(f"{int(angle_y)} deg", 80, 35)
    # 左下に状態表示（FLAT または Tilted）
    if is_flat:
        oled.text("FLAT", 0, OLED_HEIGHT - 10)
    else:
        oled.text("Tilted", 0, OLED_HEIGHT - 10)

def update_display_bubble(oled, angle_x, angle_y, is_flat):
    """バブルレベル表示モードの画面更新"""
    oled.hline(0, OLED_HEIGHT // 2, OLED_WIDTH, 1)
    oled.vline(OLED_WIDTH // 2, 0, OLED_HEIGHT, 1)
    bubble_x = int((OLED_WIDTH // 2) - angle_x * 2)
    bubble_y = int((OLED_HEIGHT // 2) + angle_y * 2)
    draw_circle(oled, bubble_x, bubble_y, 2)
    # 左下に状態表示
    if is_flat:
        oled.text("FLAT", 0, OLED_HEIGHT - 10)
    else:
        oled.text("Tilted", 0, OLED_HEIGHT - 10)

def main():
    # 初期化処理
    i2c_mpu, i2c_oled = init_i2c()
    mpu, oled, buzzer, screen_button, sound_stop_button = init_devices(i2c_mpu, i2c_oled)
    show_startup_screen(oled)
    print("Car-Camp-Leveler 起動中...")
    gyro_offset_x, gyro_offset_y = calibrate_gyro(mpu)

    # 初期状態の設定
    angle_x = 0
    angle_y = 0
    last_time = ticks_ms()
    screen_mode = 1   # 1: テキスト表示, 2: バブルレベル表示

    # ボタン状態の保存（エッジ検出用）
    last_screen_button = screen_button.value()
    last_sound_button = sound_stop_button.value()

    sound_stop = False  # 音停止状態フラグ

    while True:
        # ボタン状態の現在値を取得
        current_screen_button = screen_button.value()
        current_sound_button = sound_stop_button.value()

        # 画面モード切替: GP27 のエッジ検出でトグル
        if last_screen_button == 1 and current_screen_button == 0:
            screen_mode = 2 if screen_mode == 1 else 1
            sleep(DEBOUNCE_DELAY)
        last_screen_button = current_screen_button

        # 音停止ボタン: GP21 のエッジ検出で sound_stop フラグをセット
        if last_sound_button == 1 and current_sound_button == 0:
            sound_stop = True
            sleep(DEBOUNCE_DELAY)
        last_sound_button = current_sound_button

        # 経過時間の計算
        current_time = ticks_ms()
        dt = (current_time - last_time) / 1000.0
        last_time = current_time

        # センサー値の取得と角度計算
        accel = mpu.get_accel_data()
        accel_angle_x = degrees(atan2(accel['y'], accel['z']))
        accel_angle_y = degrees(atan2(-accel['x'], accel['z']))

        gyro = mpu.get_gyro_data()
        # キャリブレーションによるオフセット補正
        gyro['x'] -= gyro_offset_x
        gyro['y'] -= gyro_offset_y

        # ジャイロによる角度積分
        angle_x += gyro['x'] * dt
        angle_y += gyro['y'] * dt

        # コンプリメントフィルタによる融合
        angle_x = ALPHA * angle_x + (1 - ALPHA) * accel_angle_x
        angle_y = ALPHA * angle_y + (1 - ALPHA) * accel_angle_y

        display_x = abs(int(angle_x))
        display_y = abs(int(angle_y))
        is_flat = (display_x < FLAT_THRESHOLD and display_y < FLAT_THRESHOLD)

        # OLED 画面の更新（テキスト／バブル表示切替）
        oled.fill(0)
        if screen_mode == 1:
            update_display_text(oled, angle_x, angle_y, is_flat)
        elif screen_mode == 2:
            update_display_bubble(oled, angle_x, angle_y, is_flat)

        # Buzzer 制御および音停止フラグのリセット
        if is_flat:
            if not sound_stop:
                buzzer.freq(5000)
                buzzer.duty_u16(65535)
            else:
                buzzer.duty_u16(0)
        else:
            # Tilted 状態になったら、音停止フラグを解除（再び音が出る状態に）
            buzzer.duty_u16(0)
            sound_stop = False

        oled.show()
        sleep(0.1)

if __name__ == "__main__":
    main()
